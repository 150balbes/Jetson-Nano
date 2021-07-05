/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/cpu_pm.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/bpmp_abi.h>
#include <linux/delay.h>
#include <linux/pstore.h>
#include <linux/ptrace.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/tegra18_cpu_map.h>
#include <linux/tegra-mce.h>
#include <linux/tegra-cpu.h>
#include <linux/version.h>
#include <linux/pm_qos.h>
#include <linux/tegra-cpufreq.h>
#include <soc/tegra/chip-id.h>

#include "cpufreq_cpu_emc_table.h"

#define MAX_NDIV		512 /* No of NDIV */
#define MAX_VINDEX		80 /* No of voltage index */
/* cpufreq transisition latency */
#define TEGRA_CPUFREQ_TRANSITION_LATENCY	(300 * 1000)

#define KHZ_TO_HZ		1000
#define REF_CLK_MHZ		408 /* 408 MHz */
#define US_DELAY		20
#define CPUFREQ_TBL_STEP_SIZE	4

#define CLUSTER_STR(cl)	(cl ==  1 ?\
				"B_CLUSTER" : "M_CLUSTER")
#define LOOP_FOR_EACH_CLUSTER(cl)	for (cl = 0; cl < MAX_CLUSTERS; cl++)
#define INDEX_STEP	2
/* EDVD register details */
#define EDVD_CL_NDIV_VHINT_OFFSET	0x20
#define EDVD_COREX_NDIV_VAL_SHIFT	(0)
#define EDVD_COREX_NDIV_MASK		(0x1ff << 0)
#define EDVD_COREX_VINDEX_VAL_SHIFT	(16)
#define EDVD_COREX_VINDEX_MASK		(0xff << 16)

/* ACTMON counter register details */
#define CORECLK_OFFSET			(0x0)
#define REFCLK_OFFSET			(0x4)
#define REG_OFFSET			(0x4)
#define REF_CLOCK_MASK			(0xfffffff)
#define coreclk_base(base, cpu)	(base + CORECLK_OFFSET \
					+ (REG_OFFSET * cpu))
#define refclk_base(base, cpu)		(base + REFCLK_OFFSET \
					+ (REG_OFFSET * cpu))
#define tcpufreq_readl(base, cpu)	readl((void __iomem *) \
					base + \
					(REG_OFFSET * cpu))
#define tcpufreq_writel(val, base, cpu)	writel(val, base + \
					(REG_OFFSET * cpu))
#define logical_to_phys_map(cpu)	(MPIDR_AFFINITY_LEVEL \
					(cpu_logical_map(cpu), 0))
#define logical_to_phys_cluster(cl)	(cl == 1 ? \
					ARM_CPU_IMP_ARM : \
					ARM_CPU_IMP_NVIDIA)

#define is_cluster_present(cl)		(cl >= MAX_CLUSTERS ? \
					false : \
				tfreq_data.pcluster[cl].cluster_present)
#define M_CLUSTER			0
#define B_CLUSTER			1
#define MAX_CLUSTERS			2

#define M_CL_OFFSET		 (SZ_64K * 7)
#define B_CL_OFFSET		 (SZ_64K * 6)

/**
 * Cpu side dvfs table
 * This table needs to be constructed at boot up time
 * BPMP will provide NDIV and Vidx tuple.
 * BPMP will also provide per custer Pdiv, Mdiv, ref_clk.
 * freq = (ndiv * refclk) / (pdiv * mdiv)
*/
struct cpu_vhint_table {
	struct cpu_vhint_data *lut; /* virtual address of NDIV[VINDEX] */
	dma_addr_t phys;
	uint32_t ref_clk_hz;
	uint16_t pdiv; /* post divider */
	uint16_t mdiv; /* input divider */
	uint16_t vfloor;
	uint16_t vceil;
	uint16_t ndiv_max;
	uint16_t ndiv_min;
	uint16_t vindex_mult;
	uint16_t vindex_div;
	uint8_t *vindx;
};

struct cc3_params {
	u32 ndiv;
	u32 vindex;
	u32 freq;
	u8 enable;
};

struct per_cluster_data {
	struct cpufreq_frequency_table *clft;
	void __iomem *edvd_pub;
	struct cpu_vhint_table dvfs_tbl;
	struct tegra_bwmgr_client *bwmgr;
	struct cpumask cpu_mask;
	struct cc3_params cc3;
	bool cluster_present;
};

struct tegra_cpufreq_data {
	struct per_cluster_data pcluster[MAX_CLUSTERS];
	struct mutex mlock; /* lock protecting below params */
	uint32_t freq_compute_delay; /* delay in reading clock counters */
	uint32_t cpu_freq[CONFIG_NR_CPUS];
	uint32_t last_hint[CONFIG_NR_CPUS];
	void *__iomem *regs;
};

static struct tegra_cpufreq_data tfreq_data;
static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static DEFINE_PER_CPU(struct mutex, pcpu_mlock);
static DEFINE_PER_CPU(spinlock_t, pcpu_slock);

static bool tegra_cpufreq_hv_mode;

struct tegra_cpu_ctr {
	uint32_t cpu;
	uint32_t coreclk_cnt, last_coreclk_cnt;
	uint32_t refclk_cnt, last_refclk_cnt;
};

static void tegra_read_counters(void *arg)
{
	struct cpuinfo_arm64 *cpuinfo;
	struct tegra_cpu_ctr *c = arg;
	u32 mpidr, midr;

	mpidr = cpu_logical_map(c->cpu);
	cpuinfo = &per_cpu(cpu_data, c->cpu);
	midr = cpuinfo->reg_midr;

	/*
	 * ref_clk_counter(28 bit counter) runs from constant clk,
	 * pll_p(408MHz).
	 * It will take = 2 ^ 28 / 408 MHz to overflow ref clk counter
	 *              = 65793 usec = 657 msec to overflow
	 *
	 * Like wise core_clk_counter(32 bit counter) runs from
	 * crab_clk(ctu_clk). ctu_clk, runs at full freq of cluster,
	 * Assuming max cluster clock ~2000MHz
	 * It will take = 2 ^ 32 / 2000 MHz to overflow core clk counter
	 *              = 2 sec to overflow
	 *
	 * Unsigned subtraction of core clock counter(32 bit) and ref clk
	 * counter(28 bit) with modulo of 2^28 avoids single overflow.
	*/

	if (tegra_get_clk_counter(mpidr, midr, &c->last_coreclk_cnt,
		&c->last_refclk_cnt))
		pr_err("Error in reading cpu clk counters before delay %u: usec\n",
			tfreq_data.freq_compute_delay);

	udelay(tfreq_data.freq_compute_delay);

	if (tegra_get_clk_counter(mpidr, midr, &c->coreclk_cnt,
		&c->refclk_cnt))
		pr_err("Error in reading cpu clk counters after delay %u: usec\n",
			tfreq_data.freq_compute_delay);
}

/**
 * Return instantaneous cpu speed
 * Instantaneous freq is calculated as -
 * -Takes sample on every query of getting the freq.
 *        - Read core and ref clock counters;
 *        - Delay for X us
 *       -  Read above cycle counters again
 *       - Calculates freq by subtracting current and previous counters
 *          divided by the delay time or eqv. of ref_clk_counter in delta time
 *       - Return Kcycles/second, freq in KHz
 *
 * - delta time period = x sec
 *          = delta ref_clk_counter / (408 * 10^6) sec
 * freq in Hz = cycles/sec
 *                 = (delta cycles / x sec
 *                 = (delta cycles * 408 * 10^6) / delta ref_clk_counter
 *     in KHz = (delta cycles * 408 * 10^3) / delta ref_clk_counter
 *
 * @cpu - logical cpu whose freq to be updated
 * Returns freq in KHz on success, 0 if cpu is offline
 */
static unsigned int tegra186_get_speed(uint32_t cpu)
{
	uint32_t delta_ccnt = 0;
	uint32_t delta_refcnt = 0;
	unsigned long rate_mhz = 0;
	struct tegra_cpu_ctr c;

	c.cpu = cpu;
	if (!smp_call_function_single(cpu, tegra_read_counters, &c, 1)) {
		delta_ccnt = c.coreclk_cnt - c.last_coreclk_cnt;
		if (!delta_ccnt)
			goto err_out;

		/* ref clock is 28 bits */
		delta_refcnt = (c.refclk_cnt - c.last_refclk_cnt) % (1 << 28);
		if (!delta_refcnt) {
			pr_err("Warning: %d is idle, delta_refcnt: 0\n", cpu);
			goto err_out;
		}

		rate_mhz = ((unsigned long) delta_ccnt * REF_CLK_MHZ)
								/ delta_refcnt;
	}
err_out:
	return (unsigned int) (rate_mhz * 1000); /* in KHz */
}

static struct cpu_emc_mapping dflt_t186_cpu_emc_mapping[] = {
	{ 450000,   408000},
	{ 800000,   660000},
	{1400000, UINT_MAX},
	{}, /* termination entry */
};

static struct cpu_emc_mapping *cpu_emc_mapping_dt;

/**
 * get_cluster_freq - returns max freq among all the cpus in a cluster.
 *
 * @cl - cluster whose freq to be returned
 * @freq - cpu freq in kHz
 * Returns:
 *         cluster freq as max freq among all the cpu's freq in
 *         a cluster
 */
static uint32_t get_cluster_freq(struct cpufreq_policy *policy)
{
	uint32_t i, cpu_freq;
	struct cpumask mask;

	cpumask_and(&mask, policy->cpus, cpu_online_mask);
	cpu_freq = policy->cur;
	for_each_cpu(i, &mask) {
		cpu_freq = max(cpu_freq, tfreq_data.cpu_freq[i]);
	}

	return cpu_freq;
}

/* Set emc clock by referring cpu_to_emc freq mapping */
static void set_cpufreq_to_emcfreq(struct cpufreq_policy *policy)
{
	unsigned long emc_freq, freq_khz;
	uint32_t cluster_freq;
	int cl;
	struct cpu_emc_mapping *mapping = cpu_emc_mapping_dt ?
		cpu_emc_mapping_dt : dflt_t186_cpu_emc_mapping;

	cluster_freq = get_cluster_freq(policy);

	freq_khz = tegra_cpu_to_emc_freq(cluster_freq, mapping);
	if (freq_khz == UINT_MAX)
		emc_freq = tegra_bwmgr_get_max_emc_rate();
	else
		emc_freq = freq_khz * KHZ_TO_HZ;

	cl = tegra18_logical_to_cluster(policy->cpu);
	tegra_bwmgr_set_emc(tfreq_data.pcluster[cl].bwmgr, emc_freq,
		TEGRA_BWMGR_SET_EMC_FLOOR);
	pr_debug("cpu: %d, cluster %s, emc freq(KHz): %lu cluster_freq(kHz): %u\n",
		policy->cpu, CLUSTER_STR(cl), emc_freq / 1000, cluster_freq);
}

static struct cpufreq_frequency_table *get_freqtable(uint8_t cpu)
{
	int cur_cl = tegra18_logical_to_cluster(cpu);

	return tfreq_data.pcluster[cur_cl].clft;
}

/**
 * tegra_update_cpu_speed - update cpu freq
 * @rate - in kHz
 * @cpu - cpu whose freq to be updated
 * Returns 0 on success, -ve on failure
 */
static void tegra_update_cpu_speed(uint32_t rate, uint8_t cpu)
{
	struct cpu_vhint_table *vhtbl;
	uint32_t val = 0, phy_cpu;
	int cur_cl;
	uint16_t ndiv;
	int8_t vindx;
	spinlock_t *slock = &per_cpu(pcpu_slock, cpu);

	cur_cl = tegra18_logical_to_cluster(cpu);
	vhtbl = &tfreq_data.pcluster[cur_cl].dvfs_tbl;

	/*
	 * LUT for this cluster is not present.
	 * Could be single cluster or n cluster chip but for <cur_cl>,
	 * current cluster LUT is not sent by BPMP.
	 */
	if (!vhtbl->lut)
		return;

	rate *= vhtbl->pdiv * vhtbl->mdiv;
	ndiv = (rate * KHZ_TO_HZ) / vhtbl->ref_clk_hz;
	if ((rate * KHZ_TO_HZ) % vhtbl->ref_clk_hz)
		ndiv++;

	if (ndiv < vhtbl->ndiv_min)
		ndiv = vhtbl->ndiv_min;
	if (ndiv > vhtbl->ndiv_max)
		ndiv = vhtbl->ndiv_max;

	val |= (ndiv << EDVD_COREX_NDIV_VAL_SHIFT);
	vindx = vhtbl->vindx[ndiv];
	if (vindx < vhtbl->vfloor)
		vindx = vhtbl->vfloor;
	else if (vindx > vhtbl->vceil)
		vindx = vhtbl->vceil;
	if (vhtbl->vindex_div > 0)
		vindx = vhtbl->vindex_mult * vindx / vhtbl->vindex_div;

	val |= (vindx << EDVD_COREX_VINDEX_VAL_SHIFT);
	phy_cpu = logical_to_phys_map(cpu);

	spin_lock(slock);
	tcpufreq_writel(val, tfreq_data.pcluster[cur_cl].edvd_pub +
		EDVD_CL_NDIV_VHINT_OFFSET, phy_cpu);
	tfreq_data.last_hint[cpu] = val;
	spin_unlock(slock);
}
/**
 * tegra186_cpufreq_set_target - Request freq to be set for policy->cpu
 * @policy - cpufreq policy per cpu
 * @index - freq table index
 * Returns 0 on success, -ve on failure
 */
static int tegra186_cpufreq_set_target(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct cpufreq_freqs freqs;
	struct mutex *mlock;
	uint32_t tgt_freq;
	int cpu, ret = 0;

	if (is_tegra_hypervisor_mode() && hv_is_set_speed_supported() == false) {
		pr_warn("cpufreq : Setting speed functionality not present\n");
		return -EINVAL;
	}

	if (!policy || (!cpu_online(policy->cpu)))
		return -EINVAL;

	mlock = &per_cpu(pcpu_mlock, policy->cpu);
	mutex_lock(mlock);

	tgt_freq = policy->freq_table[index].frequency;
	freqs.old = tfreq_data.cpu_freq[policy->cpu];

	if (policy->cur == tgt_freq)
		goto out;

	freqs.new = tgt_freq;

	cpufreq_freq_transition_begin(policy, &freqs);

	if (freqs.old != tgt_freq) {
		/*
		 * In hypervisor case cpufreq server will take care of
		 * updating frequency for each cpu in a cluster. So no
		 * need to run through the loop.
		 */
		if (tegra_cpufreq_hv_mode)
			tegra_update_cpu_speed_hv(tgt_freq, policy->cpu);

		for_each_cpu(cpu, policy->cpus) {
			if (!tegra_cpufreq_hv_mode)
				tegra_update_cpu_speed(tgt_freq, cpu);
			/*
			 * Update the freq data for each cpu regardless
			 * hypervisor or native mode.
			 */
			tfreq_data.cpu_freq[cpu] = tgt_freq;
		}
	}
	policy->cur = tgt_freq;
	freqs.new = policy->cur;

	if (freqs.old != tgt_freq)
		set_cpufreq_to_emcfreq(policy);

	cpufreq_freq_transition_end(policy, &freqs, ret);
out:
	pr_debug("cpu: %d, oldfreq(kHz): %d, req freq(kHz): %d final freq(kHz): %d tgt_index %u\n",
		policy->cpu, freqs.old, tgt_freq, policy->cur, index);
	mutex_unlock(mlock);
	return ret;
}

static void __tegra_mce_cc3_ctrl(void *data)
{
	struct cc3_params *param = (struct cc3_params *)data;

	tegra_mce_cc3_ctrl(param->ndiv, param->vindex, param->enable);
}

static inline u16 map_ndiv_to_vindex(struct cpu_vhint_table *vhtbl, u16 ndiv)
{
	struct cpu_vhint_data *lut = vhtbl->lut;

	if (ndiv == lut->ndiv_min)
		return lut->vfloor;
	else
		return vhtbl->vindx[ndiv];
}

static inline u16 clamp_ndiv(struct cpu_vhint_table *vhtbl, u16 ndiv)
{
	u16 min = vhtbl->lut->ndiv_min;
	u16 max = vhtbl->lut->ndiv_max;

	if (!ndiv || (ndiv < min))
		ndiv = min;
	if (ndiv > max)
		ndiv = max;

	return ndiv;
}

static inline u16 map_freq_to_ndiv(struct cpu_vhint_table *vhtbl,
	u32 freq)
{
	struct cpu_vhint_data *lut = vhtbl->lut;

	return (freq * 1000 * lut->pdiv * lut->mdiv) / lut->ref_clk_hz;
}

static void enable_cc3(struct device_node *dn)
{
	struct cpu_vhint_table *vhtbl;
	struct cc3_params *cc3;
	u32 enb, freq = 0, idx = 0;
	u16 ndiv;
	int cl;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		cc3 = &tfreq_data.pcluster[cl].cc3;

		if (!vhtbl->lut)
			goto idx_inc;

		ret = of_property_read_u32_index(dn, "nvidia,enable-autocc3",
			idx + 1, &enb);
		if (!enb || ret)
			goto idx_inc;

		ret = of_property_read_u32_index(dn, "nvidia,autocc3-freq",
			idx + 1, &freq);
		if (ret)
			freq = 0;

		ndiv = map_freq_to_ndiv(vhtbl, freq);
		ndiv = clamp_ndiv(vhtbl, ndiv);
		cc3->enable = 1;
		cc3->ndiv = ndiv;
		cc3->vindex = map_ndiv_to_vindex(vhtbl, ndiv);

		ret = smp_call_function_any(&tfreq_data.pcluster[cl].cpu_mask,
				__tegra_mce_cc3_ctrl,
				cc3, 1);
		WARN_ON_ONCE(ret);
idx_inc:
		idx += INDEX_STEP;
	}
}

#ifdef CONFIG_DEBUG_FS
#define RW_MODE			(S_IWUSR | S_IRUGO)
#define RO_MODE			(S_IRUGO)

static int get_delay(void *data, u64 *val)
{
	mutex_lock(&tfreq_data.mlock);

	*val = tfreq_data.freq_compute_delay;

	mutex_unlock(&tfreq_data.mlock);
	return 0;
}

static int set_delay(void *data, u64 val)
{
	uint32_t udelay = val;

	mutex_lock(&tfreq_data.mlock);

	if (udelay)
		tfreq_data.freq_compute_delay = udelay;

	mutex_unlock(&tfreq_data.mlock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(freq_compute_fops, get_delay, set_delay,
	"%llu\n");

static int freq_get(void *data, u64 *val)
{
	uint64_t cpu = (uint64_t)data;
	struct mutex *mlock;

	mlock = &per_cpu(pcpu_mlock, cpu);
	mutex_lock(mlock);

	*val = tegra186_get_speed(cpu);

	mutex_unlock(mlock);
	return 0;
}

/* Set freq in Khz for a cpu  */
static int freq_set(void *data, u64 val)
{
	uint64_t cpu = (uint64_t)data;
	unsigned int freq = val;
	struct mutex *mlock;

	mlock = &per_cpu(pcpu_mlock, cpu);
	mutex_lock(mlock);

	if (val) {
		if (tegra_cpufreq_hv_mode)
			tegra_update_cpu_speed_hv(freq, cpu);
		else
			tegra_update_cpu_speed(freq, cpu);
	}

	mutex_unlock(mlock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(freq_fops, freq_get, freq_set, "%llu\n");

/* Set ndiv / vindex hint for a cpu */
static int set_hint(void *data, u64 val)
{
	uint64_t cpu = (uint64_t)data;
	int cur_cl;
	uint32_t hint = val;

	if (!val)
		return 0;

	/* Take hotplug lock before taking tegra cpufreq lock */
	get_online_cpus();
	if (cpu_online(cpu)) {
		spinlock_t *slock = &per_cpu(pcpu_slock, cpu);
		cur_cl = tegra18_logical_to_cluster(cpu);
		cpu = logical_to_phys_map(cpu);
		spin_lock(slock);
		tcpufreq_writel(hint, tfreq_data.pcluster[cur_cl].edvd_pub +
			EDVD_CL_NDIV_VHINT_OFFSET, cpu);
		tfreq_data.last_hint[cpu] = hint;
		spin_unlock(slock);
	}
	put_online_cpus();
	return 0;
}

/* get ndiv / vindex hint for a cpu */
static int get_hint(void *data, u64 *hint)
{
	uint64_t cpu = (uint64_t)data;
	int cur_cl;

	*hint = 0;

	/* Take hotplug lock before taking tegra cpufreq lock */
	get_online_cpus();
	if (cpu_online(cpu)) {
		cur_cl = tegra18_logical_to_cluster(cpu);
		cpu = logical_to_phys_map(cpu);
		pstore_rtrace_set_bypass(1);
		*hint = tcpufreq_readl(tfreq_data.pcluster[cur_cl].edvd_pub +
			EDVD_CL_NDIV_VHINT_OFFSET, cpu);
		pstore_rtrace_set_bypass(0);
	}
	put_online_cpus();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ndiv_vindex_fops, get_hint, set_hint, "%08llx\n");

static void dump_lut(struct seq_file *s, struct cpu_vhint_table *vht)
{
	uint16_t i;

	seq_printf(s, "reference clk(hz): %u\n", vht->ref_clk_hz);
	seq_printf(s, "pdiv: %u\n", vht->pdiv);
	seq_printf(s, "mdiv: %u\n", vht->mdiv);
	seq_printf(s, "vfloor: %u\n", vht->vfloor);
	seq_printf(s, "vceil: %u\n", vht->vceil);
	seq_printf(s, "ndiv_max: %u\n", vht->ndiv_max);
	seq_printf(s, "ndiv_min: %u\n", vht->ndiv_min);
	seq_printf(s, "vindex_mult: %u\n", vht->vindex_mult);
	seq_printf(s, "vindex_div: %u\n", vht->vindex_div);
	for (i = vht->ndiv_min; i <= vht->ndiv_max; i++)
		seq_printf(s, "vindex[ndiv==%u]: %u\n", i, vht->vindx[i]);
	seq_puts(s, "\n");
}

static int show_bpmp_to_cpu_lut(struct seq_file *s, void *data)
{
	struct cpu_vhint_table *vht;
	int cl;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vht = &tfreq_data.pcluster[cl].dvfs_tbl;

		/*
		 * LUT for this cluster is not present.
		 * Could be single cluster or n cluster chip but for <cl>,
		 * current cluster, LUT is not sent by BPMP.
		 */
		if (!vht->lut)
			continue;

		seq_printf(s, "%s:\n", CLUSTER_STR(cl));
		dump_lut(s, vht);
	}

	return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_bpmp_to_cpu_lut, inode->i_private);
}

static const struct file_operations lut_fops = {
	.open = stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int get_pcluster_cc3(void *data, u64 *val)
{
	long cl = (long)data;

	mutex_lock(&tfreq_data.mlock);

	*val = tfreq_data.pcluster[cl].cc3.enable;

	mutex_unlock(&tfreq_data.mlock);

	return 0;
}

static int set_pcluster_cc3(void *data, u64 val)
{
	long cl = (long)data;
	int wait = 1;
	int ret = 0;

	mutex_lock(&tfreq_data.mlock);

	if (tfreq_data.pcluster[cl].cc3.enable ^ (bool) val) {
		tfreq_data.pcluster[cl].cc3.enable = (bool) val;
		ret = smp_call_function_any(&tfreq_data.pcluster[cl].cpu_mask,
				__tegra_mce_cc3_ctrl,
				&tfreq_data.pcluster[cl].cc3, wait);
	}

	mutex_unlock(&tfreq_data.mlock);
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(pcl_cc3_ops, get_pcluster_cc3,
	set_pcluster_cc3, "%llu\n");

static int get_ndiv(void *data, u64 *val)
{
	long cl = (long)data;

	mutex_lock(&tfreq_data.mlock);

	*val = tfreq_data.pcluster[cl].cc3.ndiv;

	mutex_unlock(&tfreq_data.mlock);

	return 0;
}

static int set_ndiv(void *data, u64 val)
{
	long cl = (long)data;
	int wait = 1;
	int ret = 0;

	mutex_lock(&tfreq_data.mlock);

	if (tfreq_data.pcluster[cl].cc3.ndiv != (u32) val) {
		tfreq_data.pcluster[cl].cc3.ndiv = (u32) val;
		ret = smp_call_function_any(&tfreq_data.pcluster[cl].cpu_mask,
				__tegra_mce_cc3_ctrl,
				&tfreq_data.pcluster[cl].cc3, wait);
	}

	mutex_unlock(&tfreq_data.mlock);
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(ndiv_ops, get_ndiv, set_ndiv,
	"%llu\n");

static int get_vindex(void *data, u64 *val)
{
	long cl = (long)data;

	mutex_lock(&tfreq_data.mlock);

	*val = tfreq_data.pcluster[cl].cc3.vindex;

	mutex_unlock(&tfreq_data.mlock);

	return 0;
}

static int set_vindex(void *data, u64 val)
{
	long cl = (long)data;
	int wait = 1;
	int ret = 0;

	mutex_lock(&tfreq_data.mlock);

	if (tfreq_data.pcluster[cl].cc3.vindex != (u32) val) {
		tfreq_data.pcluster[cl].cc3.vindex = (u32) val;
		ret = smp_call_function_any(&tfreq_data.pcluster[cl].cpu_mask,
				__tegra_mce_cc3_ctrl,
				&tfreq_data.pcluster[cl].cc3, wait);
	}

	mutex_unlock(&tfreq_data.mlock);
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(vidx_ops, get_vindex, set_vindex,
	"%llu\n");

static struct dentry *tegra_cpufreq_debugfs_root;
static int __init cc3_debug_init(void)
{
	struct dentry *dir;
	long int cl;
	uint8_t buff[15];

	LOOP_FOR_EACH_CLUSTER(cl) {
		snprintf(buff, sizeof(buff), CLUSTER_STR(cl));
		dir = debugfs_create_dir(buff, tegra_cpufreq_debugfs_root);
		if (!dir)
			goto err_out;

		snprintf(buff, sizeof(buff), "cc3");
		dir = debugfs_create_dir(buff, dir);
		if (!dir)
			goto err_out;

		if (!debugfs_create_file("enable", RW_MODE, dir, (void *)cl,
			&pcl_cc3_ops))
			goto err_out;

		if (!debugfs_create_file("ndiv", RW_MODE, dir, (void *)cl,
			&ndiv_ops))
			goto err_out;
		if (!debugfs_create_file("vindex", RW_MODE, dir, (void *)cl,
			&vidx_ops))
			goto err_out;
	}
	return 0;

err_out:
	return -EINVAL;
}

static int __init tegra_cpufreq_debug_init(void)
{
	struct dentry *dir;
	uint8_t buff[15];
	uint64_t cpu;

	tegra_cpufreq_debugfs_root = debugfs_create_dir("tegra_cpufreq", NULL);
	if (!tegra_cpufreq_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file("bpmp_cpu_vhint_table", RO_MODE,
				 tegra_cpufreq_debugfs_root,
					NULL,
					&lut_fops))
		goto err_out;

	if (!debugfs_create_file("freq_compute_delay", RW_MODE,
				 tegra_cpufreq_debugfs_root,
					NULL,
					&freq_compute_fops))
		goto err_out;

	if (cc3_debug_init())
		goto err_out;

	if (!tegra_debugfs_create_cpu_emc_map(tegra_cpufreq_debugfs_root,
		cpu_emc_mapping_dt ? cpu_emc_mapping_dt :
		dflt_t186_cpu_emc_mapping))
		goto err_out;

	for_each_possible_cpu(cpu) {
		snprintf(buff, sizeof(buff), "cpu%llu", cpu);
		dir = debugfs_create_dir(buff, tegra_cpufreq_debugfs_root);
		if (!dir)
			goto err_out;
		if (!debugfs_create_file("freq", RW_MODE, dir, (void *)cpu,
			&freq_fops))
			goto err_out;
		if (!debugfs_create_file("ndiv_vindex_hint", RW_MODE, dir,
			(void *)cpu, &ndiv_vindex_fops))
			goto err_out;
	}
	return 0;

err_out:
	debugfs_remove_recursive(tegra_cpufreq_debugfs_root);
	return -ENOMEM;
}

static void tegra_cpufreq_debug_exit(void)
{
	debugfs_remove_recursive(tegra_cpufreq_debugfs_root);
}
#endif

static int tegra186_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *ftbl;
	struct mutex *mlock;

	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	mlock = &per_cpu(pcpu_mlock, policy->cpu);
	mutex_lock(mlock);

	ftbl = get_freqtable(policy->cpu);

	cpufreq_table_validate_and_show(policy, ftbl);

	policy->cur = tegra186_get_speed(policy->cpu);
	tfreq_data.cpu_freq[policy->cpu] = policy->cur;

	set_cpufreq_to_emcfreq(policy);

	policy->cpuinfo.transition_latency =
	TEGRA_CPUFREQ_TRANSITION_LATENCY;

	cpumask_copy(policy->cpus, topology_core_cpumask(policy->cpu));

	mutex_unlock(mlock);

	return 0;
}

static int tegra186_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct mutex *mlock;
	int cl;

	mlock = &per_cpu(pcpu_mlock, policy->cpu);
	mutex_lock(mlock);

	cpufreq_frequency_table_cpuinfo(policy, policy->freq_table);

	cl = tegra18_logical_to_cluster(policy->cpu);
	if (tfreq_data.pcluster[cl].bwmgr)
		tegra_bwmgr_set_emc(tfreq_data.pcluster[cl].bwmgr, 0,
			TEGRA_BWMGR_SET_EMC_FLOOR);

	mutex_unlock(mlock);
	return 0;
}

static struct cpufreq_driver tegra_cpufreq_driver = {
	.name		= "tegra_cpufreq",
	.flags		= CPUFREQ_ASYNC_NOTIFICATION | CPUFREQ_STICKY |
				CPUFREQ_CONST_LOOPS |
				CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= tegra186_cpufreq_set_target,
	.get		= tegra186_get_speed,
	.init		= tegra186_cpufreq_init,
	.exit		= tegra186_cpufreq_exit,
	.attr		= tegra_cpufreq_attr,
};

static int cluster_freq_notify(struct notifier_block *b,
			unsigned long l, void *v)
{
	struct cpufreq_policy *policy;
	struct cpumask updated_cpus;
	int cpu, ret = 0;

	cpumask_clear(&updated_cpus);

	for_each_online_cpu(cpu) {
		/* Skip CPUs already covered by a previous update */
		if (cpumask_test_cpu(cpu, &updated_cpus))
			continue;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		cpufreq_update_policy(policy->cpu);
		cpumask_or(&updated_cpus, &updated_cpus, policy->cpus);
		cpufreq_cpu_put(policy);
	}

	return notifier_from_errno(ret);
}

/* Clipping policy object's min/max to pmqos limits */
static int tegra_boundaries_policy_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int qmin = 0;
	unsigned int qmax = UINT_MAX;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	qmin = pm_qos_read_min_bound(PM_QOS_CPU_FREQ_BOUNDS);
	qmax = pm_qos_read_max_bound(PM_QOS_CPU_FREQ_BOUNDS);

	/*
	 * Clamp pmqos to stay within sysfs upper boundary
	 * but allow pmqos cap override sysfs min freq settings
	 */
	qmin = min(qmin, policy->user_policy.max);
	qmax = min(qmax, policy->user_policy.max);

	/* Apply pmqos limits on top of existing limits */
	policy->min = max(policy->min, qmin);
	policy->max = min(policy->max, qmax);

	if (policy->min > policy->max)
		policy->min = policy->max;

	return NOTIFY_OK;
}

#ifdef CONFIG_CPU_PM
static int tegra_cpu_pm_notifier(struct notifier_block *nb,
		unsigned long cmd, void *v)
{
	unsigned int cpu = smp_processor_id();

	switch (cmd) {
	case CPU_PM_EXIT:
		/*
		 * Only Denver can enter CC6 and lose the subsequent
		 * VF request at runtime. So no need to do this for
		 * ARM cores for now.
		 */
		if (!tegra18_is_cpu_denver(cpu))
			break;

		if (tfreq_data.last_hint[cpu]) {
			int cur_cl, phy_cpu;
			spinlock_t *slock = &per_cpu(pcpu_slock, cpu);

			cur_cl = tegra18_logical_to_cluster(cpu);
			phy_cpu = logical_to_phys_map(cpu);

			if (spin_trylock(slock)) {
				tcpufreq_writel(tfreq_data.last_hint[cpu],
						tfreq_data.pcluster[cur_cl].edvd_pub +
						EDVD_CL_NDIV_VHINT_OFFSET, phy_cpu);
				spin_unlock(slock);
			}
		}
		break;
	}

	return NOTIFY_OK;
}
#endif

static struct notifier_block cluster_freq_nb = {
	.notifier_call = cluster_freq_notify,
};

static struct notifier_block tegra_boundaries_cpufreq_nb = {
	.notifier_call = tegra_boundaries_policy_notifier,
};

#ifdef CONFIG_CPU_PM
static struct notifier_block tegra_cpu_pm_nb = {
	.notifier_call = tegra_cpu_pm_notifier,
};
#endif

static void pm_qos_register_notifier(void)
{
	pm_qos_add_min_notifier(PM_QOS_CPU_FREQ_BOUNDS,
		&cluster_freq_nb);
	pm_qos_add_max_notifier(PM_QOS_CPU_FREQ_BOUNDS,
		&cluster_freq_nb);
}

/* Free lut space shared beteen CPU and BPMP */
static void __init free_shared_lut(void)
{
	uint16_t size = sizeof(struct cpu_vhint_data);
	struct cpu_vhint_table *vhtbl;
	int cl;

	LOOP_FOR_EACH_CLUSTER(cl) {
		/* Free lut space shared by BPMP */
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		if (vhtbl->lut && vhtbl->phys)
			tegra_bpmp_free_coherent(size, vhtbl->lut,
				vhtbl->phys);
	}
}
static void free_resources(struct device *dev)
{
	int cl;

	/* unmap iova space */
	if (tfreq_data.regs)
		devm_iounmap(dev, tfreq_data.regs);

	LOOP_FOR_EACH_CLUSTER(cl) {
		/* free ndiv_to_vindex mem */
		kfree(tfreq_data.pcluster[cl].dvfs_tbl.vindx);

		/* free table */
		kfree(tfreq_data.pcluster[cl].clft);

		/* unregister from emc bw manager */
		tegra_bwmgr_unregister(tfreq_data.pcluster[cl].bwmgr);
	}

	/* kfree handles NULL just well */
	kfree(cpu_emc_mapping_dt);
	cpu_emc_mapping_dt = NULL;
}

static int __init init_freqtbls(struct device_node *dn)
{
	u16 freq_table_step_size = CPUFREQ_TBL_STEP_SIZE;
	u16 dt_freq_table_step_size = 0;
	struct cpufreq_frequency_table *ftbl;
	struct cpu_vhint_table *vhtbl;
	u16 ndiv, max_freq_steps, delta_ndiv;
	int cl;
	int ret = 0, index;

	if (!of_property_read_u16(dn, "freq_table_step_size",
					&dt_freq_table_step_size)) {
		freq_table_step_size = dt_freq_table_step_size;
		if (!freq_table_step_size) {
			freq_table_step_size = CPUFREQ_TBL_STEP_SIZE;
			pr_info("Invalid cpu freq_table_step_size:%d setting to default value:%d\n",
				dt_freq_table_step_size, freq_table_step_size);
		}
	}

	pr_debug("CPU frequency table step size: %d\n", freq_table_step_size);

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;

		/*
		 * LUT for this cluster is not present.
		 * Could be single cluster or n cluster chip but for <cl>,
		 * current cluster, LUT is not sent by BPMP.
		 */
		if (!vhtbl->lut) {
			pr_warn("%s: cluster %d has no LUT\n",
				__func__, cl);
			continue;
		}

		delta_ndiv = vhtbl->ndiv_max - vhtbl->ndiv_min;
		if (unlikely(delta_ndiv == 0))
			max_freq_steps = 1;
		else {
			/* We store both ndiv_min and ndiv_max hence the +1 */
			max_freq_steps = delta_ndiv / freq_table_step_size + 1;
		}

		max_freq_steps += (delta_ndiv % freq_table_step_size) ? 1 : 0;

		/* Allocate memory 1 + max_freq_steps to write END_OF_TABLE */
		ftbl = kzalloc(sizeof(struct cpufreq_frequency_table) *
			(max_freq_steps + 1), GFP_KERNEL);
		if (!ftbl) {
			ret = -ENOMEM;
			while (cl--)
				kfree(tfreq_data.pcluster[cl].clft);
			goto err_out;
		}

		for (index = 0, ndiv = vhtbl->ndiv_min;
				ndiv < vhtbl->ndiv_max;
				index++, ndiv += freq_table_step_size)
			ftbl[index].frequency = (unsigned long)
				(vhtbl->ref_clk_hz * ndiv)
				/ (vhtbl->pdiv * vhtbl->mdiv * 1000);

		ftbl[index++].frequency = (unsigned long)
			(vhtbl->ndiv_max * vhtbl->ref_clk_hz) /
			(vhtbl->pdiv * vhtbl->mdiv * 1000);

		ftbl[index].frequency = CPUFREQ_TABLE_END;

		tfreq_data.pcluster[cl].clft = ftbl;
	}

err_out:
	return ret;
}

static int __init create_ndiv_to_vindex_table(void)
{
	struct cpu_vhint_table *vhtbl;
	struct cpu_vhint_data *lut;
	uint16_t mid_ndiv, i;
	int cl;
	uint8_t vindx;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;

		lut = vhtbl->lut;

		/*
		 * LUT for this cluster is not present.
		 * Could be single cluster or n cluster chip but for <cl>,
		 * current cluster, LUT is not sent by BPMP.
		 */
		if (!lut) {
			pr_warn("%s: cluster %d has no LUT\n",
				__func__, cl);
			continue;
		}

		vhtbl->vindx = kzalloc(sizeof(uint8_t) * MAX_NDIV,
				GFP_KERNEL);
		if (!vhtbl->vindx) {
			ret = -ENOMEM;
			while (cl--)
				kfree(tfreq_data.pcluster[cl].dvfs_tbl.vindx);
			goto err_out;
		}

		i = 0;
		vhtbl->vfloor = lut->vfloor;
		vhtbl->vceil = lut->vceil;
		for (vindx = vhtbl->vfloor; vindx <= vhtbl->vceil; ++vindx) {
			mid_ndiv = lut->ndiv[vindx];
			for (; ((mid_ndiv < MAX_NDIV) && (i <= mid_ndiv)); i++)
				vhtbl->vindx[i] = vindx;
		}

		/* Fill remaining vindex table by last vindex value */
		for (; i < MAX_NDIV; i++)
			vhtbl->vindx[i] = vhtbl->vceil;

		vhtbl->ref_clk_hz =  lut->ref_clk_hz;
		vhtbl->pdiv = lut->pdiv;
		vhtbl->mdiv = lut->mdiv;
		vhtbl->ndiv_min = lut->ndiv_min;
		vhtbl->ndiv_max = lut->ndiv_max;
		vhtbl->vindex_mult = lut->vindex_mult;
		vhtbl->vindex_div = lut->vindex_div;
	}
err_out:
	return ret;
}

static int __init get_lut_from_bpmp(void)
{
	const size_t size = sizeof(struct cpu_vhint_data);
	struct mrq_cpu_vhint_request md;
	struct cpu_vhint_table *vhtbl;
	struct cpu_vhint_data *virt;
	dma_addr_t phys;
	uint32_t cl;
	int ret = 0;
	bool ok = false;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		if (is_cluster_present(cl)) {
			virt = (struct cpu_vhint_data *)
				tegra_bpmp_alloc_coherent(size,
				&phys, GFP_KERNEL);
			if (!virt) {
				ret = -ENOMEM;
				while (cl--) {
					vhtbl =
					&tfreq_data.pcluster[cl].dvfs_tbl;
					tegra_bpmp_free_coherent(size,
							vhtbl->lut,
							vhtbl->phys);
				}
				ok = false;
				goto err_out;
			}
			vhtbl->lut = virt;
			vhtbl->phys = phys;
			md.addr = (uint32_t)phys;
			md.cluster_id = cl;
			ret = tegra_bpmp_send_receive(MRQ_CPU_VHINT, &md,
					sizeof(struct mrq_cpu_vhint_request),
							 NULL, 0);
			if (ret) {
				pr_err("%s: cluster %d: vhint query failed: %d\n",
					__func__, cl, ret);
				tegra_bpmp_free_coherent(size, vhtbl->lut,
						vhtbl->phys);
				vhtbl->lut = NULL;
				vhtbl->phys = 0;
			} else
				ok = true;
		} else {
			vhtbl->lut = NULL;
			vhtbl->phys = 0;
		}
	}
err_out:
	return ok ? 0 : ret;
}

static void set_cpu_mask(void)
{
	int cpu_num;

	for_each_possible_cpu(cpu_num) {
		cpumask_set_cpu(cpu_num,
			&tfreq_data.pcluster[
				tegra18_logical_to_cluster(cpu_num)].cpu_mask);
	}
}

static int __init register_with_emc_bwmgr(void)
{
	enum tegra_bwmgr_client_id bw_id = TEGRA_BWMGR_CLIENT_CPU_CLUSTER_0;
	struct tegra_bwmgr_client *bwmgr;
	int cl;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		bwmgr = tegra_bwmgr_register(bw_id);
		if (IS_ERR_OR_NULL(bwmgr)) {
			pr_warn("emc bw manager registration failed for %s\n",
			CLUSTER_STR(cl));
			ret = -ENODEV;
			while (cl--)
				tegra_bwmgr_unregister(
				tfreq_data.pcluster[cl].bwmgr);
			goto err_out;
		}
		tfreq_data.pcluster[cl].bwmgr = bwmgr;
		bw_id = TEGRA_BWMGR_CLIENT_CPU_CLUSTER_1;
	}
err_out:
	return ret;
}

static int __init tegra186_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *dn = NULL;
	struct resource *res;
	void __iomem *regs;
	uint32_t cpu;
	int ret = 0;

	dn = pdev->dev.of_node;
	if (of_device_is_compatible(dn, "nvidia,tegra18x-cpufreq-hv")) {
		tegra_cpufreq_hv_mode = true;
		pr_info("tegra18x-cpufreq: Using hv path\n");
		ret = parse_hv_dt_data(dn);
		if (ret)
			goto err_out;
	} else {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		regs = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(regs))
			goto err_out;
		tfreq_data.pcluster[B_CLUSTER].edvd_pub = regs + B_CL_OFFSET;
		tfreq_data.pcluster[M_CLUSTER].edvd_pub = regs + M_CL_OFFSET;
		tfreq_data.regs = regs;
	}

	set_cpu_mask();

	mutex_init(&tfreq_data.mlock);
	tfreq_data.freq_compute_delay = US_DELAY;

	for_each_possible_cpu(cpu) {
		mutex_init(&per_cpu(pcpu_mlock, cpu));
		spin_lock_init(&per_cpu(pcpu_slock, cpu));
	}

	tfreq_data.pcluster[B_CLUSTER].cluster_present = false;
	tfreq_data.pcluster[M_CLUSTER].cluster_present = false;
	for_each_possible_cpu(cpu) {
		if (tegra18_logical_to_cluster(cpu) == B_CLUSTER)
			tfreq_data.pcluster[B_CLUSTER].cluster_present = true;
		else if (tegra18_logical_to_cluster(cpu) == M_CLUSTER)
			tfreq_data.pcluster[M_CLUSTER].cluster_present = true;
	}

	cpu_emc_mapping_dt = tegra_cpufreq_cpu_emc_map_dt_init(dn);
	pr_info("CPU EMC frequency mapping table: %s\n",
		cpu_emc_mapping_dt ?  "from device tree" : "default setting");

#ifdef CONFIG_DEBUG_FS
	tegra_cpufreq_debug_init();
#endif

	ret = register_with_emc_bwmgr();
	if (ret) {
		pr_err("tegra18x-cpufreq: unable to register with emc bw manager\n");
		goto err_free_res;
	}

	ret = get_lut_from_bpmp();
	if (ret)
		goto err_free_res;

	ret = create_ndiv_to_vindex_table();
	if (ret)
		goto err_free_res;

	enable_cc3(dn);

	ret = init_freqtbls(dn);
	if (ret)
		goto err_free_res;

	ret = cpufreq_register_driver(&tegra_cpufreq_driver);
	if (ret)
		goto err_free_res;

	pm_qos_register_notifier();

	cpufreq_register_notifier(&tegra_boundaries_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);

#ifdef CONFIG_CPU_PM
	cpu_pm_register_notifier(&tegra_cpu_pm_nb);
#endif

	goto exit_out;
err_free_res:
	free_resources(&pdev->dev);
exit_out:
	free_shared_lut();
err_out:
	pr_info("cpufreq: platform driver Initialization: %s\n",
		(ret ? "fail" : "pass"));
	return ret;
}

static int tegra186_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_notifier(&tegra_boundaries_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);
#ifdef CONFIG_DEBUG_FS
	tegra_cpufreq_debug_exit();
#endif
	cpufreq_unregister_driver(&tegra_cpufreq_driver);
	free_resources(&pdev->dev);
	return 0;
}

static const struct of_device_id tegra186_cpufreq_of_match[] = {
	{ .compatible = "nvidia,tegra18x-cpufreq", },
	{ .compatible = "nvidia,tegra18x-cpufreq-hv", },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra186_cpufreq_of_match);

static struct platform_driver tegra186_cpufreq_platform_driver __refdata = {
	.driver = {
		.name = "tegra186-cpufreq",
		.of_match_table = tegra186_cpufreq_of_match,
	},
	.probe = tegra186_cpufreq_probe,
	.remove = tegra186_cpufreq_remove,
};
module_platform_driver(tegra186_cpufreq_platform_driver);

MODULE_AUTHOR("Puneet Saxena <puneets@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra186 cpufreq driver");
MODULE_LICENSE("GPL v2");
