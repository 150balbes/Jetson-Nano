/*
 * Cryptographic API.
 * drivers/crypto/tegra-se-nvhost.c
 *
 * Support for Tegra Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <soc/tegra/chip-id.h>
#include <linux/nvhost.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/akcipher.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/akcipher.h>
#include <crypto/sha.h>
#include <linux/tegra_pm_domains.h>
#include <crypto/internal/kpp.h>
#include <crypto/kpp.h>
#include <crypto/dh.h>
#include <linux/version.h>
#include <linux/pm_qos.h>
#include <linux/jiffies.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include "tegra-se-nvhost.h"
#include "t186/hardware_t186.h"
#include "nvhost_job.h"
#include "nvhost_channel.h"
#include "nvhost_acm.h"

#define DRIVER_NAME	"tegra-se-nvhost"
#define NV_SE1_CLASS_ID		0x3A
#define NV_SE2_CLASS_ID		0x3B
#define NV_SE3_CLASS_ID		0x3C
#define NV_SE4_CLASS_ID		0x3D
#define NUM_SE_ALGO	5
#define MIN_DH_SZ_BITS	1536

#define __nvhost_opcode_nonincr(x, y)	nvhost_opcode_nonincr((x) / 4, (y))
#define __nvhost_opcode_incr(x, y)	nvhost_opcode_incr((x) / 4, (y))

/* Security Engine operation modes */
enum tegra_se_aes_op_mode {
	SE_AES_OP_MODE_CBC,	/* Cipher Block Chaining (CBC) mode */
	SE_AES_OP_MODE_ECB,	/* Electronic Codebook (ECB) mode */
	SE_AES_OP_MODE_CTR,	/* Counter (CTR) mode */
	SE_AES_OP_MODE_OFB,	/* Output feedback (CFB) mode */
	SE_AES_OP_MODE_CMAC,	/* Cipher-based MAC (CMAC) mode */
	SE_AES_OP_MODE_RNG_DRBG,	/* Deterministic Random Bit Generator */
	SE_AES_OP_MODE_SHA1,	/* Secure Hash Algorithm-1 (SHA1) mode */
	SE_AES_OP_MODE_SHA224,	/* Secure Hash Algorithm-224  (SHA224) mode */
	SE_AES_OP_MODE_SHA256,	/* Secure Hash Algorithm-256  (SHA256) mode */
	SE_AES_OP_MODE_SHA384,	/* Secure Hash Algorithm-384  (SHA384) mode */
	SE_AES_OP_MODE_SHA512,	/* Secure Hash Algorithm-512  (SHA512) mode */
	SE_AES_OP_MODE_XTS	/* XTS mode */
};

/* Security Engine key table type */
enum tegra_se_key_table_type {
	SE_KEY_TABLE_TYPE_KEY,	/* Key */
	SE_KEY_TABLE_TYPE_KEY_IN_MEM,	/* Key in Memory */
	SE_KEY_TABLE_TYPE_ORGIV,	/* Original IV */
	SE_KEY_TABLE_TYPE_UPDTDIV,	/* Updated IV */
	SE_KEY_TABLE_TYPE_XTS_KEY1,	/* XTS Key1 */
	SE_KEY_TABLE_TYPE_XTS_KEY2,	/* XTS Key2 */
	SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM,	/* XTS Key1 in Memory */
	SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM	/* XTS Key2 in Memory */
};

struct tegra_se_chipdata {
	unsigned long aes_freq;
	unsigned int cpu_freq_mhz;
};

/* Security Engine Linked List */
struct tegra_se_ll {
	dma_addr_t addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

enum tegra_se_algo {
	SE_DRBG,
	SE_AES,
	SE_CMAC,
	SE_RSA,
	SE_SHA,
};

enum tegra_se_callback {
	NONE,
	AES_CB,
	SHA_CB,
};

struct tegra_se_dev {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *io_regs;	/* se device memory/io */
	void __iomem *pmc_io_reg;	/* pmc device memory/io */
	struct mutex lock;	/* Protect request queue */
	/* Mutex lock (mtx) is to protect Hardware, as it can be used
	 * in parallel by different threads. For example, set_key
	 * request can come from a different thread and access HW
	 */
	struct mutex mtx;
	struct clk *pclk;	/* Security Engine clock */
	struct clk *enclk;	/* Security Engine clock */
	struct crypto_queue queue; /* Security Engine crypto queue */
	struct tegra_se_slot *slot_list;	/* pointer to key slots */
	struct tegra_se_rsa_slot *rsa_slot_list; /* rsa key slot pointer */
	struct tegra_se_cmdbuf *cmdbuf_addr_list;
	unsigned int cmdbuf_list_entry;
	struct tegra_se_chipdata *chipdata; /* chip specific data */
	u32 *src_ll_buf;	/* pointer to source linked list buffer */
	dma_addr_t src_ll_buf_adr; /* Source linked list buffer dma address */
	u32 src_ll_size;	/* Size of source linked list buffer */
	u32 *dst_ll_buf;	/* pointer to destination linked list buffer */
	dma_addr_t dst_ll_buf_adr; /* Destination linked list dma address */
	u32 dst_ll_size;	/* Size of destination linked list buffer */
	struct tegra_se_ll *src_ll;
	struct tegra_se_ll *dst_ll;
	struct tegra_se_ll *aes_src_ll;
	struct tegra_se_ll *aes_dst_ll;
	u32 *dh_buf1, *dh_buf2;
	struct ablkcipher_request *reqs[SE_MAX_TASKS_PER_SUBMIT];
	struct ahash_request *sha_req;
	unsigned int req_cnt;
	u32 syncpt_id;
	u32 opcode_addr;
	bool work_q_busy;	/* Work queue busy status */
	struct nvhost_channel *channel;
	struct work_struct se_work;
	struct workqueue_struct *se_work_q;
	struct scatterlist sg;
	bool dynamic_mem;
	u32 *total_aes_buf;
	dma_addr_t total_aes_buf_addr;
	void *aes_buf;
	dma_addr_t aes_buf_addr;
	void *aes_bufs[SE_MAX_AESBUF_ALLOC];
	dma_addr_t aes_buf_addrs[SE_MAX_AESBUF_ALLOC];
	atomic_t aes_buf_stat[SE_MAX_AESBUF_ALLOC];
	dma_addr_t aes_addr;
	dma_addr_t aes_cur_addr;
	unsigned int cmdbuf_cnt;
	unsigned int bytes_mapped;
	unsigned int gather_buf_sz;
	unsigned int aesbuf_entry;
	u32 *aes_cmdbuf_cpuvaddr;
	dma_addr_t aes_cmdbuf_iova;
	struct pm_qos_request boost_cpufreq_req;
	/* Lock to protect cpufreq boost status */
	struct mutex boost_cpufreq_lock;
	struct delayed_work restore_cpufreq_work;
	unsigned long cpufreq_last_boosted;
	bool cpufreq_boosted;
	bool ioc;
	bool sha_last;
	bool sha_src_mapped;
	bool sha_dst_mapped;
};

static struct tegra_se_dev *se_devices[NUM_SE_ALGO];

/* Security Engine request context */
struct tegra_se_req_context {
	enum tegra_se_aes_op_mode op_mode; /* Security Engine operation mode */
	bool encrypt;	/* Operation type */
	u32 config;
	u32 crypto_config;
	struct tegra_se_dev *se_dev;
};

struct tegra_se_priv_data {
	struct ablkcipher_request *reqs[SE_MAX_TASKS_PER_SUBMIT];
	struct ahash_request *sha_req;
	struct tegra_se_dev *se_dev;
	unsigned int req_cnt;
	unsigned int bytes_mapped;
	unsigned int gather_buf_sz;
	struct scatterlist sg;
	void *buf;
	bool dynmem;
	bool sha_last;
	bool sha_src_mapped;
	bool sha_dst_mapped;
	dma_addr_t buf_addr;
	dma_addr_t iova;
	unsigned int cmdbuf_node;
	unsigned int aesbuf_entry;
};

/* Security Engine AES context */
struct tegra_se_aes_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct ablkcipher_request *req;
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u32 op_mode;	/* AES operation mode */
	bool is_key_in_mem; /* Whether key is in memory */
	u8 key[64]; /* To store key if is_key_in_mem set */
};

/* Security Engine random number generator context */
struct tegra_se_rng_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct ablkcipher_request *req;
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 *dt_buf;	/* Destination buffer pointer */
	dma_addr_t dt_buf_adr;	/* Destination buffer dma address */
	u32 *rng_buf;	/* RNG buffer pointer */
	dma_addr_t rng_buf_adr;	/* RNG buffer dma address */
};

/* Security Engine SHA context */
struct tegra_se_sha_context {
	struct tegra_se_dev	*se_dev;	/* Security Engine device */
	u32 op_mode;	/* SHA operation mode */
	bool is_first; /* Represents first block */
	u8 *sha_buf[2];	/* Buffer to store residual data */
	dma_addr_t sha_buf_addr[2];	/* DMA address to residual data */
	u32 total_count; /* Total bytes in all the requests */
	u32 residual_bytes; /* Residual byte count */
	u32 blk_size; /* SHA block size */
};

struct tegra_se_sha_zero_length_vector {
	unsigned int size;
	char *digest;
};

/* Security Engine AES CMAC context */
struct tegra_se_aes_cmac_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u8 K1[TEGRA_SE_KEY_128_SIZE];	/* Key1 */
	u8 K2[TEGRA_SE_KEY_128_SIZE];	/* Key2 */
	dma_addr_t dma_addr;	/* DMA address of local buffer */
	u8	*buffer;	/* local buffer pointer */
};

struct tegra_se_dh_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_rsa_slot *slot;	/* Security Engine rsa key slot */
	void *key;
	void *p;
	void *g;
	unsigned int key_size;
	unsigned int p_size;
	unsigned int g_size;
};

/* Security Engine key slot */
struct tegra_se_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	bool available; /* Tells whether key slot is free to use */
};

static struct tegra_se_slot ssk_slot = {
	.slot_num = 15,
	.available = false,
};

static struct tegra_se_slot keymem_slot = {
	.slot_num = 14,
	.available = false,
};

static struct tegra_se_slot srk_slot = {
	.slot_num = 0,
	.available = false,
};

static struct tegra_se_slot pre_allocated_slot = {
	.slot_num = 0,
	.available = false,
};

struct tegra_se_cmdbuf {
	atomic_t free;
	u32 *cmdbuf_addr;
	dma_addr_t iova;
};

static LIST_HEAD(key_slot);
static LIST_HEAD(rsa_key_slot);
static DEFINE_SPINLOCK(rsa_key_slot_lock);
static DEFINE_SPINLOCK(key_slot_lock);

#define RNG_RESEED_INTERVAL	0x00773594

/* create a work for handling the async transfers */
static void tegra_se_work_handler(struct work_struct *work);

static DEFINE_DMA_ATTRS(attrs);
static int force_reseed_count;

#define GET_MSB(x)  ((x) >> (8 * sizeof(x) - 1))
#define BOOST_PERIOD	(msecs_to_jiffies(2 * 1000)) /* 2 seconds */

static unsigned int boost_cpu_freq;
module_param(boost_cpu_freq, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(boost_cpu_freq, "CPU frequency (in MHz) to boost");

static void tegra_se_restore_cpu_freq_fn(struct work_struct *work)
{
	struct tegra_se_dev *se_dev = container_of(
			work, struct tegra_se_dev, restore_cpufreq_work.work);
	unsigned long delay = BOOST_PERIOD;

	mutex_lock(&se_dev->boost_cpufreq_lock);
	if (time_is_after_jiffies(se_dev->cpufreq_last_boosted + delay)) {
		schedule_delayed_work(&se_dev->restore_cpufreq_work, delay);
	} else {
		pm_qos_update_request(&se_dev->boost_cpufreq_req,
				      PM_QOS_DEFAULT_VALUE);
		se_dev->cpufreq_boosted = false;
	}
	mutex_unlock(&se_dev->boost_cpufreq_lock);
}

static void tegra_se_boost_cpu_freq(struct tegra_se_dev *se_dev)
{
	unsigned long delay = BOOST_PERIOD;
	s32 cpufreq_hz = boost_cpu_freq * 1000;

	mutex_lock(&se_dev->boost_cpufreq_lock);
	if (!se_dev->cpufreq_boosted) {
		pm_qos_update_request(&se_dev->boost_cpufreq_req, cpufreq_hz);
		schedule_delayed_work(&se_dev->restore_cpufreq_work, delay);
		se_dev->cpufreq_boosted = true;
	}

	se_dev->cpufreq_last_boosted = jiffies;
	mutex_unlock(&se_dev->boost_cpufreq_lock);
}

static void tegra_se_boost_cpu_init(struct tegra_se_dev *se_dev)
{
	boost_cpu_freq = se_dev->chipdata->cpu_freq_mhz;

	INIT_DELAYED_WORK(&se_dev->restore_cpufreq_work,
			  tegra_se_restore_cpu_freq_fn);

	pm_qos_add_request(&se_dev->boost_cpufreq_req, PM_QOS_CPU_FREQ_MIN,
			   PM_QOS_DEFAULT_VALUE);

	mutex_init(&se_dev->boost_cpufreq_lock);
}

static void tegra_se_boost_cpu_deinit(struct tegra_se_dev *se_dev)
{
	mutex_destroy(&se_dev->boost_cpufreq_lock);
	pm_qos_remove_request(&se_dev->boost_cpufreq_req);
	cancel_delayed_work_sync(&se_dev->restore_cpufreq_work);
}

static void tegra_se_leftshift_onebit(u8 *in_buf, u32 size, u8 *org_msb)
{
	u8 carry;
	u32 i;

	*org_msb = GET_MSB(in_buf[0]);

	/* left shift one bit */
	in_buf[0] <<= 1;
	for (carry = 0, i = 1; i < size; i++) {
		carry = GET_MSB(in_buf[i]);
		in_buf[i - 1] |= carry;
		in_buf[i] <<= 1;
	}
}

static inline void se_writel(struct tegra_se_dev *se_dev, unsigned int val,
			     unsigned int reg_offset)
{
	writel(val, se_dev->io_regs + reg_offset);
}

static inline unsigned int se_readl(struct tegra_se_dev *se_dev,
				    unsigned int reg_offset)
{
	unsigned int val;

	val = readl(se_dev->io_regs + reg_offset);

	return val;
}

static int tegra_se_init_cmdbuf_addr(struct tegra_se_dev *se_dev)
{
	int i = 0;

	se_dev->cmdbuf_addr_list = devm_kzalloc(
				se_dev->dev, sizeof(struct tegra_se_cmdbuf) *
				SE_MAX_SUBMIT_CHAIN_SZ, GFP_KERNEL);
	if (!se_dev->cmdbuf_addr_list)
		return -ENOMEM;

	for (i = 0; i < SE_MAX_SUBMIT_CHAIN_SZ; i++) {
		se_dev->cmdbuf_addr_list[i].cmdbuf_addr =
			se_dev->aes_cmdbuf_cpuvaddr + (i * SZ_4K);
		se_dev->cmdbuf_addr_list[i].iova = se_dev->aes_cmdbuf_iova +
					(i * SZ_4K * SE_WORD_SIZE_BYTES);
		atomic_set(&se_dev->cmdbuf_addr_list[i].free, 1);
	}

	return 0;
}

static void tegra_se_free_key_slot(struct tegra_se_slot *slot)
{
	if (slot) {
		spin_lock(&key_slot_lock);
		slot->available = true;
		spin_unlock(&key_slot_lock);
	}
}

static struct tegra_se_slot *tegra_se_alloc_key_slot(void)
{
	struct tegra_se_slot *slot = NULL;
	bool found = false;

	spin_lock(&key_slot_lock);
	list_for_each_entry(slot, &key_slot, node) {
		if (slot->available &&
		    (slot->slot_num != pre_allocated_slot.slot_num)) {
			slot->available = false;
			found = true;
			break;
		}
	}
	spin_unlock(&key_slot_lock);

	return found ? slot : NULL;
}

static int tegra_init_key_slot(struct tegra_se_dev *se_dev)
{
	int i;

	se_dev->slot_list = devm_kzalloc(se_dev->dev,
					 sizeof(struct tegra_se_slot) *
					 TEGRA_SE_KEYSLOT_COUNT, GFP_KERNEL);
	if (!se_dev->slot_list)
		return -ENOMEM;

	spin_lock_init(&key_slot_lock);
	spin_lock(&key_slot_lock);
	for (i = 0; i < TEGRA_SE_KEYSLOT_COUNT; i++) {
		/*
		 * Slot 0, 14 and 15 are reserved and will not be added to the
		 * free slots pool. Slot 0 is used for SRK generation, Slot 14
		 * for handling keys which are stored in memories and Slot 15 is
		 * is used for SSK operation.
		 */
		if ((i == srk_slot.slot_num) || (i == ssk_slot.slot_num)
				|| (i == keymem_slot.slot_num))
			continue;
		se_dev->slot_list[i].available = true;
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}
	spin_unlock(&key_slot_lock);

	return 0;
}

static int tegra_se_alloc_ll_buf(struct tegra_se_dev *se_dev, u32 num_src_sgs,
				 u32 num_dst_sgs)
{
	if (se_dev->src_ll_buf || se_dev->dst_ll_buf) {
		dev_err(se_dev->dev,
			"trying to allocate memory to allocated memory\n");
		return -EBUSY;
	}

	if (num_src_sgs) {
		se_dev->src_ll_size = sizeof(struct tegra_se_ll) * num_src_sgs;
		se_dev->src_ll_buf = dma_alloc_coherent(
					se_dev->dev, se_dev->src_ll_size,
					&se_dev->src_ll_buf_adr, GFP_KERNEL);
		if (!se_dev->src_ll_buf) {
			dev_err(se_dev->dev,
				"can not allocate src lldma buffer\n");
			return -ENOMEM;
		}
	}
	if (num_dst_sgs) {
		se_dev->dst_ll_size = sizeof(struct tegra_se_ll) * num_dst_sgs;
		se_dev->dst_ll_buf = dma_alloc_coherent(
					se_dev->dev, se_dev->dst_ll_size,
					&se_dev->dst_ll_buf_adr, GFP_KERNEL);
		if (!se_dev->dst_ll_buf) {
			dev_err(se_dev->dev,
				"can not allocate dst ll dma buffer\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static void tegra_se_free_ll_buf(struct tegra_se_dev *se_dev)
{
	if (se_dev->src_ll_buf) {
		dma_free_coherent(se_dev->dev, se_dev->src_ll_size,
				  se_dev->src_ll_buf, se_dev->src_ll_buf_adr);
		se_dev->src_ll_buf = NULL;
	}

	if (se_dev->dst_ll_buf) {
		dma_free_coherent(se_dev->dev, se_dev->dst_ll_size,
				  se_dev->dst_ll_buf, se_dev->dst_ll_buf_adr);
		se_dev->dst_ll_buf = NULL;
	}
}

static u32 tegra_se_get_config(struct tegra_se_dev *se_dev,
			       enum tegra_se_aes_op_mode mode, bool encrypt,
			       u32 key_len)
{
	u32 val = 0;

	switch (mode) {
	case SE_AES_OP_MODE_CBC:
	case SE_AES_OP_MODE_CMAC:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
			val |= SE_CONFIG_DEC_ALG(ALG_NOP);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
		}
		if (mode == SE_AES_OP_MODE_CMAC)
			val |= SE_CONFIG_DST(DST_HASHREG);
		else
			val |= SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_RNG_DRBG:
		val = SE_CONFIG_ENC_ALG(ALG_RNG) |
			SE_CONFIG_ENC_MODE(MODE_KEY192) |
			SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_ECB:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_CTR:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_OFB:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_SHA1:
		val = SE_CONFIG_DEC_ALG(ALG_NOP) |
			SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA1) |
			SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_SHA224:
		val = SE_CONFIG_DEC_ALG(ALG_NOP) |
			SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA224) |
			SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_SHA256:
		val = SE_CONFIG_DEC_ALG(ALG_NOP) |
			SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA256) |
			SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_SHA384:
		val = SE_CONFIG_DEC_ALG(ALG_NOP) |
			SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA384) |
			SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_SHA512:
		val = SE_CONFIG_DEC_ALG(ALG_NOP) |
			SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA512) |
			SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_XTS:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if ((key_len / 2) == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
			val |= SE_CONFIG_DEC_ALG(ALG_NOP);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len / 2 == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
			else
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
			val |= SE_CONFIG_ENC_ALG(ALG_NOP);
		}
			val |= SE_CONFIG_DST(DST_MEMORY);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}

	return val;
}

static void tegra_unmap_sg(struct device *dev, struct scatterlist *sg,
			   enum dma_data_direction dir, u32 total)
{
	while (sg) {
		dma_unmap_sg(dev, sg, 1, dir);
		sg = sg_next(sg);
	}
}

static unsigned int tegra_se_count_sgs(struct scatterlist *sl, u32 nbytes)
{
	unsigned int sg_nents = 0;

	while (sl) {
		sg_nents++;
		nbytes -= min((size_t)sl->length, (size_t)nbytes);
		if (!nbytes)
			break;
		sl = sg_next(sl);
	}

	return sg_nents;
}


static void tegra_se_sha_complete_callback(void *priv, int nr_completed)
{
	struct tegra_se_priv_data *priv_data = priv;
	struct ahash_request *req;
	struct tegra_se_dev *se_dev;

	se_dev = priv_data->se_dev;

	req = priv_data->sha_req;
	if (!req) {
		dev_err(se_dev->dev, "Invalid request for callback\n");
		devm_kfree(se_dev->dev, priv_data);
		return;
	}

	if (priv_data->sha_src_mapped)
		tegra_unmap_sg(se_dev->dev, req->src, DMA_TO_DEVICE,
			       priv_data->bytes_mapped);

	if (priv_data->sha_dst_mapped)
		tegra_unmap_sg(se_dev->dev, &priv_data->sg, DMA_FROM_DEVICE,
			       priv_data->bytes_mapped);

	req->base.complete(&req->base, 0);

	devm_kfree(se_dev->dev, priv_data);
}

static void tegra_se_aes_complete_callback(void *priv, int nr_completed)
{
	int i = 0;
	struct tegra_se_priv_data *priv_data = priv;
	struct ablkcipher_request *req;
	struct tegra_se_dev *se_dev;
	void *buf;
	u32 num_sgs;

	se_dev = priv_data->se_dev;
	atomic_set(&se_dev->cmdbuf_addr_list[priv_data->cmdbuf_node].free, 1);

	if (!priv_data->req_cnt) {
		devm_kfree(se_dev->dev, priv_data);
		return;
	}

	if (!se_dev->ioc)
		dma_sync_single_for_cpu(se_dev->dev, priv_data->buf_addr,
				priv_data->gather_buf_sz, DMA_BIDIRECTIONAL);

	buf = priv_data->buf;
	for (i = 0; i < priv_data->req_cnt; i++) {
		req = priv_data->reqs[i];
		if (!req) {
			dev_err(se_dev->dev, "Invalid request for callback\n");
			if (priv_data->dynmem)
				kfree(priv_data->buf);
			devm_kfree(se_dev->dev, priv_data);
			return;
		}

		num_sgs = tegra_se_count_sgs(req->dst, req->nbytes);
		if (num_sgs == 1)
			memcpy(sg_virt(req->dst), buf, req->nbytes);
		else
			sg_copy_from_buffer(req->dst, num_sgs, buf,
					    req->nbytes);

		buf += req->nbytes;
		req->base.complete(&req->base, 0);
	}

	if (!se_dev->ioc)
		dma_unmap_sg(se_dev->dev, &priv_data->sg, 1, DMA_BIDIRECTIONAL);

	if (unlikely(priv_data->dynmem)) {
		if (se_dev->ioc)
			dma_free_coherent(se_dev->dev, priv_data->gather_buf_sz,
					  priv_data->buf, priv_data->buf_addr);
		else
			kfree(priv_data->buf);
	} else {
		atomic_set(&se_dev->aes_buf_stat[priv_data->aesbuf_entry], 1);
	}

	devm_kfree(se_dev->dev, priv_data);
}

static void se_nvhost_write_method(u32 *buf, u32 op1, u32 op2, u32 *offset)
{
	int i = 0;

	buf[i++] = op1;
	buf[i++] = op2;
	*offset = *offset + 2;
}

static int tegra_se_channel_submit_gather(struct tegra_se_dev *se_dev,
					  u32 *cpuvaddr, dma_addr_t iova,
					  u32 offset, u32 num_words,
					  enum tegra_se_callback callback)
{
	int i = 0;
	struct nvhost_job *job = NULL;
	u32 syncpt_id = 0;
	int err = 0;
	struct tegra_se_priv_data *priv = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(se_dev->pdev);

	if (callback) {
		priv = devm_kzalloc(se_dev->dev,
				    sizeof(struct tegra_se_priv_data),
				    GFP_KERNEL);
		if (!priv)
			return -ENOMEM;
	}

	err = nvhost_module_busy(se_dev->pdev);
	if (err) {
		dev_err(se_dev->dev, "nvhost_module_busy failed for se_dev\n");
		if (priv)
			devm_kfree(se_dev->dev, priv);
		return err;
	}

	if (!se_dev->channel) {
		err = nvhost_channel_map(pdata, &se_dev->channel, pdata);
		if (err) {
			dev_err(se_dev->dev, "Nvhost Channel map failed\n");
			goto exit;
		}
	}

	job = nvhost_job_alloc(se_dev->channel, 1, 0, 0, 1);
	if (!job) {
		dev_err(se_dev->dev, "Nvhost Job allocation failed\n");
		err = -ENOMEM;
		goto exit;
	}

	if (!se_dev->syncpt_id) {
		se_dev->syncpt_id = nvhost_get_syncpt_host_managed(
					se_dev->pdev, 0, se_dev->pdev->name);
		if (!se_dev->syncpt_id) {
			dev_err(se_dev->dev, "Cannot get syncpt_id for SE(%s)\n",
				se_dev->pdev->name);
			err = -ENOMEM;
			goto error;
		}
	}
	syncpt_id = se_dev->syncpt_id;

	/* initialize job data */
	se_dev->channel->syncpts[0] = syncpt_id;
	job->sp->id = syncpt_id;
	job->sp->incrs = 1;
	job->num_syncpts = 1;

	/* push increment after work has been completed */
	se_nvhost_write_method(&cpuvaddr[num_words], nvhost_opcode_nonincr(
					host1x_uclass_incr_syncpt_r(), 1),
			       nvhost_class_host_incr_syncpt(
				host1x_uclass_incr_syncpt_cond_op_done_v(),
				syncpt_id), &num_words);

	err = nvhost_job_add_client_gather_address(job, num_words,
						   pdata->class, iova);
	if (err) {
		dev_err(se_dev->dev, "Nvhost failed to add gather\n");
		goto error;
	}

	err = nvhost_channel_submit(job);
	if (err) {
		dev_err(se_dev->dev, "Nvhost submit failed\n");
		goto error;
	}

	if (callback == AES_CB) {
		priv->se_dev = se_dev;
		for (i = 0; i < se_dev->req_cnt; i++)
			priv->reqs[i] = se_dev->reqs[i];

		if (!se_dev->ioc)
			priv->sg = se_dev->sg;

		if (unlikely(se_dev->dynamic_mem)) {
			priv->buf = se_dev->aes_buf;
			priv->dynmem = se_dev->dynamic_mem;
		} else {
			priv->buf = se_dev->aes_bufs[se_dev->aesbuf_entry];
			priv->aesbuf_entry = se_dev->aesbuf_entry;
		}

		priv->buf_addr = se_dev->aes_addr;
		priv->req_cnt = se_dev->req_cnt;
		priv->gather_buf_sz = se_dev->gather_buf_sz;
		priv->cmdbuf_node = se_dev->cmdbuf_list_entry;

		/* Register callback to be called once
		 * syncpt value has been reached
		 */
		err = nvhost_intr_register_fast_notifier(
			se_dev->pdev, job->sp->id, job->sp->fence,
			tegra_se_aes_complete_callback, priv);
		if (err) {
			dev_err(se_dev->dev,
				"add nvhost interrupt action failed for AES\n");
			goto error;
		}
	} else if (callback == SHA_CB) {
		priv->se_dev = se_dev;
		priv->sha_req = se_dev->sha_req;
		priv->sg = se_dev->sg;
		priv->bytes_mapped = se_dev->bytes_mapped;
		priv->sha_src_mapped = se_dev->sha_src_mapped;
		priv->sha_dst_mapped = se_dev->sha_dst_mapped;
		priv->sha_last = se_dev->sha_last;
		priv->buf_addr = se_dev->dst_ll->addr;

		err = nvhost_intr_register_fast_notifier(
			se_dev->pdev, job->sp->id, job->sp->fence,
			tegra_se_sha_complete_callback, priv);
		if (err) {
			dev_err(se_dev->dev,
				"add nvhost interrupt action failed for SHA\n");
			goto error;
		}
	} else {
		/* wait until host1x has processed work */
		nvhost_syncpt_wait_timeout_ext(
			se_dev->pdev, job->sp->id, job->sp->fence,
			(u32)MAX_SCHEDULE_TIMEOUT, NULL, NULL);

		if (se_dev->cmdbuf_addr_list)
			atomic_set(&se_dev->cmdbuf_addr_list[
				   se_dev->cmdbuf_list_entry].free, 1);
	}

	se_dev->req_cnt = 0;
	se_dev->gather_buf_sz = 0;
	se_dev->cmdbuf_cnt = 0;
	se_dev->bytes_mapped = 0;
	se_dev->sha_src_mapped = false;
	se_dev->sha_dst_mapped = false;
	se_dev->sha_last = false;
error:
	nvhost_job_put(job);
	job = NULL;
exit:
	nvhost_module_idle(se_dev->pdev);
	if (err)
		devm_kfree(se_dev->dev, priv);

	return err;
}

static void tegra_se_send_ctr_seed(struct tegra_se_dev *se_dev, u32 *pdata,
				   unsigned int opcode_addr, u32 *cpuvaddr)
{
	u32 j;
	u32 cmdbuf_num_words = 0, i = 0;

	i = se_dev->cmdbuf_cnt;

	cpuvaddr[i++] = __nvhost_opcode_nonincr(opcode_addr +
						SE_AES_CRYPTO_CTR_SPARE, 1);
	cpuvaddr[i++] = SE_AES_CTR_LITTLE_ENDIAN;
	cpuvaddr[i++] = __nvhost_opcode_incr(opcode_addr +
					     SE_AES_CRYPTO_LINEAR_CTR, 4);
	for (j = 0; j < SE_CRYPTO_CTR_REG_COUNT; j++)
		cpuvaddr[i++] = pdata[j];

	cmdbuf_num_words = i;
	se_dev->cmdbuf_cnt = i;
}

static int tegra_se_send_key_data(struct tegra_se_dev *se_dev, u8 *pdata,
				  u32 data_len, u8 slot_num,
				  enum tegra_se_key_table_type type,
				  unsigned int opcode_addr, u32 *cpuvaddr,
				  dma_addr_t iova,
				  enum tegra_se_callback callback)
{
	u32 data_size;
	u32 *pdata_buf = (u32 *)pdata;
	u8 pkt = 0, quad = 0;
	u32 val = 0, j;
	u32 cmdbuf_num_words = 0, i = 0;
	int err = 0;

	if (!pdata_buf) {
		dev_err(se_dev->dev, "No Key Data available\n");
		return -ENODATA;
	}

	if ((type == SE_KEY_TABLE_TYPE_KEY) &&
	    (slot_num == ssk_slot.slot_num)) {
		dev_err(se_dev->dev, "SSK Key Slot used\n");
		return -EINVAL;
	}

	if ((type == SE_KEY_TABLE_TYPE_ORGIV) ||
	    (type == SE_KEY_TABLE_TYPE_XTS_KEY2) ||
	    (type == SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM))
		quad = QUAD_ORG_IV;
	else if (type == SE_KEY_TABLE_TYPE_UPDTDIV)
		quad = QUAD_UPDTD_IV;
	else if ((type == SE_KEY_TABLE_TYPE_KEY) ||
		 (type == SE_KEY_TABLE_TYPE_XTS_KEY1) ||
		 (type == SE_KEY_TABLE_TYPE_KEY_IN_MEM) ||
		 (type == SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM))
		quad = QUAD_KEYS_128;

	i = se_dev->cmdbuf_cnt;

	if (!se_dev->cmdbuf_cnt) {
		cpuvaddr[i++] = __nvhost_opcode_nonincr(
				opcode_addr + SE_AES_OPERATION_OFFSET, 1);
		cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_OP(OP_DUMMY);
	}

	data_size = SE_KEYTABLE_QUAD_SIZE_BYTES;

	do {
		if (type == SE_KEY_TABLE_TYPE_XTS_KEY2 ||
				type == SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM)
			pkt = SE_CRYPTO_KEYIV_PKT_SUBKEY_SEL(SUBKEY_SEL_KEY2);
		else if (type == SE_KEY_TABLE_TYPE_XTS_KEY1 ||
				type == SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM)
			pkt = SE_CRYPTO_KEYIV_PKT_SUBKEY_SEL(SUBKEY_SEL_KEY1);

		pkt |= (SE_KEYTABLE_SLOT(slot_num) | SE_KEYTABLE_QUAD(quad));

		for (j = 0; j < data_size; j += 4, data_len -= 4) {
			cpuvaddr[i++] = __nvhost_opcode_nonincr(
					opcode_addr +
					SE_AES_CRYPTO_KEYTABLE_ADDR_OFFSET, 1);

			val = (SE_KEYTABLE_PKT(pkt) | (j / 4));
			cpuvaddr[i++] = val;

			cpuvaddr[i++] = __nvhost_opcode_incr(
					opcode_addr +
					SE_AES_CRYPTO_KEYTABLE_DATA_OFFSET, 1);
			cpuvaddr[i++] = *pdata_buf++;
		}
		data_size = data_len;
		if ((type == SE_KEY_TABLE_TYPE_KEY) ||
		    (type == SE_KEY_TABLE_TYPE_XTS_KEY1) ||
		    (type == SE_KEY_TABLE_TYPE_KEY_IN_MEM) ||
		    (type == SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM))
			quad = QUAD_KEYS_256;
		else if ((type == SE_KEY_TABLE_TYPE_XTS_KEY2) ||
			(type == SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM))
			quad = QUAD_UPDTD_IV;

	} while (data_len);

	if ((type != SE_KEY_TABLE_TYPE_ORGIV) &&
	    (type != SE_KEY_TABLE_TYPE_UPDTDIV) &&
		(type != SE_KEY_TABLE_TYPE_KEY_IN_MEM) &&
		(type != SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM) &&
		(type != SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM)) {
		cpuvaddr[i++] = __nvhost_opcode_nonincr(
				opcode_addr + SE_AES_OPERATION_OFFSET, 1);
		cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_OP(OP_DUMMY);
	}

	cmdbuf_num_words = i;
	se_dev->cmdbuf_cnt = i;

	if ((type != SE_KEY_TABLE_TYPE_ORGIV) &&
	    (type != SE_KEY_TABLE_TYPE_UPDTDIV) &&
		(type != SE_KEY_TABLE_TYPE_KEY_IN_MEM) &&
		(type != SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM) &&
		(type != SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM))
		err = tegra_se_channel_submit_gather(
			se_dev, cpuvaddr, iova, 0, cmdbuf_num_words, callback);

	return err;
}

static u32 tegra_se_get_crypto_config(struct tegra_se_dev *se_dev,
				      enum tegra_se_aes_op_mode mode,
				      bool encrypt, u8 slot_num, bool org_iv)
{
	u32 val = 0;
	unsigned long freq = 0;

	switch (mode) {
	case SE_AES_OP_MODE_XTS:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_TWEAK) |
				SE_CRYPTO_XOR_POS(XOR_BOTH) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_TWEAK) |
				SE_CRYPTO_XOR_POS(XOR_BOTH) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_CMAC:
	case SE_AES_OP_MODE_CBC:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_AESOUT) |
				SE_CRYPTO_XOR_POS(XOR_TOP) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_PREVAHB) |
				SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_RNG_DRBG:
		val = SE_CRYPTO_INPUT_SEL(INPUT_RANDOM) |
			SE_CRYPTO_XOR_POS(XOR_BYPASS) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		break;
	case SE_AES_OP_MODE_ECB:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_CTR:
		val = SE_CRYPTO_INPUT_SEL(INPUT_LNR_CTR) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_MEMORY) |
			SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_OFB:
		val = SE_CRYPTO_INPUT_SEL(INPUT_AESOUT) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_MEMORY) |
			SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->aes_freq;
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}

	if (mode == SE_AES_OP_MODE_CTR) {
		val |= SE_CRYPTO_HASH(HASH_DISABLE) |
			SE_CRYPTO_KEY_INDEX(slot_num) |
			SE_CRYPTO_CTR_CNTN(1);
	} else {
		val |= SE_CRYPTO_HASH(HASH_DISABLE) |
			SE_CRYPTO_KEY_INDEX(slot_num) |
			(org_iv ? SE_CRYPTO_IV_SEL(IV_ORIGINAL) :
			SE_CRYPTO_IV_SEL(IV_UPDATED));
	}

	/* enable hash for CMAC */
	if (mode == SE_AES_OP_MODE_CMAC)
		val |= SE_CRYPTO_HASH(HASH_ENABLE);

	if (mode == SE_AES_OP_MODE_RNG_DRBG) {
		/* Make sure engine is powered ON*/
		nvhost_module_busy(se_dev->pdev);

		if (force_reseed_count <= 0) {
			se_writel(se_dev,
				  SE_RNG_CONFIG_MODE(DRBG_MODE_FORCE_RESEED) |
				  SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
				  SE_RNG_CONFIG_REG_OFFSET);
		force_reseed_count = RNG_RESEED_INTERVAL;
		} else {
			se_writel(se_dev,
				  SE_RNG_CONFIG_MODE(DRBG_MODE_NORMAL) |
				  SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
				  SE_RNG_CONFIG_REG_OFFSET);
		}
		--force_reseed_count;

		se_writel(se_dev, RNG_RESEED_INTERVAL,
			  SE_RNG_RESEED_INTERVAL_REG_OFFSET);

		/* Power off device after register access done */
		nvhost_module_idle(se_dev->pdev);
	}

	return val;
}

static int tegra_se_send_sha_data(struct tegra_se_dev *se_dev,
				  struct tegra_se_req_context *req_ctx,
				  struct tegra_se_sha_context *sha_ctx,
				  u32 count, bool last)
{
	int err = 0;
	u32 cmdbuf_num_words = 0, i = 0;
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	struct tegra_se_ll *src_ll = se_dev->src_ll;
	struct tegra_se_ll *dst_ll = se_dev->dst_ll;
	unsigned int total = count, val;
	u64 msg_len;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_4K,
					  &cmdbuf_iova, GFP_KERNEL,
					  __DMA_ATTR(attrs));
	if (!cmdbuf_cpuvaddr) {
		dev_err(se_dev->dev, "Failed to allocate cmdbuf\n");
		return -ENOMEM;
	}

	while (total) {
		if (src_ll->data_len & SE_BUFF_SIZE_MASK) {
			dma_free_attrs(se_dev->dev->parent, SZ_4K,
				       cmdbuf_cpuvaddr, cmdbuf_iova,
				       __DMA_ATTR(attrs));
			return -EINVAL;
		}

		if (total == count) {
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(
						se_dev->opcode_addr +
						SE_SHA_MSG_LENGTH_OFFSET, 8);
			msg_len = (count * 8);
			cmdbuf_cpuvaddr[i++] =
					(sha_ctx->total_count * 8);
			cmdbuf_cpuvaddr[i++] = (u32)(msg_len >> 32);
			cmdbuf_cpuvaddr[i++] = 0;
			cmdbuf_cpuvaddr[i++] = 0;

			/* If it is not last request, length of message left
			 * should be more than input buffer length.
			 */
			if (!last)
				cmdbuf_cpuvaddr[i++] =
					(u32)(msg_len + 8) & 0xFFFFFFFFULL;
			else
				cmdbuf_cpuvaddr[i++] =
					(u32)((msg_len) &  0xFFFFFFFFULL);
			cmdbuf_cpuvaddr[i++] = (u32)(msg_len >> 32);
			cmdbuf_cpuvaddr[i++] = 0;
			cmdbuf_cpuvaddr[i++] = 0;

			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(
						se_dev->opcode_addr, 6);

			cmdbuf_cpuvaddr[i++] = req_ctx->config;

			if (sha_ctx->is_first)
				cmdbuf_cpuvaddr[i++] =
					SE4_HW_INIT_HASH(HW_INIT_HASH_ENABLE);
			else
				cmdbuf_cpuvaddr[i++] =
					SE4_HW_INIT_HASH(HW_INIT_HASH_DISABLE);
		} else {
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(
						se_dev->opcode_addr +
						SE4_SHA_IN_ADDR_OFFSET, 4);
		}
		cmdbuf_cpuvaddr[i++] = src_ll->addr;
		cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(src_ll->addr)) |
					SE_ADDR_HI_SZ(src_ll->data_len));
		cmdbuf_cpuvaddr[i++] = dst_ll->addr;
		cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(dst_ll->addr)) |
					SE_ADDR_HI_SZ(dst_ll->data_len));

		cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
						se_dev->opcode_addr +
						SE_SHA_OPERATION_OFFSET, 1);

		val = SE_OPERATION_WRSTALL(WRSTALL_TRUE);
		if (total == count) {
			if (total == src_ll->data_len)
				val |= SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
					SE_OPERATION_OP(OP_START);
			else
				val |= SE_OPERATION_LASTBUF(LASTBUF_FALSE) |
					SE_OPERATION_OP(OP_START);
		} else {
			if (total == src_ll->data_len)
				val |= SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
					SE_OPERATION_OP(OP_RESTART_IN);
			else
				val |= SE_OPERATION_LASTBUF(LASTBUF_FALSE) |
					SE_OPERATION_OP(OP_RESTART_IN);
		}
		cmdbuf_cpuvaddr[i++] = val;
		total -= src_ll->data_len;
		src_ll++;
	}

	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(se_dev, cmdbuf_cpuvaddr,
					     cmdbuf_iova, 0, cmdbuf_num_words,
					     SHA_CB);
	dma_free_attrs(se_dev->dev->parent, SZ_4K, cmdbuf_cpuvaddr,
		       cmdbuf_iova, __DMA_ATTR(attrs));

	return err;
}

static void tegra_se_read_cmac_result(struct tegra_se_dev *se_dev, u8 *pdata,
				      u32 nbytes, bool swap32)
{
	u32 *result = (u32 *)pdata;
	u32 i;

	/* Make SE engine is powered ON */
	nvhost_module_busy(se_dev->pdev);

	for (i = 0; i < nbytes / 4; i++) {
		result[i] = se_readl(se_dev, SE_CMAC_RESULT_REG_OFFSET +
				     (i * sizeof(u32)));
		if (swap32)
			result[i] = be32_to_cpu(result[i]);
	}

	nvhost_module_idle(se_dev->pdev);
}

static void tegra_se_send_data(struct tegra_se_dev *se_dev,
			       struct tegra_se_req_context *req_ctx,
			       struct ablkcipher_request *req, u32 nbytes,
			       unsigned int opcode_addr, u32 *cpuvaddr)
{
	u32 cmdbuf_num_words = 0, i = 0;
	u32 total, val;
	unsigned int restart_op;
	struct tegra_se_ll *src_ll;
	struct tegra_se_ll *dst_ll;

	if (req) {
		src_ll = se_dev->aes_src_ll;
		dst_ll = se_dev->aes_dst_ll;
		src_ll->addr = se_dev->aes_cur_addr;
		dst_ll->addr = se_dev->aes_cur_addr;
		src_ll->data_len = req->nbytes;
		dst_ll->data_len = req->nbytes;
	} else {
		src_ll = se_dev->src_ll;
		dst_ll = se_dev->dst_ll;
	}

	i = se_dev->cmdbuf_cnt;
	total = nbytes;

	/* Create Gather Buffer Command */
	while (total) {
		if (total == nbytes) {
			cpuvaddr[i++] = __nvhost_opcode_nonincr(
					opcode_addr +
					SE_AES_CRYPTO_LAST_BLOCK_OFFSET, 1);
			cpuvaddr[i++] = ((nbytes /
					 TEGRA_SE_AES_BLOCK_SIZE) - 1);

			cpuvaddr[i++] = __nvhost_opcode_incr(opcode_addr, 6);
			cpuvaddr[i++] = req_ctx->config;
			cpuvaddr[i++] = req_ctx->crypto_config;
		} else {
			cpuvaddr[i++] = __nvhost_opcode_incr(
					opcode_addr + SE_AES_IN_ADDR_OFFSET, 4);
		}

		cpuvaddr[i++] = (u32)(src_ll->addr);
		cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(src_ll->addr)) |
				SE_ADDR_HI_SZ(src_ll->data_len));
		cpuvaddr[i++] = (u32)(dst_ll->addr);
		cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(dst_ll->addr)) |
				SE_ADDR_HI_SZ(dst_ll->data_len));

		if (req_ctx->op_mode == SE_AES_OP_MODE_CMAC)
			restart_op = OP_RESTART_IN;
		else if (req_ctx->op_mode == SE_AES_OP_MODE_RNG_DRBG)
			restart_op = OP_RESTART_OUT;
		else
			restart_op = OP_RESTART_INOUT;

		cpuvaddr[i++] = __nvhost_opcode_nonincr(
				opcode_addr + SE_AES_OPERATION_OFFSET, 1);

		val = SE_OPERATION_WRSTALL(WRSTALL_TRUE);
		if (total == nbytes) {
			if (total == src_ll->data_len)
				val |= SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
					SE_OPERATION_OP(OP_START);
			else
				val |= SE_OPERATION_LASTBUF(LASTBUF_FALSE) |
					SE_OPERATION_OP(OP_START);
		} else {
			if (total == src_ll->data_len)
				val |= SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
					SE_OPERATION_OP(restart_op);
			else
				val |= SE_OPERATION_LASTBUF(LASTBUF_FALSE) |
					SE_OPERATION_OP(restart_op);
		}
		cpuvaddr[i++] = val;
		total -= src_ll->data_len;
		src_ll++;
		dst_ll++;
	}

	cmdbuf_num_words = i;
	se_dev->cmdbuf_cnt = i;
	if (req)
		se_dev->aes_cur_addr += req->nbytes;
}

static int tegra_map_sg(struct device *dev, struct scatterlist *sg,
			unsigned int nents, enum dma_data_direction dir,
			struct tegra_se_ll *se_ll, u32 total)
{
	u32 total_loop = 0;
	int ret = 0;

	total_loop = total;
	while (sg) {
		ret = dma_map_sg(dev, sg, nents, dir);
		if (!ret) {
			dev_err(dev, "dma_map_sg  error\n");
			return ret;
		}
		se_ll->addr = sg_dma_address(sg);
		se_ll->data_len = min((size_t)sg->length, (size_t)total_loop);
		total_loop -= min((size_t)sg->length, (size_t)total_loop);
		sg = sg_next(sg);
		se_ll++;
	}

	return ret;
}

static int tegra_se_setup_ablk_req(struct tegra_se_dev *se_dev)
{
	struct ablkcipher_request *req;
	void *buf;
	int i, ret = 0;
	u32 num_sgs;
	unsigned int index = 0;

	if (unlikely(se_dev->dynamic_mem)) {
		if (se_dev->ioc)
			se_dev->aes_buf = dma_alloc_coherent(
					se_dev->dev, se_dev->gather_buf_sz,
					&se_dev->aes_buf_addr, GFP_KERNEL);
		else
			se_dev->aes_buf = kmalloc(se_dev->gather_buf_sz,
						  GFP_KERNEL);
		if (!se_dev->aes_buf)
			return -ENOMEM;
		buf = se_dev->aes_buf;
	} else {
		index = se_dev->aesbuf_entry + 1;
		for (i = 0; i < SE_MAX_AESBUF_TIMEOUT; i++, index++) {
			index = index % SE_MAX_AESBUF_ALLOC;
			if (atomic_read(&se_dev->aes_buf_stat[index])) {
				se_dev->aesbuf_entry = index;
				atomic_set(&se_dev->aes_buf_stat[index], 0);
				break;
			}
			if (i % SE_MAX_AESBUF_ALLOC == 0)
				udelay(SE_WAIT_UDELAY);
		}

		if (i == SE_MAX_AESBUF_TIMEOUT) {
			pr_err("aes_buffer not available\n");
			return -ETIMEDOUT;
		}
		buf = se_dev->aes_bufs[index];
	}

	for (i = 0; i < se_dev->req_cnt; i++) {
		req = se_dev->reqs[i];

		num_sgs = tegra_se_count_sgs(req->src, req->nbytes);

		if (num_sgs == 1)
			memcpy(buf, sg_virt(req->src), req->nbytes);
		else
			sg_copy_to_buffer(req->src, num_sgs, buf, req->nbytes);
		buf += req->nbytes;
	}

	if (se_dev->ioc) {
		if (unlikely(se_dev->dynamic_mem))
			se_dev->aes_addr = se_dev->aes_buf_addr;
		else
			se_dev->aes_addr = se_dev->aes_buf_addrs[index];
	} else {
		if (unlikely(se_dev->dynamic_mem))
			sg_init_one(&se_dev->sg, se_dev->aes_buf,
				    se_dev->gather_buf_sz);
		else
			sg_init_one(&se_dev->sg, se_dev->aes_bufs[index],
				    se_dev->gather_buf_sz);

		ret = dma_map_sg(se_dev->dev, &se_dev->sg, 1,
				 DMA_BIDIRECTIONAL);
		if (!ret) {
			dev_err(se_dev->dev, "dma_map_sg  error\n");

			if (unlikely(se_dev->dynamic_mem))
				kfree(se_dev->aes_buf);
			else
				atomic_set(&se_dev->aes_buf_stat[index], 1);
			return ret;
		}

		se_dev->aes_addr = sg_dma_address(&se_dev->sg);
	}

	se_dev->aes_cur_addr = se_dev->aes_addr;

	return 0;
}

static int tegra_se_prepare_cmdbuf(struct tegra_se_dev *se_dev,
				   u32 *cpuvaddr, dma_addr_t iova)
{
	int i, ret = 0;
	struct tegra_se_aes_context *aes_ctx;
	struct ablkcipher_request *req;
	struct tegra_se_req_context *req_ctx;
	struct crypto_ablkcipher *tfm;
	u32 keylen;

	for (i = 0; i < se_dev->req_cnt; i++) {
		req = se_dev->reqs[i];
		tfm = crypto_ablkcipher_reqtfm(req);
		aes_ctx = crypto_ablkcipher_ctx(tfm);
		/* Ensure there is valid slot info */
		if (!aes_ctx->slot) {
			dev_err(se_dev->dev, "Invalid AES Ctx Slot\n");
			return -EINVAL;
		}

		if (aes_ctx->is_key_in_mem) {
			if (strcmp(crypto_tfm_alg_name(&tfm->base),
					       "xts(aes)")) {
				ret = tegra_se_send_key_data(
					se_dev, aes_ctx->key, aes_ctx->keylen,
					aes_ctx->slot->slot_num,
					SE_KEY_TABLE_TYPE_KEY_IN_MEM,
					se_dev->opcode_addr, cpuvaddr, iova,
					AES_CB);
			} else {
				keylen = aes_ctx->keylen / 2;
				ret = tegra_se_send_key_data(se_dev,
					aes_ctx->key, keylen,
					aes_ctx->slot->slot_num,
					SE_KEY_TABLE_TYPE_XTS_KEY1_IN_MEM,
					se_dev->opcode_addr, cpuvaddr, iova,
					AES_CB);
				if (ret) {
					dev_err(se_dev->dev, "Error in setting Key\n");
					goto out;
				}

				ret = tegra_se_send_key_data(se_dev,
					aes_ctx->key + keylen, keylen,
					aes_ctx->slot->slot_num,
					SE_KEY_TABLE_TYPE_XTS_KEY2_IN_MEM,
					se_dev->opcode_addr, cpuvaddr, iova,
					AES_CB);
			}
			if (ret) {
				dev_err(se_dev->dev, "Error in setting Key\n");
				goto out;
			}
		}

		req_ctx = ablkcipher_request_ctx(req);

		if (req->info) {
			if (req_ctx->op_mode == SE_AES_OP_MODE_CTR ||
			    req_ctx->op_mode == SE_AES_OP_MODE_XTS) {
				tegra_se_send_ctr_seed(se_dev, (u32 *)req->info,
						       se_dev->opcode_addr,
						       cpuvaddr);
			} else {
				ret = tegra_se_send_key_data(
				se_dev, req->info, TEGRA_SE_AES_IV_SIZE,
				aes_ctx->slot->slot_num,
				SE_KEY_TABLE_TYPE_UPDTDIV, se_dev->opcode_addr,
				cpuvaddr, iova, AES_CB);
			}
		}

		if (ret)
			return ret;

		req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
						      req_ctx->encrypt,
						      aes_ctx->keylen);
		req_ctx->crypto_config = tegra_se_get_crypto_config(
						se_dev, req_ctx->op_mode,
						req_ctx->encrypt,
						aes_ctx->slot->slot_num, false);

		tegra_se_send_data(se_dev, req_ctx, req, req->nbytes,
				   se_dev->opcode_addr, cpuvaddr);
	}

out:
	return ret;
}

static int tegra_se_get_free_cmdbuf(struct tegra_se_dev *se_dev)
{
	int i = 0;
	unsigned int index = se_dev->cmdbuf_list_entry + 1;

	for (i = 0; i < SE_MAX_CMDBUF_TIMEOUT; i++, index++) {
		index = index % SE_MAX_SUBMIT_CHAIN_SZ;
		if (atomic_read(&se_dev->cmdbuf_addr_list[index].free)) {
			atomic_set(&se_dev->cmdbuf_addr_list[index].free, 0);
			break;
		}
		if (i % SE_MAX_SUBMIT_CHAIN_SZ == 0)
			udelay(SE_WAIT_UDELAY);
	}

	return (i == SE_MAX_CMDBUF_TIMEOUT) ? -ENOMEM : index;
}

static void tegra_se_process_new_req(struct tegra_se_dev *se_dev)
{
	struct ablkcipher_request *req;
	u32 *cpuvaddr = NULL;
	dma_addr_t iova = 0;
	unsigned int index = 0;
	int err = 0, i = 0;

	tegra_se_boost_cpu_freq(se_dev);

	for (i = 0; i < se_dev->req_cnt; i++) {
		req = se_dev->reqs[i];
		if (req->nbytes != SE_STATIC_MEM_ALLOC_BUFSZ) {
			se_dev->dynamic_mem = true;
			break;
		}
	}

	err = tegra_se_setup_ablk_req(se_dev);
	if (err)
		goto mem_out;

	err = tegra_se_get_free_cmdbuf(se_dev);
	if (err < 0) {
		dev_err(se_dev->dev, "Couldn't get free cmdbuf\n");
		goto index_out;
	}

	index = err;

	cpuvaddr = se_dev->cmdbuf_addr_list[index].cmdbuf_addr;
	iova = se_dev->cmdbuf_addr_list[index].iova;
	se_dev->cmdbuf_list_entry = index;

	err = tegra_se_prepare_cmdbuf(se_dev, cpuvaddr, iova);
	if (err)
		goto cmdbuf_out;

	err = tegra_se_channel_submit_gather(se_dev, cpuvaddr, iova, 0,
					     se_dev->cmdbuf_cnt, AES_CB);
	if (err)
		goto cmdbuf_out;
	se_dev->dynamic_mem = false;

	return;
cmdbuf_out:
	atomic_set(&se_dev->cmdbuf_addr_list[index].free, 1);
index_out:
	dma_unmap_sg(se_dev->dev, &se_dev->sg, 1, DMA_BIDIRECTIONAL);
	kfree(se_dev->aes_buf);
mem_out:
	for (i = 0; i < se_dev->req_cnt; i++) {
		req = se_dev->reqs[i];
		req->base.complete(&req->base, err);
	}
	se_dev->req_cnt = 0;
	se_dev->gather_buf_sz = 0;
	se_dev->cmdbuf_cnt = 0;
	se_dev->dynamic_mem = false;
}

static void tegra_se_work_handler(struct work_struct *work)
{
	struct tegra_se_dev *se_dev = container_of(work, struct tegra_se_dev,
						   se_work);
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	struct ablkcipher_request *req;
	bool process_requests;

	mutex_lock(&se_dev->mtx);
	do {
		process_requests = false;
		mutex_lock(&se_dev->lock);
		do {
			backlog = crypto_get_backlog(&se_dev->queue);
			async_req = crypto_dequeue_request(&se_dev->queue);
			if (!async_req)
				se_dev->work_q_busy = false;

			if (backlog) {
				backlog->complete(backlog, -EINPROGRESS);
				backlog = NULL;
			}

			if (async_req) {
				req = ablkcipher_request_cast(async_req);
				se_dev->reqs[se_dev->req_cnt] = req;
				se_dev->gather_buf_sz += req->nbytes;
				se_dev->req_cnt++;
				process_requests = true;
			} else {
				break;
			}
		} while (se_dev->queue.qlen &&
			 (se_dev->req_cnt < SE_MAX_TASKS_PER_SUBMIT));
		mutex_unlock(&se_dev->lock);

		if (process_requests)
			tegra_se_process_new_req(se_dev);
	} while (se_dev->work_q_busy);
	mutex_unlock(&se_dev->mtx);
}

static int tegra_se_aes_queue_req(struct tegra_se_dev *se_dev,
				  struct ablkcipher_request *req)
{
	int err = 0;

	mutex_lock(&se_dev->lock);
	err = ablkcipher_enqueue_request(&se_dev->queue, req);

	if (!se_dev->work_q_busy) {
		se_dev->work_q_busy = true;
		mutex_unlock(&se_dev->lock);
		queue_work(se_dev->se_work_q, &se_dev->se_work);
	} else {
		mutex_unlock(&se_dev->lock);
	}

	return err;
}

static int tegra_se_aes_xts_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	if (!req_ctx->se_dev) {
		pr_err("Device is NULL\n");
		return -ENODEV;
	}

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_XTS;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_xts_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	if (!req_ctx->se_dev) {
		pr_err("Device is NULL\n");
		return -ENODEV;
	}

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_XTS;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->se_dev = se_devices[SE_AES];
	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;

	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static void tegra_se_init_aesbuf(struct tegra_se_dev *se_dev)
{
	int i;
	void *buf = se_dev->total_aes_buf;
	dma_addr_t buf_addr = se_dev->total_aes_buf_addr;

	for (i = 0; i < SE_MAX_AESBUF_ALLOC; i++) {
		se_dev->aes_bufs[i] = buf + (i * SE_MAX_GATHER_BUF_SZ);
		if (se_dev->ioc)
			se_dev->aes_buf_addrs[i] = buf_addr +
						(i * SE_MAX_GATHER_BUF_SZ);
		atomic_set(&se_dev->aes_buf_stat[i], 1);
	}
}

static int tegra_se_aes_setkey(struct crypto_ablkcipher *tfm,
			       const u8 *key, u32 keylen)
{
	struct tegra_se_aes_context *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_se_dev *se_dev;
	struct tegra_se_slot *pslot;
	u8 *pdata = (u8 *)key;
	int ret = 0;
	unsigned int index = 0;
	u32 *cpuvaddr = NULL;
	dma_addr_t iova = 0;

	se_dev = se_devices[SE_AES];

	if (!ctx || !se_dev) {
		pr_err("invalid context or dev");
		return -EINVAL;
	}
	ctx->se_dev = se_dev;

	if (((keylen & SE_KEY_LEN_MASK) != TEGRA_SE_KEY_128_SIZE) &&
	    ((keylen & SE_KEY_LEN_MASK) != TEGRA_SE_KEY_192_SIZE) &&
	    ((keylen & SE_KEY_LEN_MASK) != TEGRA_SE_KEY_256_SIZE) &&
	    ((keylen & SE_KEY_LEN_MASK) != TEGRA_SE_KEY_512_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		return -EINVAL;
	}

	if ((keylen >> SE_MAGIC_PATTERN_OFFSET) == SE_STORE_KEY_IN_MEM) {
		ctx->is_key_in_mem = true;
		ctx->keylen = (keylen & SE_KEY_LEN_MASK);
		ctx->slot = &keymem_slot;
		memcpy(ctx->key, key, ctx->keylen);
		return 0;
	}
	ctx->is_key_in_mem = false;

	mutex_lock(&se_dev->mtx);
	if (key) {
		if (!ctx->slot ||
		    (ctx->slot &&
		     ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				mutex_unlock(&se_dev->mtx);
				return -ENOMEM;
			}
			ctx->slot = pslot;
		}
		ctx->keylen = keylen;
	} else if ((keylen >> SE_MAGIC_PATTERN_OFFSET) == SE_MAGIC_PATTERN) {
		ctx->slot = &pre_allocated_slot;
		spin_lock(&key_slot_lock);
		pre_allocated_slot.slot_num =
			((keylen & SE_SLOT_NUM_MASK) >> SE_SLOT_POSITION);
		spin_unlock(&key_slot_lock);
		ctx->keylen = (keylen & SE_KEY_LEN_MASK);
		goto out;
	} else {
		tegra_se_free_key_slot(ctx->slot);
		ctx->slot = &ssk_slot;
		ctx->keylen = AES_KEYSIZE_128;
		goto out;
	}

	ret = tegra_se_get_free_cmdbuf(se_dev);
	if (ret < 0) {
		dev_err(se_dev->dev, "Couldn't get free cmdbuf\n");
		goto keyslt_free;
	}

	index = ret;

	cpuvaddr = se_dev->cmdbuf_addr_list[index].cmdbuf_addr;
	iova = se_dev->cmdbuf_addr_list[index].iova;
	atomic_set(&se_dev->cmdbuf_addr_list[index].free, 0);
	se_dev->cmdbuf_list_entry = index;

	/* load the key */

	if (strcmp(crypto_tfm_alg_name(&tfm->base), "xts(aes)")) {
		ret = tegra_se_send_key_data(
			se_dev, pdata, keylen, ctx->slot->slot_num,
			SE_KEY_TABLE_TYPE_KEY, se_dev->opcode_addr, cpuvaddr,
			iova, AES_CB);
	} else {
		keylen = keylen / 2;
		ret = tegra_se_send_key_data(
			se_dev, pdata, keylen, ctx->slot->slot_num,
			SE_KEY_TABLE_TYPE_XTS_KEY1, se_dev->opcode_addr,
			cpuvaddr, iova, AES_CB);
		if (ret)
			goto keyslt_free;

		ret = tegra_se_send_key_data(se_dev, pdata + keylen, keylen,
					     ctx->slot->slot_num,
					     SE_KEY_TABLE_TYPE_XTS_KEY2,
					     se_dev->opcode_addr, cpuvaddr,
					     iova, AES_CB);
	}
keyslt_free:
	if (ret)
		tegra_se_free_key_slot(ctx->slot);
out:
	mutex_unlock(&se_dev->mtx);

	return ret;
}

static int tegra_se_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct tegra_se_req_context);

	return 0;
}

static void tegra_se_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_context *ctx = crypto_tfm_ctx(tfm);

	tegra_se_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

static int tegra_se_rng_drbg_init(struct crypto_tfm *tfm)
{
	struct tegra_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);
	struct tegra_se_dev *se_dev;

	se_dev = se_devices[SE_DRBG];

	mutex_lock(&se_dev->mtx);

	rng_ctx->se_dev = se_dev;
	rng_ctx->dt_buf = dma_alloc_coherent(se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
					     &rng_ctx->dt_buf_adr, GFP_KERNEL);
	if (!rng_ctx->dt_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}

	rng_ctx->rng_buf = dma_alloc_coherent(
				rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				&rng_ctx->rng_buf_adr, GFP_KERNEL);
	if (!rng_ctx->rng_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->dt_buf, rng_ctx->dt_buf_adr);
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}
	mutex_unlock(&se_dev->mtx);

	return 0;
}

static int tegra_se_rng_drbg_get_random(struct crypto_rng *tfm, const u8 *src,
					unsigned int slen, u8 *rdata,
					unsigned int dlen)
{
	struct tegra_se_rng_context *rng_ctx = crypto_rng_ctx(tfm);
	struct tegra_se_dev *se_dev = rng_ctx->se_dev;
	u8 *rdata_addr;
	int ret = 0, j;
	unsigned int num_blocks, data_len = 0;
	struct tegra_se_req_context *req_ctx =
		devm_kzalloc(se_dev->dev,
			     sizeof(struct tegra_se_req_context), GFP_KERNEL);
	if (!req_ctx)
		return -ENOMEM;

	num_blocks = (dlen / TEGRA_SE_RNG_DT_SIZE);
	data_len = (dlen % TEGRA_SE_RNG_DT_SIZE);
	if (data_len == 0)
		num_blocks = num_blocks - 1;

	mutex_lock(&se_dev->mtx);
	req_ctx->op_mode = SE_AES_OP_MODE_RNG_DRBG;

	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);
	se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);

	req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode, true,
					      TEGRA_SE_KEY_128_SIZE);
	req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
							    req_ctx->op_mode,
							    true, 0, true);
	for (j = 0; j <= num_blocks; j++) {
		se_dev->src_ll->addr = rng_ctx->dt_buf_adr;
		se_dev->src_ll->data_len = TEGRA_SE_RNG_DT_SIZE;
		se_dev->dst_ll->addr = rng_ctx->rng_buf_adr;
		se_dev->dst_ll->data_len = TEGRA_SE_RNG_DT_SIZE;

		tegra_se_send_data(se_dev, req_ctx, NULL, TEGRA_SE_RNG_DT_SIZE,
				   se_dev->opcode_addr,
				   se_dev->aes_cmdbuf_cpuvaddr);
		ret = tegra_se_channel_submit_gather(
				se_dev, se_dev->aes_cmdbuf_cpuvaddr,
				se_dev->aes_cmdbuf_iova, 0, se_dev->cmdbuf_cnt,
				NONE);
		if (ret)
			break;

		rdata_addr = (rdata + (j * TEGRA_SE_RNG_DT_SIZE));

		if (data_len && num_blocks == j)
			memcpy(rdata_addr, rng_ctx->rng_buf, data_len);
		else
			memcpy(rdata_addr, rng_ctx->rng_buf,
			       TEGRA_SE_RNG_DT_SIZE);
	}

	if (!ret)
		ret = dlen;

	mutex_unlock(&se_dev->mtx);
	devm_kfree(se_dev->dev, req_ctx);

	return ret;
}

static int tegra_se_rng_drbg_reset(struct crypto_rng *tfm, const u8 *seed,
				   unsigned int slen)
{
	return 0;
}

static void tegra_se_rng_drbg_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);

	if (rng_ctx->dt_buf)
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->dt_buf, rng_ctx->dt_buf_adr);

	if (rng_ctx->rng_buf)
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->rng_buf, rng_ctx->rng_buf_adr);

	rng_ctx->se_dev = NULL;
}

static void tegra_se_sha_copy_residual_data(
		struct ahash_request *req, struct tegra_se_sha_context *sha_ctx,
		u32 bytes_to_copy)
{
	struct sg_mapping_iter miter;
	unsigned int sg_flags, total = 0;
	u32 num_sgs, last_block_bytes = bytes_to_copy;
	unsigned long flags;
	struct scatterlist *src_sg;
	u8 *temp_buffer = NULL;

	src_sg = req->src;
	num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
	sg_flags = SG_MITER_ATOMIC | SG_MITER_FROM_SG;
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	local_irq_save(flags);

	temp_buffer = sha_ctx->sha_buf[0];
	while (sg_miter_next(&miter) && total < req->nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(req->nbytes - total));
		if ((req->nbytes - (total + len)) <= last_block_bytes) {
			bytes_to_copy = last_block_bytes -
					(req->nbytes - (total + len));
			memcpy(temp_buffer, miter.addr + (len - bytes_to_copy),
			       bytes_to_copy);
			last_block_bytes -= bytes_to_copy;
			temp_buffer += bytes_to_copy;
		}
		total += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);
}

static int tegra_se_sha_process_buf(struct ahash_request *req, bool is_last,
				    bool process_cur_req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = ahash_request_ctx(req);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	struct scatterlist *src_sg = req->src;
	struct tegra_se_ll *src_ll;
	struct tegra_se_ll *dst_ll;
	u32 current_total = 0, num_sgs, bytes_process_in_req = 0, num_blks;
	int err = 0;

	sg_init_one(&se_dev->sg, req->result, req->nbytes);

	se_dev->sha_last = is_last;

	if (is_last) {
		/* Prepare buf for residual and current data */
		/* Fill sgs entries */
		se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);
		src_ll = se_dev->src_ll;
		se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);
		dst_ll = se_dev->dst_ll;

		if (sha_ctx->residual_bytes) {
			src_ll->addr = sha_ctx->sha_buf_addr[0];
			src_ll->data_len = sha_ctx->residual_bytes;
			src_ll++;
		}

		if (process_cur_req) {
			bytes_process_in_req = req->nbytes;
			err = tegra_map_sg(se_dev->dev, src_sg, 1,
					   DMA_TO_DEVICE, src_ll,
					   bytes_process_in_req);
			if (!err)
				return -EINVAL;

			current_total = req->nbytes + sha_ctx->residual_bytes;
			sha_ctx->total_count += current_total;

			err = tegra_map_sg(se_dev->dev, &se_dev->sg, 1,
					   DMA_FROM_DEVICE, dst_ll,
					   bytes_process_in_req);
			if (!err)
				return -EINVAL;

			se_dev->bytes_mapped = bytes_process_in_req;
			se_dev->sha_src_mapped = true;
			se_dev->sha_dst_mapped = true;
		} else {
			current_total = sha_ctx->residual_bytes;
			sha_ctx->total_count += current_total;
			if (!current_total) {
				req->base.complete(&req->base, 0);
				return 0;
			}
			err = tegra_map_sg(se_dev->dev, &se_dev->sg, 1,
					   DMA_FROM_DEVICE, dst_ll,
					   req->nbytes);
			if (!err)
				return -EINVAL;

			se_dev->sha_dst_mapped = true;
		}
	} else {
		current_total = req->nbytes + sha_ctx->residual_bytes;
		num_blks = current_total / sha_ctx->blk_size;

		/* If total bytes is less than or equal to one blk,
		 * copy to residual and return.
		 */
		if (num_blks <= 1) {
			num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
			sg_copy_to_buffer(req->src, num_sgs,
					  sha_ctx->sha_buf[0] +
					  sha_ctx->residual_bytes, req->nbytes);
			sha_ctx->residual_bytes += req->nbytes;
			req->base.complete(&req->base, 0);
			return 0;
		}

		/* Number of bytes to be processed from given request buffers */
		bytes_process_in_req = (num_blks * sha_ctx->blk_size) -
					sha_ctx->residual_bytes;
		sha_ctx->total_count += bytes_process_in_req;

		/* Fill sgs entries */
		/* If residual bytes are present copy it to second buffer */
		if (sha_ctx->residual_bytes)
			memcpy(sha_ctx->sha_buf[1], sha_ctx->sha_buf[0],
			       sha_ctx->residual_bytes);

		sha_ctx->total_count += sha_ctx->residual_bytes;

		se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);
		src_ll = se_dev->src_ll;
		if (sha_ctx->residual_bytes) {
			src_ll->addr = sha_ctx->sha_buf_addr[1];
			src_ll->data_len = sha_ctx->residual_bytes;
			src_ll++;
		}

		se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);
		dst_ll = se_dev->dst_ll;

		/* Map required bytes to process in given request */
		err = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
				   src_ll, bytes_process_in_req);
		if (!err)
			return -EINVAL;

		/* Copy residual data */
		sha_ctx->residual_bytes = current_total - (num_blks *
							   sha_ctx->blk_size);
		tegra_se_sha_copy_residual_data(req, sha_ctx,
						sha_ctx->residual_bytes);

		/* Total bytes to be processed */
		current_total = (num_blks * sha_ctx->blk_size);

		err = tegra_map_sg(se_dev->dev, &se_dev->sg, 1, DMA_FROM_DEVICE,
				   dst_ll, bytes_process_in_req);
		if (!err)
			return -EINVAL;

		se_dev->bytes_mapped = bytes_process_in_req;
		se_dev->sha_src_mapped = true;
		se_dev->sha_dst_mapped = true;
	}

	req_ctx->config = tegra_se_get_config(se_dev, sha_ctx->op_mode,
					      false, 0);
	err = tegra_se_send_sha_data(se_dev, req_ctx, sha_ctx,
				     current_total, is_last);
	if (err) {
		if (se_dev->sha_src_mapped)
			tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE,
				       bytes_process_in_req);
		if (se_dev->sha_dst_mapped)
			tegra_unmap_sg(se_dev->dev, &se_dev->sg,
				       DMA_FROM_DEVICE, bytes_process_in_req);
		return err;
	}
	sha_ctx->is_first = false;

	return 0;
}

static int tegra_se_sha_op(struct ahash_request *req, bool is_last,
			   bool process_cur_req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	u32 mode;
	int ret;

	struct tegra_se_sha_zero_length_vector zero_vec[] = {
		{
			.size = SHA1_DIGEST_SIZE,
			.digest = "\xda\x39\xa3\xee\x5e\x6b\x4b\x0d"
				  "\x32\x55\xbf\xef\x95\x60\x18\x90"
				  "\xaf\xd8\x07\x09",
		}, {
			.size = SHA224_DIGEST_SIZE,
			.digest = "\xd1\x4a\x02\x8c\x2a\x3a\x2b\xc9"
				  "\x47\x61\x02\xbb\x28\x82\x34\xc4"
				  "\x15\xa2\xb0\x1f\x82\x8e\xa6\x2a"
				  "\xc5\xb3\xe4\x2f",
		}, {
			.size = SHA256_DIGEST_SIZE,
			.digest = "\xe3\xb0\xc4\x42\x98\xfc\x1c\x14"
				  "\x9a\xfb\xf4\xc8\x99\x6f\xb9\x24"
				  "\x27\xae\x41\xe4\x64\x9b\x93\x4c"
				  "\xa4\x95\x99\x1b\x78\x52\xb8\x55",
		}, {
			.size = SHA384_DIGEST_SIZE,
			.digest = "\x38\xb0\x60\xa7\x51\xac\x96\x38"
				  "\x4c\xd9\x32\x7e\xb1\xb1\xe3\x6a"
				  "\x21\xfd\xb7\x11\x14\xbe\x07\x43"
				  "\x4c\x0c\xc7\xbf\x63\xf6\xe1\xda"
				  "\x27\x4e\xde\xbf\xe7\x6f\x65\xfb"
				  "\xd5\x1a\xd2\xf1\x48\x98\xb9\x5b",
		}, {
			.size = SHA512_DIGEST_SIZE,
			.digest = "\xcf\x83\xe1\x35\x7e\xef\xb8\xbd"
				  "\xf1\x54\x28\x50\xd6\x6d\x80\x07"
				  "\xd6\x20\xe4\x05\x0b\x57\x15\xdc"
				  "\x83\xf4\xa9\x21\xd3\x6c\xe9\xce"
				  "\x47\xd0\xd1\x3c\x5d\x85\xf2\xb0"
				  "\xff\x83\x18\xd2\x87\x7e\xec\x2f"
				  "\x63\xb9\x31\xbd\x47\x41\x7a\x81"
				  "\xa5\x38\x32\x7a\xf9\x27\xda\x3e",
		}
	};

	switch (crypto_ahash_digestsize(tfm)) {
	case SHA1_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA1;
		break;
	case SHA224_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA224;
		break;
	case SHA256_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA256;
		break;
	case SHA384_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA384;
		break;
	case SHA512_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA512;
		break;
	default:
		dev_err(se_dev->dev, "Invalid SHA digest size\n");
		return -EINVAL;
	}
	/* If the request length is zero, */
	if (!req->nbytes) {
		if (sha_ctx->total_count) {
			req->base.complete(&req->base, 0);
			return 0;	/* allow empty packets */
		} else {	/* total_count equals zero */
			if (is_last) {
			/* If this is the last packet SW WAR for zero
			 * length SHA operation since SE HW can't accept
			 * zero length SHA operation.
			 */
				mode = sha_ctx->op_mode - SE_AES_OP_MODE_SHA1;
				memcpy(req->result, zero_vec[mode].digest,
						zero_vec[mode].size);
				req->base.complete(&req->base, 0);
				return 0;
			} else {
				req->base.complete(&req->base, 0);
				return 0;	/* allow empty first packet */
			}
		}
	}
	mutex_lock(&se_dev->mtx);

	ret = tegra_se_sha_process_buf(req, is_last, process_cur_req);
	if (ret) {
		mutex_unlock(&se_dev->mtx);
		return ret;
	}

	mutex_unlock(&se_dev->mtx);

	return 0;
}

static int tegra_se_sha_init(struct ahash_request *req)
{
	struct tegra_se_req_context *req_ctx;
	struct crypto_ahash *tfm = NULL;
	struct tegra_se_sha_context *sha_ctx;
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	tfm = crypto_ahash_reqtfm(req);
	if (!tfm) {
		dev_err(se_dev->dev, "SHA transform not valid\n");
		return -EINVAL;
	}

	sha_ctx = crypto_ahash_ctx(tfm);
	if (!sha_ctx) {
		dev_err(se_dev->dev, "SHA context not valid\n");
		return -EINVAL;
	}

	req_ctx = ahash_request_ctx(req);
	if (!req_ctx) {
		dev_err(se_dev->dev, "Request context not valid\n");
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);

	sha_ctx->total_count = 0;
	sha_ctx->is_first = true;
	sha_ctx->blk_size = crypto_tfm_alg_blocksize(crypto_ahash_tfm(tfm));
	sha_ctx->residual_bytes = 0;
	mutex_unlock(&se_dev->mtx);

	return 0;
}

static int tegra_se_sha_update(struct ahash_request *req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	se_dev->sha_req = req;

	ret = tegra_se_sha_op(req, false, false);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_update failed - %d\n", ret);
	else
		ret = -EBUSY;

	return ret;
}

static int tegra_se_sha_finup(struct ahash_request *req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	se_dev->sha_req = req;

	ret = tegra_se_sha_op(req, true, true);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_finup failed - %d\n", ret);
	else
		ret = -EBUSY;

	return ret;
}

static int tegra_se_sha_final(struct ahash_request *req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	se_dev->sha_req = req;

	/* Do not process data in given request */
	ret = tegra_se_sha_op(req, true, false);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_final failed - %d\n", ret);
	else
		ret = -EBUSY;

	return ret;
}

static int tegra_se_sha_digest(struct ahash_request *req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int ret = 0;

	ret = tegra_se_sha_init(req);
	if (ret)
		return ret;

	se_dev->sha_req = req;

	ret = tegra_se_sha_op(req, true, true);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_digest failed - %d\n", ret);
	else
		ret = -EBUSY;

	return ret;
}

static int tegra_se_sha_export(struct ahash_request *req, void *out)
{
	return 0;
}

static int tegra_se_sha_import(struct ahash_request *req, const void *in)
{
	return 0;
}

static int tegra_se_sha_cra_init(struct crypto_tfm *tfm)
{
	struct tegra_se_sha_context *sha_ctx;
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
			sizeof(struct tegra_se_sha_context));
	sha_ctx = crypto_tfm_ctx(tfm);
	if (!sha_ctx) {
		dev_err(se_dev->dev, "SHA context not valid\n");
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);
	sha_ctx->sha_buf[0] = dma_alloc_coherent(
			se_dev->dev, (TEGRA_SE_SHA_MAX_BLOCK_SIZE * 2),
			&sha_ctx->sha_buf_addr[0], GFP_KERNEL);
	if (!sha_ctx->sha_buf[0]) {
		dev_err(se_dev->dev, "Cannot allocate memory to sha_buf[0]\n");
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}
	sha_ctx->sha_buf[1] = dma_alloc_coherent(
			se_dev->dev, (TEGRA_SE_SHA_MAX_BLOCK_SIZE * 2),
			&sha_ctx->sha_buf_addr[1], GFP_KERNEL);
	if (!sha_ctx->sha_buf[1]) {
		dma_free_coherent(
				se_dev->dev, (TEGRA_SE_SHA_MAX_BLOCK_SIZE * 2),
				sha_ctx->sha_buf[0], sha_ctx->sha_buf_addr[0]);
		sha_ctx->sha_buf[0] = NULL;
		dev_err(se_dev->dev, "Cannot allocate memory to sha_buf[1]\n");
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}
	mutex_unlock(&se_dev->mtx);

	return 0;
}

static void tegra_se_sha_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_sha_context *sha_ctx = crypto_tfm_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int i;

	mutex_lock(&se_dev->mtx);
	for (i = 0; i < 2; i++) {
		/* dma_free_coherent does not panic if addr is NULL */
		dma_free_coherent(
				se_dev->dev, (TEGRA_SE_SHA_MAX_BLOCK_SIZE * 2),
				sha_ctx->sha_buf[i], sha_ctx->sha_buf_addr[i]);
		sha_ctx->sha_buf[i] = NULL;
	}
	mutex_unlock(&se_dev->mtx);
}

static int tegra_se_aes_cmac_init(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_aes_cmac_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_aes_cmac_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_aes_cmac_context *cmac_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = ahash_request_ctx(req);
	struct tegra_se_dev *se_dev;
	struct scatterlist *src_sg;
	struct sg_mapping_iter miter;
	u32 num_sgs, blocks_to_process, last_block_bytes = 0, bytes_to_copy = 0;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	unsigned int total;
	int ret = 0, i = 0;
	bool padding_needed = false;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;
	u8 *temp_buffer = NULL;
	bool use_orig_iv = true;

	se_dev = se_devices[SE_CMAC];

	mutex_lock(&se_dev->mtx);

	req_ctx->op_mode = SE_AES_OP_MODE_CMAC;
	blocks_to_process = req->nbytes / TEGRA_SE_AES_BLOCK_SIZE;
	/* num of bytes less than block size */
	if ((req->nbytes % TEGRA_SE_AES_BLOCK_SIZE) || !blocks_to_process) {
		padding_needed = true;
		last_block_bytes = req->nbytes % TEGRA_SE_AES_BLOCK_SIZE;
	} else {
		/* decrement num of blocks */
		blocks_to_process--;
		if (blocks_to_process)
			last_block_bytes = req->nbytes -
				(blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE);
		else
			last_block_bytes = req->nbytes;
	}

	/* first process all blocks except last block */
	if (blocks_to_process) {
		num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
		if (num_sgs > SE_MAX_SRC_SG_COUNT) {
			dev_err(se_dev->dev, "num of SG buffers are more\n");
			ret = -EDOM;
			goto out;
		}
		se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);

		src_sg = req->src;
		total = blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE;

		ret = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
				   se_dev->src_ll, total);
		if (!ret) {
			ret = -EINVAL;
			goto out;
		}

		req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
						      true, cmac_ctx->keylen);
		/* write zero IV */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);

		ret = tegra_se_send_key_data(
			se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_ORGIV,
			se_dev->opcode_addr, se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, NONE);
		if (ret)
			goto out;

		req_ctx->crypto_config = tegra_se_get_crypto_config(
				se_dev, req_ctx->op_mode, true,
				cmac_ctx->slot->slot_num, true);

		tegra_se_send_data(se_dev, req_ctx, NULL, total,
				   se_dev->opcode_addr,
				   se_dev->aes_cmdbuf_cpuvaddr);
		ret = tegra_se_channel_submit_gather(
			se_dev, se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, 0, se_dev->cmdbuf_cnt, NONE);
		if (ret)
			goto out;

		tegra_se_read_cmac_result(se_dev, piv,
					  TEGRA_SE_AES_CMAC_DIGEST_SIZE, false);
		src_sg = req->src;
		tegra_unmap_sg(se_dev->dev, src_sg,  DMA_TO_DEVICE, total);
		use_orig_iv = false;
	}

	/* get the last block bytes from the sg_dma buffer using miter */
	src_sg = req->src;
	num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
	sg_flags |= SG_MITER_FROM_SG;
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	total = 0;
	cmac_ctx->buffer = dma_alloc_coherent(se_dev->dev,
					      TEGRA_SE_AES_BLOCK_SIZE,
					      &cmac_ctx->dma_addr, GFP_KERNEL);
	if (!cmac_ctx->buffer) {
		ret = -ENOMEM;
		goto out;
	}

	local_irq_save(flags);
	temp_buffer = cmac_ctx->buffer;
	while (sg_miter_next(&miter) && total < req->nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(req->nbytes - total));
		if ((req->nbytes - (total + len)) <= last_block_bytes) {
			bytes_to_copy = last_block_bytes -
				(req->nbytes - (total + len));
			memcpy(temp_buffer, miter.addr + (len - bytes_to_copy),
			       bytes_to_copy);
			last_block_bytes -= bytes_to_copy;
			temp_buffer += bytes_to_copy;
		}
		total += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);

	/* process last block */
	if (padding_needed) {
		/* pad with 0x80, 0, 0 ... */
		last_block_bytes = req->nbytes % TEGRA_SE_AES_BLOCK_SIZE;
		cmac_ctx->buffer[last_block_bytes] = 0x80;
		for (i = last_block_bytes + 1; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] = 0;
		/* XOR with K2 */
		for (i = 0; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] ^= cmac_ctx->K2[i];
	} else {
		/* XOR with K1 */
		for (i = 0; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] ^= cmac_ctx->K1[i];
	}

	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);

	se_dev->src_ll->addr = cmac_ctx->dma_addr;
	se_dev->src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	if (use_orig_iv) {
		/* use zero IV, this is when num of bytes is
		 * less <= block size
		 */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);
		ret = tegra_se_send_key_data(
			se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_ORGIV,
			se_dev->opcode_addr, se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, NONE);
	} else {
		ret = tegra_se_send_key_data(
			se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_UPDTDIV,
			se_dev->opcode_addr, se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, NONE);
	}

	if (ret)
		goto out;

	req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
					      true, cmac_ctx->keylen);
	req_ctx->crypto_config = tegra_se_get_crypto_config(
				se_dev, req_ctx->op_mode, true,
				cmac_ctx->slot->slot_num, use_orig_iv);

	tegra_se_send_data(se_dev, req_ctx, NULL, TEGRA_SE_AES_BLOCK_SIZE,
			   se_dev->opcode_addr, se_dev->aes_cmdbuf_cpuvaddr);
	ret = tegra_se_channel_submit_gather(
			se_dev, se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, 0, se_dev->cmdbuf_cnt, NONE);
	if (ret)
		goto out;
	tegra_se_read_cmac_result(se_dev, req->result,
				  TEGRA_SE_AES_CMAC_DIGEST_SIZE, false);
out:
	if (cmac_ctx->buffer)
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
				  cmac_ctx->buffer, cmac_ctx->dma_addr);
	mutex_unlock(&se_dev->mtx);

	return ret;
}

static int tegra_se_aes_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
				    unsigned int keylen)
{
	struct tegra_se_aes_cmac_context *ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = NULL;
	struct tegra_se_dev *se_dev;
	struct tegra_se_slot *pslot;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	u32 *pbuf;
	dma_addr_t pbuf_adr;
	int ret = 0;
	u8 const rb = 0x87;
	u8 msb;

	se_dev = se_devices[SE_CMAC];

	mutex_lock(&se_dev->mtx);

	if (!ctx) {
		dev_err(se_dev->dev, "invalid context");
		mutex_unlock(&se_dev->mtx);
		return -EINVAL;
	}

	req_ctx = devm_kzalloc(se_dev->dev, sizeof(struct tegra_se_req_context),
			       GFP_KERNEL);
	if (!req_ctx) {
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}
	if ((keylen != TEGRA_SE_KEY_128_SIZE) &&
	    (keylen != TEGRA_SE_KEY_192_SIZE) &&
	    (keylen != TEGRA_SE_KEY_256_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		ret = -EINVAL;
		goto free_ctx;
	}

	if (key) {
		if (!ctx->slot ||
		    (ctx->slot &&
		     ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				ret = -ENOMEM;
				goto free_ctx;
			}
			ctx->slot = pslot;
		}
		ctx->keylen = keylen;
	} else {
		tegra_se_free_key_slot(ctx->slot);
		ctx->slot = &ssk_slot;
		ctx->keylen = AES_KEYSIZE_128;
	}

	pbuf = dma_alloc_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
				  &pbuf_adr, GFP_KERNEL);
	if (!pbuf) {
		dev_err(se_dev->dev, "can not allocate dma buffer");
		ret = -ENOMEM;
		goto keyslt_free;
	}
	memset(pbuf, 0, TEGRA_SE_AES_BLOCK_SIZE);

	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);
	se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);

	se_dev->src_ll->addr = pbuf_adr;
	se_dev->src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;
	se_dev->dst_ll->addr = pbuf_adr;
	se_dev->dst_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	/* load the key */
	ret = tegra_se_send_key_data(
		se_dev, (u8 *)key, keylen, ctx->slot->slot_num,
		SE_KEY_TABLE_TYPE_KEY, se_dev->opcode_addr,
		se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova, NONE);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_send_key_data for loading cmac key failed\n");
		goto out;
	}

	/* write zero IV */
	memset(piv, 0, TEGRA_SE_AES_IV_SIZE);

	/* load IV */
	ret = tegra_se_send_key_data(
		se_dev, piv, TEGRA_SE_AES_IV_SIZE, ctx->slot->slot_num,
		SE_KEY_TABLE_TYPE_ORGIV, se_dev->opcode_addr,
		se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova, NONE);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_send_key_data for loading cmac iv failed\n");
		goto out;
	}

	/* config crypto algo */
	req_ctx->config = tegra_se_get_config(se_dev, SE_AES_OP_MODE_CBC,
					      true, keylen);
	req_ctx->crypto_config = tegra_se_get_crypto_config(
				se_dev, SE_AES_OP_MODE_CBC, true,
				ctx->slot->slot_num, true);

	tegra_se_send_data(se_dev, req_ctx, NULL, TEGRA_SE_AES_BLOCK_SIZE,
			   se_dev->opcode_addr, se_dev->aes_cmdbuf_cpuvaddr);
	ret = tegra_se_channel_submit_gather(
		se_dev, se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova,
		0, se_dev->cmdbuf_cnt, NONE);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_aes_cmac_setkey:: start op failed\n");
		goto out;
	}

	/* compute K1 subkey */
	memcpy(ctx->K1, pbuf, TEGRA_SE_AES_BLOCK_SIZE);
	tegra_se_leftshift_onebit(ctx->K1, TEGRA_SE_AES_BLOCK_SIZE, &msb);
	if (msb)
		ctx->K1[TEGRA_SE_AES_BLOCK_SIZE - 1] ^= rb;

	/* compute K2 subkey */
	memcpy(ctx->K2, ctx->K1, TEGRA_SE_AES_BLOCK_SIZE);
	tegra_se_leftshift_onebit(ctx->K2, TEGRA_SE_AES_BLOCK_SIZE, &msb);

	if (msb)
		ctx->K2[TEGRA_SE_AES_BLOCK_SIZE - 1] ^= rb;
out:
	if (pbuf)
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
				  pbuf, pbuf_adr);
keyslt_free:
	if (ret)
		tegra_se_free_key_slot(ctx->slot);
free_ctx:
	devm_kfree(se_dev->dev, req_ctx);
	mutex_unlock(&se_dev->mtx);

	return ret;
}

static int tegra_se_aes_cmac_digest(struct ahash_request *req)
{
	return tegra_se_aes_cmac_init(req) ?: tegra_se_aes_cmac_final(req);
}

static int tegra_se_aes_cmac_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_aes_cmac_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_se_aes_cmac_context));

	return 0;
}

static void tegra_se_aes_cmac_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_cmac_context *ctx = crypto_tfm_ctx(tfm);

	tegra_se_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

/* Security Engine rsa key slot */
struct tegra_se_rsa_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	bool available; /* Tells whether key slot is free to use */
};

/* Security Engine AES RSA context */
struct tegra_se_aes_rsa_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_rsa_slot *slot;	/* Security Engine rsa key slot */
	u32 mod_len;
	u32 exp_len;
};

static void tegra_se_rsa_free_key_slot(struct tegra_se_rsa_slot *slot)
{
	if (slot) {
		spin_lock(&rsa_key_slot_lock);
		slot->available = true;
		spin_unlock(&rsa_key_slot_lock);
	}
}

static struct tegra_se_rsa_slot *tegra_se_alloc_rsa_key_slot(void)
{
	struct tegra_se_rsa_slot *slot = NULL;
	bool found = false;

	spin_lock(&rsa_key_slot_lock);
	list_for_each_entry(slot, &rsa_key_slot, node) {
		if (slot->available) {
			slot->available = false;
			found = true;
			break;
		}
	}
	spin_unlock(&rsa_key_slot_lock);

	return found ? slot : NULL;
}

static int tegra_init_rsa_key_slot(struct tegra_se_dev *se_dev)
{
	int i;

	se_dev->rsa_slot_list = devm_kzalloc(
			se_dev->dev, sizeof(struct tegra_se_rsa_slot) *
			TEGRA_SE_RSA_KEYSLOT_COUNT, GFP_KERNEL);
	if (!se_dev->rsa_slot_list)
		return -ENOMEM;

	spin_lock_init(&rsa_key_slot_lock);
	spin_lock(&rsa_key_slot_lock);
	for (i = 0; i < TEGRA_SE_RSA_KEYSLOT_COUNT; i++) {
		se_dev->rsa_slot_list[i].available = true;
		se_dev->rsa_slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->rsa_slot_list[i].node);
		list_add_tail(&se_dev->rsa_slot_list[i].node, &rsa_key_slot);
	}
	spin_unlock(&rsa_key_slot_lock);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static unsigned int tegra_se_rsa_max_size(struct crypto_akcipher *tfm)
#else
static int tegra_se_rsa_max_size(struct crypto_akcipher *tfm)
#endif
{
	struct tegra_se_aes_rsa_context *ctx = akcipher_tfm_ctx(tfm);

	if (!ctx) {
		pr_err("No RSA context\n");
		return -EINVAL;
	}

	return ctx->mod_len;
}

static int tegra_se_send_rsa_data(struct tegra_se_dev *se_dev,
				  struct tegra_se_aes_rsa_context *rsa_ctx)
{
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	u32 cmdbuf_num_words = 0, i = 0;
	int err = 0;
	u32 val = 0;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_4K,
					  &cmdbuf_iova, GFP_KERNEL,
					  __DMA_ATTR(attrs));
	if (!cmdbuf_cpuvaddr) {
		dev_err(se_dev->dev, "Failed to allocate memory for cmdbuf\n");
		return -ENOMEM;
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	val = SE_CONFIG_ENC_ALG(ALG_RSA) |
		SE_CONFIG_DEC_ALG(ALG_NOP) |
		SE_CONFIG_DST(DST_MEMORY);
	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(se_dev->opcode_addr, 8);
	cmdbuf_cpuvaddr[i++] = val;
	cmdbuf_cpuvaddr[i++] = RSA_KEY_SLOT(rsa_ctx->slot->slot_num);
	cmdbuf_cpuvaddr[i++] = (rsa_ctx->mod_len / 64) - 1;
	cmdbuf_cpuvaddr[i++] = (rsa_ctx->exp_len / 4);
	cmdbuf_cpuvaddr[i++] = (u32)(se_dev->src_ll->addr);
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->src_ll->addr)) |
				SE_ADDR_HI_SZ(se_dev->src_ll->data_len));
	cmdbuf_cpuvaddr[i++] = (u32)(se_dev->dst_ll->addr);
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->dst_ll->addr)) |
				SE_ADDR_HI_SZ(se_dev->dst_ll->data_len));
	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
				SE_OPERATION_OP(OP_START);

	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(se_dev, cmdbuf_cpuvaddr,
					     cmdbuf_iova, 0, cmdbuf_num_words,
					     NONE);

	dma_free_attrs(se_dev->dev->parent, SZ_4K, cmdbuf_cpuvaddr,
		       cmdbuf_iova, __DMA_ATTR(attrs));

	return err;
}

static int tegra_se_rsa_setkey(struct crypto_akcipher *tfm, const void *key,
			       unsigned int keylen)
{
	struct tegra_se_aes_rsa_context *ctx = akcipher_tfm_ctx(tfm);
	struct tegra_se_dev *se_dev;
	u32 module_key_length = 0;
	u32 exponent_key_length = 0;
	u32 pkt, val;
	u32 key_size_words;
	u32 key_word_size = 4;
	u32 *pkeydata = (u32 *)key;
	s32 j = 0;
	struct tegra_se_rsa_slot *pslot;
	u32 cmdbuf_num_words = 0, i = 0;
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	int err = 0;
	int timeout = 0;

	se_dev = se_devices[SE_RSA];

	if (!ctx || !key) {
		dev_err(se_dev->dev, "No RSA context or Key\n");
		return -EINVAL;
	}

	/* Allocate rsa key slot */
	if (!ctx->slot) {
		for (timeout = 0; timeout < SE_KEYSLOT_TIMEOUT; timeout++) {
			pslot = tegra_se_alloc_rsa_key_slot();
			if (!pslot) {
				mdelay(SE_KEYSLOT_MDELAY);
				continue;
			} else {
				break;
			}
		}

		if (!pslot) {
			dev_err(se_dev->dev, "no free key slot\n");
			return -ENOMEM;
		} else {
			ctx->slot = pslot;
		}
	}

	module_key_length = (keylen >> 16);
	exponent_key_length = (keylen & (0xFFFF));

	if (!(((module_key_length / 64) >= 1) &&
	      ((module_key_length / 64) <= 4))) {
		tegra_se_rsa_free_key_slot(ctx->slot);
		dev_err(se_dev->dev, "Invalid RSA modulus length\n");
		return -EDOM;
	}

	ctx->mod_len = module_key_length;
	ctx->exp_len = exponent_key_length;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_64K,
					  &cmdbuf_iova, GFP_KERNEL,
					  __DMA_ATTR(attrs));
	if (!cmdbuf_cpuvaddr) {
		tegra_se_rsa_free_key_slot(ctx->slot);
		dev_err(se_dev->dev, "Failed to allocate memory for cmdbuf\n");
		return -ENOMEM;
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	if (exponent_key_length) {
		key_size_words = (exponent_key_length / key_word_size);
		/* Write exponent */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_ADDR_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_DATA_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = *pkeydata++;
		}
	}

	if (module_key_length) {
		key_size_words = (module_key_length / key_word_size);
		/* Write modulus */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_ADDR_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_DATA_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = *pkeydata++;
		}
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
				SE_OPERATION_OP(OP_DUMMY);
	cmdbuf_num_words = i;

	mutex_lock(&se_dev->mtx);
	err = tegra_se_channel_submit_gather(se_dev, cmdbuf_cpuvaddr,
					     cmdbuf_iova, 0, cmdbuf_num_words,
					     NONE);
	mutex_unlock(&se_dev->mtx);
	if (err)
		tegra_se_rsa_free_key_slot(ctx->slot);
	dma_free_attrs(se_dev->dev->parent, SZ_64K, cmdbuf_cpuvaddr,
		       cmdbuf_iova, __DMA_ATTR(attrs));

	return err;
}

static int tegra_se_rsa_op(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = NULL;
	struct tegra_se_aes_rsa_context *rsa_ctx = NULL;
	struct tegra_se_dev *se_dev;
	u32 num_src_sgs, num_dst_sgs;
	int ret1 = 0, ret2 = 0;

	se_dev = se_devices[SE_RSA];

	if (!req) {
		dev_err(se_dev->dev, "Invalid RSA request\n");
		return -EINVAL;
	}

	tfm = crypto_akcipher_reqtfm(req);
	if (!tfm) {
		dev_err(se_dev->dev, "Invalid RSA transform\n");
		return -EINVAL;
	}

	rsa_ctx = akcipher_tfm_ctx(tfm);
	if (!rsa_ctx || !rsa_ctx->slot) {
		dev_err(se_dev->dev, "Invalid RSA context\n");
		return -EINVAL;
	}

	if ((req->src_len < TEGRA_SE_RSA512_INPUT_SIZE) ||
	    (req->src_len > TEGRA_SE_RSA2048_INPUT_SIZE)) {
		dev_err(se_dev->dev, "RSA src input length not in range\n");
		return -EDOM;
	}

	if ((req->dst_len < TEGRA_SE_RSA512_INPUT_SIZE) ||
	    (req->dst_len > TEGRA_SE_RSA2048_INPUT_SIZE)) {
		dev_err(se_dev->dev, "RSA dst input length not in range\n");
		return -EDOM;
	}

	if (req->src_len != rsa_ctx->mod_len) {
		dev_err(se_dev->dev, "Invalid RSA src input length\n");
		return -EINVAL;
	}

	num_src_sgs = tegra_se_count_sgs(req->src, req->src_len);
	num_dst_sgs = tegra_se_count_sgs(req->dst, req->dst_len);
	if ((num_src_sgs > SE_MAX_SRC_SG_COUNT) ||
	    (num_dst_sgs > SE_MAX_DST_SG_COUNT)) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EDOM;
	}

	mutex_lock(&se_dev->mtx);
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);

	if (req->src == req->dst) {
		se_dev->dst_ll = se_dev->src_ll;
		ret1 = tegra_map_sg(se_dev->dev, req->src, 1, DMA_BIDIRECTIONAL,
				    se_dev->src_ll, req->src_len);
		if (!ret1) {
			mutex_unlock(&se_dev->mtx);
			return -EINVAL;
		}
	} else {
		se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);
		ret1 = tegra_map_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE,
				    se_dev->src_ll, req->src_len);
		ret2 = tegra_map_sg(se_dev->dev, req->dst, 1, DMA_FROM_DEVICE,
				    se_dev->dst_ll, req->dst_len);
		if (!ret1 || !ret2) {
			mutex_unlock(&se_dev->mtx);
			return -EINVAL;
		}
	}

	ret1 = tegra_se_send_rsa_data(se_dev, rsa_ctx);
	if (ret1)
		dev_err(se_dev->dev, "RSA send data failed err = %d\n", ret1);

	if (req->src == req->dst) {
		tegra_unmap_sg(se_dev->dev, req->src, DMA_BIDIRECTIONAL,
			       req->src_len);
	} else {
		tegra_unmap_sg(se_dev->dev, req->src, DMA_TO_DEVICE,
			       req->src_len);
		tegra_unmap_sg(se_dev->dev, req->dst, DMA_FROM_DEVICE,
			       req->dst_len);
	}

	mutex_unlock(&se_dev->mtx);
	return ret1;
}

static void tegra_se_rsa_exit(struct crypto_akcipher *tfm)
{
	struct tegra_se_aes_rsa_context *ctx = akcipher_tfm_ctx(tfm);

	tegra_se_rsa_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

static inline struct tegra_se_dh_context *tegra_se_dh_get_ctx(
						struct crypto_kpp *tfm)
{
	return kpp_tfm_ctx(tfm);
}

static int tegra_se_dh_check_params_length(unsigned int p_len)
{
	if (p_len < MIN_DH_SZ_BITS) {
		pr_err("DH Modulus length not in range\n");
		return -EDOM;
	}

	return 0;
}

static int tegra_se_dh_set_params(struct tegra_se_dh_context *ctx,
				  struct dh *params)
{
	int ret = 0;

	ret = tegra_se_dh_check_params_length(params->p_size << 3);
	if (ret)
		return ret;

	ctx->key = (void *)params->key;
	ctx->key_size = params->key_size;
	if (!ctx->key) {
		dev_err(ctx->se_dev->dev, "Invalid DH Key\n");
		return -ENODATA;
	}

	ctx->p = (void *)params->p;
	ctx->p_size = params->p_size;
	if (!ctx->p) {
		dev_err(ctx->se_dev->dev, "Invalid DH Modulus\n");
		return -ENODATA;
	}

	ctx->g = (void *)params->g;
	ctx->g_size = params->g_size;
	if (!ctx->g) {
		dev_err(ctx->se_dev->dev, "Invalid DH generator\n");
		return -ENODATA;
	}

	if (ctx->g_size > ctx->p_size) {
		dev_err(ctx->se_dev->dev, "Invalid DH generator size\n");
		return -EDOM;
	}

	return 0;
}

static int tegra_se_dh_setkey(struct crypto_kpp *tfm)
{
	struct tegra_se_dh_context *ctx = tegra_se_dh_get_ctx(tfm);
	struct tegra_se_dev *se_dev;
	u32 module_key_length = 0;
	u32 exponent_key_length = 0;
	u32 pkt, val;
	u32 key_size_words;
	u32 key_word_size = 4;
	u32 *pkeydata, *cmdbuf_cpuvaddr = NULL;
	struct tegra_se_rsa_slot *pslot;
	u32 cmdbuf_num_words = 0;
	dma_addr_t cmdbuf_iova = 0;
	int i = 0, err, j;

	if (!ctx) {
		pr_err("Invalid DH context\n");
		return -EINVAL;
	}

	se_dev = ctx->se_dev;
	pkeydata = (u32 *)ctx->key;

	/* Allocate rsa key slot */
	if (!ctx->slot) {
		pslot = tegra_se_alloc_rsa_key_slot();
		if (!pslot) {
			dev_err(se_dev->dev, "no free key slot\n");
			return -ENOMEM;
		}
		ctx->slot = pslot;
	}

	module_key_length = ctx->p_size;
	exponent_key_length = ctx->key_size;

	if (!(((module_key_length / 64) >= 1) &&
	      ((module_key_length / 64) <= 4))) {
		tegra_se_rsa_free_key_slot(ctx->slot);
		dev_err(se_dev->dev, "DH Modulus length not in range\n");
		return -EDOM;
	}

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_64K,
					  &cmdbuf_iova, GFP_KERNEL,
					  __DMA_ATTR(attrs));
	if (!cmdbuf_cpuvaddr) {
		tegra_se_rsa_free_key_slot(ctx->slot);
		dev_err(se_dev->dev, "Failed to allocate cmdbuf\n");
		return -ENOMEM;
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	if (exponent_key_length) {
		key_size_words = (exponent_key_length / key_word_size);
		/* Write exponent */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_ADDR_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_DATA_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = be32_to_cpu(*pkeydata++);
		}
	}

	if (module_key_length) {
		pkeydata = (u32 *)ctx->p;
		key_size_words = (module_key_length / key_word_size);
		/* Write modulus */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_ADDR_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
					se_dev->opcode_addr +
					SE_RSA_KEYTABLE_DATA_OFFSET, 1);
			cmdbuf_cpuvaddr[i++] = be32_to_cpu(*pkeydata++);
		}
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
				SE_OPERATION_OP(OP_DUMMY);
	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(se_dev, cmdbuf_cpuvaddr,
					     cmdbuf_iova, 0, cmdbuf_num_words,
					     NONE);
	if (err) {
		dev_err(se_dev->dev, "%s: channel_submit failed\n", __func__);
		tegra_se_rsa_free_key_slot(ctx->slot);
	}
	dma_free_attrs(se_dev->dev->parent, SZ_64K, cmdbuf_cpuvaddr,
		       cmdbuf_iova, __DMA_ATTR(attrs));

	return err;
}

static void tegra_se_fix_endianness(struct tegra_se_dev *se_dev,
				    struct scatterlist *sg, u32 num_sgs,
				    unsigned int nbytes, bool be)
{
	int j, k;

	sg_copy_to_buffer(sg, num_sgs, se_dev->dh_buf1, nbytes);

	for (j = (nbytes / 4 - 1), k = 0; j >= 0; j--, k++) {
		if (be)
			se_dev->dh_buf2[k] = be32_to_cpu(se_dev->dh_buf1[j]);
		else
			se_dev->dh_buf2[k] = cpu_to_be32(se_dev->dh_buf1[j]);
	}

	sg_copy_from_buffer(sg, num_sgs, se_dev->dh_buf2, nbytes);
}

static int tegra_se_dh_compute_value(struct kpp_request *req)
{
	struct crypto_kpp *tfm = NULL;
	struct tegra_se_dh_context *dh_ctx = NULL;
	struct tegra_se_dev *se_dev;
	struct scatterlist *src_sg;
	struct tegra_se_ll *src_ll, *dst_ll;
	u32 num_src_sgs, num_dst_sgs;
	u8 *base_buff = NULL;
	struct scatterlist src;
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	u32 cmdbuf_num_words = 0, i = 0;
	int err, j;
	unsigned int total, zpad_sz;
	u32 val;

	if (!req) {
		pr_err("Invalid DH request\n");
		return -EINVAL;
	}

	tfm = crypto_kpp_reqtfm(req);
	if (!tfm) {
		pr_err("Invalid DH transform\n");
		return -EINVAL;
	}

	dh_ctx = tegra_se_dh_get_ctx(tfm);
	if (!dh_ctx || !dh_ctx->slot) {
		pr_err("Invalid DH context\n");
		return -EINVAL;
	}

	se_dev = dh_ctx->se_dev;

	if (req->src) {
		src_sg = req->src;
		total = req->src_len;
	} else {
		base_buff = (u8 *)devm_kzalloc(se_dev->dev,
					       dh_ctx->p_size, GFP_KERNEL);
		if (!base_buff)
			return -ENOMEM;

		if (dh_ctx->g_size < dh_ctx->p_size) {
			zpad_sz = dh_ctx->p_size - dh_ctx->g_size;

			for (j = 0; j < zpad_sz; j++)
				base_buff[j] = 0x0;
			for (j = zpad_sz; j < dh_ctx->p_size; j++)
				base_buff[j] = *(u8 *)(dh_ctx->g++);

			dh_ctx->g_size = dh_ctx->p_size;
		} else {
			memcpy(base_buff, (u8 *)(dh_ctx->g), dh_ctx->g_size);
		}

		sg_init_one(&src, base_buff, dh_ctx->g_size);

		src_sg = &src;
		total = dh_ctx->g_size;
	}

	num_src_sgs = tegra_se_count_sgs(src_sg, total);
	num_dst_sgs = tegra_se_count_sgs(req->dst, req->dst_len);
	if ((num_src_sgs > SE_MAX_SRC_SG_COUNT) ||
	    (num_dst_sgs > SE_MAX_DST_SG_COUNT)) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		err = -EDOM;
		goto free;
	}

	tegra_se_fix_endianness(se_dev, src_sg, num_src_sgs, total, true);

	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf);

	err = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
			   se_dev->src_ll, total);
	if (!err) {
		err = -EINVAL;
		goto free;
	}
	err = tegra_map_sg(se_dev->dev, req->dst, 1, DMA_FROM_DEVICE,
			   se_dev->dst_ll, req->dst_len);
	if (!err) {
		err = -EINVAL;
		goto unmap_src;
	}

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_4K,
					  &cmdbuf_iova, GFP_KERNEL,
					  __DMA_ATTR(attrs));
	if (!cmdbuf_cpuvaddr) {
		dev_err(se_dev->dev, "%s: dma_alloc_attrs failed\n", __func__);
		err = -ENOMEM;
		goto unmap_dst;
	}

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	val = SE_CONFIG_ENC_ALG(ALG_RSA) | SE_CONFIG_DEC_ALG(ALG_NOP) |
		SE_CONFIG_DST(DST_MEMORY);
	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(se_dev->opcode_addr, 8);
	cmdbuf_cpuvaddr[i++] = val;
	cmdbuf_cpuvaddr[i++] = RSA_KEY_SLOT(dh_ctx->slot->slot_num);
	cmdbuf_cpuvaddr[i++] = (dh_ctx->p_size / 64) - 1;
	cmdbuf_cpuvaddr[i++] = (dh_ctx->key_size / 4);
	cmdbuf_cpuvaddr[i++] = (u32)(se_dev->src_ll->addr);
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->src_ll->addr)) |
				SE_ADDR_HI_SZ(se_dev->src_ll->data_len));
	cmdbuf_cpuvaddr[i++] = (u32)(se_dev->dst_ll->addr);
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->dst_ll->addr)) |
				SE_ADDR_HI_SZ(se_dev->dst_ll->data_len));

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_nonincr(
			se_dev->opcode_addr + SE_RSA_OPERATION_OFFSET, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
				SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
				SE_OPERATION_OP(OP_START);

	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(
				se_dev, cmdbuf_cpuvaddr, cmdbuf_iova, 0,
				cmdbuf_num_words, NONE);
	if (err) {
		dev_err(se_dev->dev, "%s: channel_submit failed\n", __func__);
		goto exit;
	}

	tegra_se_fix_endianness(se_dev, req->dst, num_dst_sgs,
				req->dst_len, false);
exit:
	dma_free_attrs(se_dev->dev->parent, SZ_4K, cmdbuf_cpuvaddr,
		       cmdbuf_iova, __DMA_ATTR(attrs));
unmap_dst:
	tegra_unmap_sg(se_dev->dev, req->dst, DMA_FROM_DEVICE, req->dst_len);
unmap_src:
	tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE, total);
free:
	if (!req->src)
		devm_kfree(se_dev->dev, base_buff);

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static int tegra_se_dh_set_secret(struct crypto_kpp *tfm,
				  const void *buf,
				  unsigned int len)
#else
static int tegra_se_dh_set_secret(struct crypto_kpp *tfm, void *buf,
				  unsigned int len)
#endif
{
	int ret = 0;

	struct tegra_se_dh_context *ctx = tegra_se_dh_get_ctx(tfm);
	struct dh params;

	ctx->se_dev = se_devices[SE_RSA];

	ret = crypto_dh_decode_key(buf, len, &params);
	if (ret) {
		dev_err(ctx->se_dev->dev, "failed to decode DH input\n");
		return ret;
	}

	ret = tegra_se_dh_set_params(ctx, &params);
	if (ret)
		return ret;

	ret = tegra_se_dh_setkey(tfm);
	if (ret)
		return ret;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static unsigned int tegra_se_dh_max_size(struct crypto_kpp *tfm)
#else
static int tegra_se_dh_max_size(struct crypto_kpp *tfm)
#endif
{
	struct tegra_se_dh_context *ctx = tegra_se_dh_get_ctx(tfm);

	return ctx->p_size;
}

static void tegra_se_dh_exit_tfm(struct crypto_kpp *tfm)
{
	struct tegra_se_dh_context *ctx = tegra_se_dh_get_ctx(tfm);

	tegra_se_rsa_free_key_slot(ctx->slot);

	ctx->key = NULL;
	ctx->p = NULL;
	ctx->g = NULL;
}

static struct kpp_alg dh_algs[] = {
	{
	.set_secret = tegra_se_dh_set_secret,
	.generate_public_key = tegra_se_dh_compute_value,
	.compute_shared_secret = tegra_se_dh_compute_value,
	.max_size = tegra_se_dh_max_size,
	.exit = tegra_se_dh_exit_tfm,
	.base = {
		.cra_name = "dh",
		.cra_driver_name = "tegra-se-dh",
		.cra_priority = 300,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct tegra_se_dh_context),
		}
	}
};

static struct rng_alg rng_algs[] = {
	{
		.generate = tegra_se_rng_drbg_get_random,
		.seed = tegra_se_rng_drbg_reset,
		.seedsize = TEGRA_SE_RNG_SEED_SIZE,
		.base = {
			.cra_name = "rng_drbg",
			.cra_driver_name = "rng_drbg-aes-tegra",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_RNG,
			.cra_ctxsize = sizeof(struct tegra_se_rng_context),
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_rng_drbg_init,
			.cra_exit = tegra_se_rng_drbg_exit,
		}
	}
};

static struct crypto_alg aes_algs[] = {
	{
		.cra_name = "xts(aes)",
		.cra_driver_name = "xts-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_xts_encrypt,
			.decrypt = tegra_se_aes_xts_decrypt,
		}
	}, {
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_cbc_encrypt,
			.decrypt = tegra_se_aes_cbc_decrypt,
		}
	}, {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ecb_encrypt,
			.decrypt = tegra_se_aes_ecb_decrypt,
		}
	}, {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ctr_encrypt,
			.decrypt = tegra_se_aes_ctr_decrypt,
			.geniv = "eseqiv",
		}
	}, {
		.cra_name = "ofb(aes)",
		.cra_driver_name = "ofb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ofb_encrypt,
			.decrypt = tegra_se_aes_ofb_decrypt,
			.geniv = "eseqiv",
		}
	}
};

static struct ahash_alg hash_algs[] = {
	{
		.init = tegra_se_aes_cmac_init,
		.update = tegra_se_aes_cmac_update,
		.final = tegra_se_aes_cmac_final,
		.finup = tegra_se_aes_cmac_finup,
		.digest = tegra_se_aes_cmac_digest,
		.setkey = tegra_se_aes_cmac_setkey,
		.halg.digestsize = TEGRA_SE_AES_CMAC_DIGEST_SIZE,
		.halg.statesize = TEGRA_SE_AES_CMAC_STATE_SIZE,
		.halg.base = {
			.cra_name = "cmac(aes)",
			.cra_driver_name = "tegra-se-cmac(aes)",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= tegra_se_aes_cmac_cra_init,
			.cra_exit	= tegra_se_aes_cmac_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.export = tegra_se_sha_export,
		.import = tegra_se_sha_import,
		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.statesize = SHA1_STATE_SIZE,
		.halg.base = {
			.cra_name = "sha1",
			.cra_driver_name = "tegra-se-sha1",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.export = tegra_se_sha_export,
		.import = tegra_se_sha_import,
		.halg.digestsize = SHA224_DIGEST_SIZE,
		.halg.statesize = SHA224_STATE_SIZE,
		.halg.base = {
			.cra_name = "sha224",
			.cra_driver_name = "tegra-se-sha224",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA224_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.export = tegra_se_sha_export,
		.import = tegra_se_sha_import,
		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.statesize = SHA256_STATE_SIZE,
		.halg.base = {
			.cra_name = "sha256",
			.cra_driver_name = "tegra-se-sha256",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.export = tegra_se_sha_export,
		.import = tegra_se_sha_import,
		.halg.digestsize = SHA384_DIGEST_SIZE,
		.halg.statesize = SHA384_STATE_SIZE,
		.halg.base = {
			.cra_name = "sha384",
			.cra_driver_name = "tegra-se-sha384",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA384_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.export = tegra_se_sha_export,
		.import = tegra_se_sha_import,
		.halg.digestsize = SHA512_DIGEST_SIZE,
		.halg.statesize = SHA512_STATE_SIZE,
		.halg.base = {
			.cra_name = "sha512",
			.cra_driver_name = "tegra-se-sha512",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}
};

static struct akcipher_alg rsa_alg = {
	.encrypt = tegra_se_rsa_op,
	.decrypt = tegra_se_rsa_op,
	.sign = tegra_se_rsa_op,
	.verify = tegra_se_rsa_op,
	.set_priv_key = tegra_se_rsa_setkey,
	.set_pub_key = tegra_se_rsa_setkey,
	.max_size = tegra_se_rsa_max_size,
	.exit = tegra_se_rsa_exit,
	.base = {
		.cra_name = "rsa-pka0",
		.cra_driver_name = "tegra-se-pka0-rsa",
		.cra_priority = 300,
		.cra_ctxsize = sizeof(struct tegra_se_aes_rsa_context),
		.cra_module = THIS_MODULE,
	}
};

static int tegra_se_nvhost_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_se_dev *se_dev = pdata->private_data;

	if (se_dev->channel) {
		nvhost_putchannel(se_dev->channel, 1);
		se_dev->channel = NULL;

		/* syncpt will be released along with channel */
		se_dev->syncpt_id = 0;
	}

	return 0;
}

static struct tegra_se_chipdata tegra18_se_chipdata = {
	.aes_freq = 600000000,
	.cpu_freq_mhz = 2400,
};

static struct nvhost_device_data nvhost_se1_info = {
	.clocks = {{"se", 600000000},
		   {"emc", UINT_MAX,
		   NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		   0, TEGRA_BWMGR_SET_EMC_FLOOR}, {} },
	.can_powergate          = true,
	.autosuspend_delay      = 500,
	.class = NV_SE1_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
	.vm_regs		= {{SE_STREAMID_REG_OFFSET, true} },
	.kernel_only = true,
	.bwmgr_client_id = TEGRA_BWMGR_CLIENT_SE1,
	.prepare_poweroff = tegra_se_nvhost_prepare_poweroff,
};

static struct nvhost_device_data nvhost_se2_info = {
	.clocks = {{"se", 600000000},
		   {"emc", UINT_MAX,
		   NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		   0, TEGRA_BWMGR_SET_EMC_FLOOR}, {} },
	.can_powergate          = true,
	.autosuspend_delay      = 500,
	.class = NV_SE2_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
	.vm_regs		= {{SE_STREAMID_REG_OFFSET, true} },
	.kernel_only = true,
	.bwmgr_client_id = TEGRA_BWMGR_CLIENT_SE2,
	.prepare_poweroff = tegra_se_nvhost_prepare_poweroff,
};

static struct nvhost_device_data nvhost_se3_info = {
	.clocks = {{"se", 600000000},
		   {"emc", UINT_MAX,
		   NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		   0, TEGRA_BWMGR_SET_EMC_FLOOR}, {} },
	.can_powergate          = true,
	.autosuspend_delay      = 500,
	.class = NV_SE3_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
	.vm_regs		= {{SE_STREAMID_REG_OFFSET, true} },
	.kernel_only = true,
	.bwmgr_client_id = TEGRA_BWMGR_CLIENT_SE3,
	.prepare_poweroff = tegra_se_nvhost_prepare_poweroff,
};

static struct nvhost_device_data nvhost_se4_info = {
	.clocks = {{"se", 600000000},
		   {"emc", UINT_MAX,
		   NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER,
		   0, TEGRA_BWMGR_SET_EMC_FLOOR}, {} },
	.can_powergate          = true,
	.autosuspend_delay      = 500,
	.class = NV_SE4_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
	.vm_regs		= {{SE_STREAMID_REG_OFFSET, true} },
	.kernel_only = true,
	.bwmgr_client_id = TEGRA_BWMGR_CLIENT_SE4,
	.prepare_poweroff = tegra_se_nvhost_prepare_poweroff,
};

static const struct of_device_id tegra_se_of_match[] = {
	{
		.compatible = "nvidia,tegra186-se1-nvhost",
		.data = &nvhost_se1_info,
	}, {
		.compatible = "nvidia,tegra186-se2-nvhost",
		.data = &nvhost_se2_info,
	}, {
		.compatible = "nvidia,tegra186-se3-nvhost",
		.data = &nvhost_se3_info,
	}, {
		.compatible = "nvidia,tegra186-se4-nvhost",
		.data = &nvhost_se4_info,
	}, {}
};
MODULE_DEVICE_TABLE(of, tegra_se_of_match);

static bool is_algo_supported(struct device_node *node, char *algo)
{
	if (of_property_match_string(node, "supported-algos", algo) >= 0)
		return true;
	else
		return false;
}

static void tegra_se_fill_se_dev_info(struct tegra_se_dev *se_dev)
{
	struct device_node *node = of_node_get(se_dev->dev->of_node);

	if (is_algo_supported(node, "aes"))
		se_devices[SE_AES] = se_dev;
	if (is_algo_supported(node, "drbg"))
		se_devices[SE_DRBG] = se_dev;
	if (is_algo_supported(node, "sha"))
		se_devices[SE_SHA] = se_dev;
	if (is_algo_supported(node, "rsa"))
		se_devices[SE_RSA] = se_dev;
	if (is_algo_supported(node, "cmac"))
		se_devices[SE_CMAC] = se_dev;
}

static int tegra_se_probe(struct platform_device *pdev)
{
	struct tegra_se_dev *se_dev = NULL;
	struct nvhost_device_data *pdata = NULL;
	const struct of_device_id *match;
	int err = 0, i = 0;
	unsigned int val;
	struct device_node *node = NULL;
	const char *rsa_name;

	se_dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_se_dev),
			      GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		match = of_match_device(of_match_ptr(tegra_se_of_match),
					&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Error: No device match found\n");
			return -ENODEV;
		}
		pdata = (struct nvhost_device_data *)match->data;
	} else {
		pdata =
		(struct nvhost_device_data *)pdev->id_entry->driver_data;
	}

	mutex_init(&se_dev->lock);
	crypto_init_queue(&se_dev->queue, TEGRA_SE_CRYPTO_QUEUE_LENGTH);

	se_dev->dev = &pdev->dev;
	se_dev->pdev = pdev;

	mutex_init(&pdata->lock);
	pdata->pdev = pdev;

	/* store chipdata inside se_dev and store se_dev into private_data */
	se_dev->chipdata = pdata->private_data;
	pdata->private_data = se_dev;

	/* store the pdata into drvdata */
	platform_set_drvdata(pdev, pdata);

	err = nvhost_client_device_get_resources(pdev);
	if (err) {
		dev_err(se_dev->dev,
			"nvhost_client_device_get_resources failed for SE(%s)\n",
			pdev->name);
		return err;
	}

	err = nvhost_module_init(pdev);
	if (err) {
		dev_err(se_dev->dev,
			"nvhost_module_init failed for SE(%s)\n", pdev->name);
		return err;
	}

	err = nvhost_client_device_init(pdev);
	if (err) {
		dev_err(se_dev->dev,
			"nvhost_client_device_init failed for SE(%s)\n",
			pdev->name);
		return err;
	}

	err = nvhost_channel_map(pdata, &se_dev->channel, pdata);
	if (err) {
		dev_err(se_dev->dev, "Nvhost Channel map failed\n");
		return err;
	}

	se_dev->io_regs = pdata->aperture[0];

	node = of_node_get(se_dev->dev->of_node);

	se_dev->ioc = of_property_read_bool(node, "nvidia,io-coherent");

	err = of_property_read_u32(node, "opcode_addr", &se_dev->opcode_addr);
	if (err) {
		dev_err(se_dev->dev, "Missing opcode_addr property\n");
		return err;
	}

	if (!of_property_count_strings(node, "supported-algos"))
		return -ENOTSUPP;

	tegra_se_fill_se_dev_info(se_dev);

	if (is_algo_supported(node, "aes") || is_algo_supported(node, "drbg")) {
		err = tegra_init_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_key_slot failed\n");
			return err;
		}
	}

	if (is_algo_supported(node, "rsa")) {
		err = tegra_init_rsa_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_rsa_key_slot failed\n");
			return err;
		}
	}

	mutex_init(&se_dev->mtx);
	INIT_WORK(&se_dev->se_work, tegra_se_work_handler);
	se_dev->se_work_q = alloc_workqueue("se_work_q",
					    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!se_dev->se_work_q) {
		dev_err(se_dev->dev, "alloc_workqueue failed\n");
		return -ENOMEM;
	}

	err = tegra_se_alloc_ll_buf(se_dev, SE_MAX_SRC_SG_COUNT,
				    SE_MAX_DST_SG_COUNT);
	if (err) {
		dev_err(se_dev->dev, "can not allocate ll dma buffer\n");
		goto ll_alloc_fail;
	}

	if (is_algo_supported(node, "drbg")) {
		INIT_LIST_HEAD(&rng_algs[0].base.cra_list);
		err = crypto_register_rng(&rng_algs[0]);
		if (err) {
			dev_err(se_dev->dev, "crypto_register_rng failed\n");
			goto reg_fail;
		}
	}

	if (is_algo_supported(node, "xts")) {
		INIT_LIST_HEAD(&aes_algs[0].cra_list);
		err = crypto_register_alg(&aes_algs[0]);
		if (err) {
			dev_err(se_dev->dev,
				"crypto_register_alg xts failed\n");
			goto reg_fail;
		}
	}
	if (is_algo_supported(node, "aes")) {

		for (i = 1; i < ARRAY_SIZE(aes_algs); i++) {
			INIT_LIST_HEAD(&aes_algs[i].cra_list);
			err = crypto_register_alg(&aes_algs[i]);
			if (err) {
				dev_err(se_dev->dev,
					"crypto_register_alg %s failed\n",
					aes_algs[i].cra_name);
				goto reg_fail;
			}
		}
	}

	if (is_algo_supported(node, "cmac")) {
		err = crypto_register_ahash(&hash_algs[0]);
		if (err) {
			dev_err(se_dev->dev,
				"crypto_register_ahash cmac failed\n");
			goto reg_fail;
		}
	}

	if (is_algo_supported(node, "sha")) {
		for (i = 1; i < 6; i++) {
			err = crypto_register_ahash(&hash_algs[i]);
			if (err) {
				dev_err(se_dev->dev,
					"crypto_register_ahash %s failed\n",
					hash_algs[i].halg.base.cra_name);
				goto reg_fail;
			}
		}
	}

	node = of_node_get(se_dev->dev->of_node);

	err = of_property_read_u32(node, "pka0-rsa-priority", &val);
	if (!err)
		rsa_alg.base.cra_priority = val;

	err = of_property_read_string(node, "pka0-rsa-name", &rsa_name);
	if (!err)
		strncpy(rsa_alg.base.cra_name, rsa_name,
				sizeof(rsa_alg.base.cra_name) - 1);

	if (is_algo_supported(node, "rsa")) {
		err = crypto_register_akcipher(&rsa_alg);
		if (err) {
			dev_err(se_dev->dev, "crypto_register_akcipher fail");
			goto reg_fail;
		}
		err = crypto_register_kpp(&dh_algs[0]);
		if (err) {
			dev_err(se_dev->dev, "crypto_register_kpp fail");
			goto reg_fail;
		}

		se_dev->dh_buf1 = (u32 *)devm_kzalloc(
				se_dev->dev, TEGRA_SE_RSA2048_INPUT_SIZE,
				GFP_KERNEL);
		se_dev->dh_buf2 = (u32 *)devm_kzalloc(
				se_dev->dev, TEGRA_SE_RSA2048_INPUT_SIZE,
				GFP_KERNEL);
		if (!se_dev->dh_buf1 || !se_dev->dh_buf2)
			goto reg_fail;
	}

	if (is_algo_supported(node, "drbg")) {
		/* Make sure engine is powered ON with clk enabled */
		err = nvhost_module_busy(pdev);
		if (err) {
			dev_err(se_dev->dev,
				"nvhost_module_busy failed for se_dev\n");
			goto reg_fail;
		}
		se_writel(se_dev,
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC(DRBG_RO_ENT_SRC_ENABLE) |
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC_LOCK(
						DRBG_RO_ENT_SRC_LOCK_ENABLE),
			  SE_RNG_SRC_CONFIG_REG_OFFSET);
		/* Power OFF after SE register update */
		nvhost_module_idle(pdev);
	}

	se_dev->syncpt_id = nvhost_get_syncpt_host_managed(se_dev->pdev,
							   0, pdev->name);
	if (!se_dev->syncpt_id) {
		err = -EINVAL;
		dev_err(se_dev->dev, "Cannot get syncpt_id for SE(%s)\n",
			pdev->name);
		goto reg_fail;
	}

	se_dev->aes_src_ll = devm_kzalloc(&pdev->dev, sizeof(struct tegra_se_ll),
				      GFP_KERNEL);
	se_dev->aes_dst_ll = devm_kzalloc(&pdev->dev, sizeof(struct tegra_se_ll),
				      GFP_KERNEL);
	if (!se_dev->aes_src_ll || !se_dev->aes_dst_ll) {
		dev_err(se_dev->dev, "Linked list memory allocation failed\n");
		goto aes_buf_alloc_fail;
	}

	if (se_dev->ioc)
		se_dev->total_aes_buf = dma_alloc_coherent(
						se_dev->dev, SE_MAX_MEM_ALLOC,
						&se_dev->total_aes_buf_addr,
						GFP_KERNEL);
	else
		se_dev->total_aes_buf = kzalloc(SE_MAX_MEM_ALLOC, GFP_KERNEL);

	if (!se_dev->total_aes_buf) {
		err = -ENOMEM;
		goto aes_buf_alloc_fail;
	}

	tegra_se_init_aesbuf(se_dev);

	if (is_algo_supported(node, "drbg") || is_algo_supported(node, "aes") ||
	    is_algo_supported(node, "cmac")) {
		se_dev->aes_cmdbuf_cpuvaddr = dma_alloc_attrs(
			se_dev->dev->parent, SZ_16K * SE_MAX_SUBMIT_CHAIN_SZ,
			&se_dev->aes_cmdbuf_iova, GFP_KERNEL,
			__DMA_ATTR(attrs));
		if (!se_dev->aes_cmdbuf_cpuvaddr)
			goto cmd_buf_alloc_fail;

		err = tegra_se_init_cmdbuf_addr(se_dev);
		if (err) {
			dev_err(se_dev->dev, "failed to init cmdbuf addr\n");
			goto dma_free;
		}
	}

	tegra_se_boost_cpu_init(se_dev);

	dev_info(se_dev->dev, "%s: complete", __func__);

	return 0;
dma_free:
	dma_free_attrs(se_dev->dev->parent, SZ_16K * SE_MAX_SUBMIT_CHAIN_SZ,
		       se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova,
		       __DMA_ATTR(attrs));
cmd_buf_alloc_fail:
	kfree(se_dev->total_aes_buf);
aes_buf_alloc_fail:
	nvhost_syncpt_put_ref_ext(se_dev->pdev, se_dev->syncpt_id);
reg_fail:
	tegra_se_free_ll_buf(se_dev);
ll_alloc_fail:
	if (se_dev->se_work_q)
		destroy_workqueue(se_dev->se_work_q);

	return err;
}

static int tegra_se_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_se_dev *se_dev = pdata->private_data;
	struct device_node *node;
	int i;

	if (!se_dev) {
		pr_err("Device is NULL\n");
		return -ENODEV;
	}

	tegra_se_boost_cpu_deinit(se_dev);

	if (se_dev->aes_cmdbuf_cpuvaddr)
		dma_free_attrs(
		se_dev->dev->parent, SZ_16K * SE_MAX_SUBMIT_CHAIN_SZ,
		se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova,
		__DMA_ATTR(attrs));

	node = of_node_get(se_dev->dev->of_node);
	if (is_algo_supported(node, "drbg"))
		crypto_unregister_rng(&rng_algs[0]);

	if (is_algo_supported(node, "xts"))
		crypto_unregister_alg(&aes_algs[0]);

	if (is_algo_supported(node, "aes")) {
		crypto_unregister_alg(&aes_algs[1]);
		for (i = 2; i < ARRAY_SIZE(aes_algs); i++)
			crypto_unregister_alg(&aes_algs[i]);
	}

	if (is_algo_supported(node, "cmac"))
		crypto_unregister_ahash(&hash_algs[0]);

	if (is_algo_supported(node, "sha")) {
		for (i = 1; i < 6; i++)
			crypto_unregister_ahash(&hash_algs[i]);
	}

	if (is_algo_supported(node, "rsa")) {
		crypto_unregister_akcipher(&rsa_alg);
		crypto_unregister_kpp(&dh_algs[0]);
	}

	tegra_se_free_ll_buf(se_dev);
	kfree(se_dev->total_aes_buf);

	cancel_work_sync(&se_dev->se_work);
	if (se_dev->se_work_q)
		destroy_workqueue(se_dev->se_work_q);

	mutex_destroy(&se_dev->mtx);
	nvhost_client_device_release(pdev);
	mutex_destroy(&pdata->lock);

	return 0;
}

static struct platform_driver tegra_se_driver = {
	.probe  = tegra_se_probe,
	.remove = tegra_se_remove,
	.driver = {
		.name   = "tegra-se-nvhost",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_of_match),
		.pm = &nvhost_module_pm_ops,
		.suppress_bind_attrs = true,
	},
};

static int __init tegra_se_module_init(void)
{
	return  platform_driver_register(&tegra_se_driver);
}

static void __exit tegra_se_module_exit(void)
{
	platform_driver_unregister(&tegra_se_driver);
}

module_init(tegra_se_module_init);
module_exit(tegra_se_module_exit);

MODULE_DESCRIPTION("Tegra Crypto algorithm support using Host1x Interface");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se-nvhost");
