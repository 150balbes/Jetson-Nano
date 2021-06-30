/*
 * RAS driver for T194
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/arm64_ras.h>
#include <linux/of_irq.h>
#include <linux/cpuhotplug.h>

static int fhi_irq[CONFIG_NR_CPUS];
static u8 is_ras_probe_done;
static LIST_HEAD(fhi_callback_list);
static DEFINE_RAW_SPINLOCK(fhi_lock);

/* saved hotplug state */
static enum cpuhp_state hp_state;

static const struct of_device_id arm64_ras_match[] = {
	{
		.name = "arm64_ras",
		.compatible = "arm,armv8.2-ras",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, arm64_ras_match);

/* Architecturally defined primary error code
 * Used for decoding ERR<x>STATUS:SERR bits
 */
static struct ras_error serr_errors[] = {
	{.name = "No error", .error_code = 0x0},
	{.name = "Implementation defined error", .error_code = 0x1},
	{.name = "Data value from non-associative internal memory",
	 .error_code = 0x2},
	{.name = "Implementation defined pin", .error_code = 0x3},
	{.name = "Assertion Failure", .error_code = 0x4},
	{.name = "Internal Data Path", .error_code = 0x5},
	{.name = "Data value from associative memory", .error_code = 0x6},
	{.name = "Address/control value(s) from associative memory",
	 .error_code = 0x7},
	{.name = "Data value from a TLB", .error_code = 0x8},
	{.name = "Address/control value(s) from a TLB", .error_code = 0x9},
	{.name = "Data value from producer", .error_code = 0xA},
	{.name = "Address/control value(s) from producer", .error_code = 0xB},
	{.name = "Data value from (non-associative) external memory",
	 .error_code = 0xC},
	{.name = "Illegal address (software fault)", .error_code = 0xD},
	{.name = "Illegal access (software fault)", .error_code = 0xE},
	{.name = "Illegal state (software fault)", .error_code = 0xF},
	{.name = "Internal data register", .error_code = 0x10},
	{.name = "Internal control register", .error_code = 0x11},
	{.name = "Error response from slave", .error_code = 0x12},
	{.name = "External timeout", .error_code = 0x13},
	{.name = "Internal timeout", .error_code = 0x14},
	{.name = "Deferred error from slave not supported at master",
	 .error_code = 0x15},
};

/* Read ERR<X>SELR */
u64 ras_read_errselr(void)
{
	u64 errselr;

	asm volatile("mrs %0, s3_0_c5_c3_1" : "=r" (errselr));
	return errselr;
}
EXPORT_SYMBOL(ras_read_errselr);

/* For read/write of any ERR<X> register, error record X is selected
 * by writing to ERRSELR_EL1
 * ISB is needed in the end to ensure this write completes before another
 * read/write to this error record comes in.
 */
void ras_write_errselr(u64 errx)
{
	asm volatile("msr s3_0_c5_c3_1, %0" : : "r" (errx));
	isb();
}
EXPORT_SYMBOL(ras_write_errselr);

/* Read ERR<X>STATUS */
u64 ras_read_error_status(void)
{
	u64 status;

	asm volatile("mrs %0, s3_0_c5_c4_2" : "=r" (status));
	return status;
}
EXPORT_SYMBOL(ras_read_error_status);

/* Write to  ERR<X>STATUS */
void ras_write_error_status(u64 status)
{
	asm volatile("msr s3_0_c5_c4_2, %0" : : "r" (status));
}
EXPORT_SYMBOL(ras_write_error_status);

/* Read ERR<X>ADDR */
static u64 ras_read_error_address(void)
{
	u64 addr;

	asm volatile("mrs %0, s3_0_c5_c4_3" : "=r" (addr));
	return addr;
}

/* Write to ERR<X>ADDR*/
void ras_write_error_addr(u64 addr)
{
	asm volatile("msr s3_0_c5_c4_3, %0" : : "r" (addr));
}
EXPORT_SYMBOL(ras_write_error_addr);

/* Read ERR<X>MISC0 */
static u64 ras_read_error_misc0(void)
{
	u64 misc0;

	asm volatile("mrs %0, s3_0_c5_c5_0" : "=r" (misc0));
	return misc0;
}

/* Write to ERR<X>MISC0*/
void ras_write_error_misc0(u64 misc0)
{
	asm volatile("msr s3_0_c5_c5_0, %0" : : "r" (misc0));
}
EXPORT_SYMBOL(ras_write_error_misc0);

/* Read ERR<X>MISC1 */
static u64 ras_read_error_misc1(void)
{
	u64 misc1;

	asm volatile("mrs %0, s3_0_c5_c5_1" : "=r" (misc1));
	return misc1;
}

/* Write to ERR<X>MISC1*/
void ras_write_error_misc1(u64 misc1)
{
	asm volatile("msr s3_0_c5_c5_1, %0" : : "r" (misc1));
}
EXPORT_SYMBOL(ras_write_error_misc1);

/* Read ERR<X>CTLR_EL1*/
u64 ras_read_error_control(void)
{
	u64 err_ctl;

	asm volatile("mrs %0, s3_0_c5_c4_1" : "=r" (err_ctl));
	return err_ctl;
}
EXPORT_SYMBOL(ras_read_error_control);

/* Write to ERR<X>CTLR_EL1*/
void ras_write_error_control(u64 err_ctl)
{
	asm volatile("msr s3_0_c5_c4_1, %0" : : "r" (err_ctl));
}
EXPORT_SYMBOL(ras_write_error_control);

/* Read ERR<X>PFGCTL_EL1*/
u64 ras_read_pfg_control(void)
{
	u64 pfg_ctl;

	asm volatile("mrs %0, s3_0_c15_c1_4" : "=r" (pfg_ctl));
	return pfg_ctl;
}
EXPORT_SYMBOL(ras_read_pfg_control);

/* Write to ERR<X>PFGCTL_EL1*/
void ras_write_pfg_control(u64 pfg_ctl)
{
	asm volatile("msr s3_0_c15_c1_4, %0" : : "r" (pfg_ctl));
}
EXPORT_SYMBOL(ras_write_pfg_control);

/* Read ERR<X>PFGCDN_EL1*/
u64 ras_read_pfg_cdn(void)
{
	u64 pfg_cdn;

	asm volatile("mrs %0, s3_0_c15_c1_6" : "=r" (pfg_cdn));
	return pfg_cdn;
}
EXPORT_SYMBOL(ras_read_pfg_cdn);

/* Write to ERR<X>PFGCDN_EL1*/
void ras_write_pfg_cdn(u64 pfg_cdn)
{
	asm volatile("msr s3_0_c15_c1_6, %0" : : "r" (pfg_cdn));
}
EXPORT_SYMBOL(ras_write_pfg_cdn);

void unregister_fhi_callback(struct ras_fhi_callback *callback)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fhi_lock, flags);
	list_del(&callback->node);
	raw_spin_unlock_irqrestore(&fhi_lock, flags);
}
EXPORT_SYMBOL(unregister_fhi_callback);

static unsigned int read_pfr0_ras_version(void)
{
	return PFR0_RAS(read_cpuid(ID_AA64PFR0_EL1));
}

static void is_ras_version1(void *ret)
{
	*(int *)(ret)  = (read_pfr0_ras_version() == PFR0_RAS_VERSION_1);
}

/* check if RAS is supported on this CPU
 *     ret = 0 if not supported
 *         = 1 if supported
 */
int is_ras_cpu(int cpu)
{
	int ret;

	smp_call_function_single(cpu, is_ras_version1, &ret, 1);
	return ret;
}
EXPORT_SYMBOL(is_ras_cpu);

int is_this_ras_cpu(void)
{
	return read_pfr0_ras_version() == PFR0_RAS_VERSION_1;
}
EXPORT_SYMBOL(is_this_ras_cpu);

int is_ras_ready(void)
{
	return is_ras_probe_done == 1;
}
EXPORT_SYMBOL(is_ras_ready);

void print_error_record(struct error_record *record, u64 status, int errselr)
{
	struct ras_error *errors;
	u64 misc0, misc1, addr;
	u16 ierr, serr;
	u64 i;
	int found = 0;
	u64 err_status = 0;

	pr_crit("**************************************\n");
	pr_crit("RAS Error in %s, ERRSELR_EL1=%d:\n", record->name, errselr);
	pr_crit("\tStatus = 0x%llx\n", status);

	/* Find the name of IERR */
	ierr = get_error_status_ierr(status);
	errors = record->errors;
	if (errors) {
		for (i = 0; errors[i].name; i++) {
			if (errors[i].error_code  == ierr) {
				pr_crit("\tIERR = %s: 0x%x\n",
					errors[i].name, ierr);
				found = 1;
				break;
			}
		}
		if (!found)
			pr_crit("\tUnknown IERR: 0x%x\n", ierr);
	} else {
		pr_crit("\tBank does not have any known IERR errors\n");
	}

	/* Find the name of SERR */
	found = 0;
	serr = get_error_status_serr(status);

	for (i = 0; serr_errors[i].name; i++) {
		if (serr_errors[i].error_code == serr) {
			pr_crit("\tSERR = %s: 0x%x\n", serr_errors[i].name,
				serr);
				found = 1;
				break;
		}
	}
	if (!found)
		pr_crit("\tUnknown SERR: 0x%x\n", serr);

	if (status & ERRi_STATUS_OF) {
		pr_crit("\tOverflow (there may be more errors) - Uncorrectable\n");
		/* Clear the error by writing 1 to ERRi_STATUS:OF */
		err_status |= ERRi_STATUS_OF;
	}
	if (status & ERRi_STATUS_UE) {
		pr_crit("\tUncorrectable (this is fatal)\n");
		/* Clear the error by writing 1 to ERR_STATUS:UE
		 * and 11 to ERR_STATUS:UET
		 */
		err_status |= ERRi_STATUS_UE;
		err_status |= ERRi_STATUS_UET;
	}
	if (get_error_status_ce(status)) {
		pr_crit("\tCorrectable Error\n");
		/* Clear the error by writing 1 to ERR_STATUS:CE */
		err_status |= ERRi_STATUS_CE;
	}
	if (status & ERRi_STATUS_MV) {
		misc0 = ras_read_error_misc0();
		misc1 = ras_read_error_misc1();
		pr_crit("\tMISC0 = 0x%llx\n", misc0);
		pr_crit("\tMISC1 = 0x%llx\n", misc1);
		/* write 1 to clear ERR_STATUS:MV */
		err_status |= ERRi_STATUS_MV;
	}
	if (status & ERRi_STATUS_AV) {
		addr = ras_read_error_address();
		pr_crit("\tADDR = 0x%llx\n", addr);
		/* write 1 to clear ERR_STATUS:AV */
		err_status |= ERRi_STATUS_AV;
	}

	/* Write 1 to clear ERR_STATUS:V */
	err_status |= ERRi_STATUS_VALID;

	/* Write to ERR_STATUS to clear the error */
	ras_write_error_status(err_status);
	pr_crit("**************************************\n");
}
EXPORT_SYMBOL(print_error_record);

/* Fault Handling Interrupt or FHI
 * for handling Correctable Errors
 */
static irqreturn_t ras_fhi_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct ras_fhi_callback *callback;

	/* Iterate through the banks looking for one with an error */
	pr_crit("CPU%d: RAS: FHI %d detected\n", smp_processor_id(), irq);

	raw_spin_lock_irqsave(&fhi_lock, flags);
	list_for_each_entry(callback, &fhi_callback_list, node) {
		callback->fn();
	}
	raw_spin_unlock_irqrestore(&fhi_lock, flags);

	return IRQ_HANDLED;
}

static int ras_fhi_enable(unsigned int cpu)
{
	if (irq_force_affinity(fhi_irq[cpu], cpumask_of(cpu))) {
		pr_info("%s: Failed to set IRQ %d affinity to CPU%d\n",
			__func__, fhi_irq[cpu], cpu);
		return -EINVAL;
	}
	enable_irq(fhi_irq[cpu]);
	pr_info("%s: FHI %d enabled on CPU%d\n", __func__, fhi_irq[cpu], cpu);
	return 0;
}

static int ras_fhi_disable(unsigned int cpu)
{
	int boot_cpu = 0;

	disable_irq_nosync(fhi_irq[cpu]);
	if (irq_force_affinity(fhi_irq[cpu], cpumask_of(boot_cpu)))
		pr_info("%s: Failed to set IRQ %d affinity to boot cpu %d\n",
					__func__, fhi_irq[cpu], boot_cpu);
	pr_info("%s: FHI %d disabled\n", __func__, fhi_irq[cpu]);
	return 0;
}

/* Register Fault Handling Interrupt or FHI
 * for Correctable Errors
 */
static int ras_register_fhi_isr(struct platform_device *pdev)
{
	int err = 0, cpu, irq_count;

	irq_count = platform_irq_count(pdev);

	for_each_possible_cpu(cpu) {
		BUG_ON(cpu >= irq_count);
		fhi_irq[cpu] = platform_get_irq(pdev, cpu);
		if (fhi_irq[cpu] <= 0) {
			dev_err(&pdev->dev, "No IRQ\n");
			err = -ENOENT;
			goto isr_err;
		}
	}

isr_err:
	return err;
}

/* This is an API for CPU specific FHI callbacks
 * to be registered with fhi_isr handler
 */
int register_fhi_callback(struct ras_fhi_callback *callback, void *cookie)
{
	unsigned long flags;
	int err = 0, cpu = 0;

	raw_spin_lock_irqsave(&fhi_lock, flags);
	list_add(&callback->node, &fhi_callback_list);
	raw_spin_unlock_irqrestore(&fhi_lock, flags);

	for_each_possible_cpu(cpu) {
		err = request_irq(fhi_irq[cpu], ras_fhi_isr,
				IRQ_PER_CPU, "ras-fhi", cookie);
		if (err) {
			pr_err("%s: request_irq(%d) failed (%d)\n", __func__,
					fhi_irq[cpu], err);
			goto isr_err;
		}
		disable_irq(fhi_irq[cpu]);
	}
	/* Ensure that any CPU brought online sets up FHI */
	hp_state = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
			"arm64_ras:online",
			ras_fhi_enable,
			ras_fhi_disable);

	if (hp_state < 0)
		pr_info("%s: unable to register FHI\n", __func__);

isr_err:
	return err;
}
EXPORT_SYMBOL(register_fhi_callback);

static int ras_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int err = 0, cpu, probe = 0;
	struct device *dev = &pdev->dev;

	/* probe only if RAS is supported on either of the online CPUs */
	for_each_online_cpu(cpu) {
		if (is_ras_cpu(cpu))
			probe = 1;
	}

	if (!probe) {
		dev_info(dev, "None of the CPUs support RAS");
		return -EINVAL;
	}

	/* register FHI for Correctable Errors */
	match = of_match_device(of_match_ptr(arm64_ras_match),
				dev);
	if (!match) {
		dev_err(dev, "RAS detected, but no device-tree node found,"
			" Cannot register RAS fault handler interrupt (FHI)");
		return -ENODEV;
	}
	err = ras_register_fhi_isr(pdev);
	if (err < 0) {
		dev_info(dev, "Failed to register Fault Handling Interrupts"
				" ISR");
		return err;
	}

	/* make sure we have executed everything in the probe
	 * before setting is_ras_probe_done
	 */
	smp_mb();
	is_ras_probe_done = 1;
	dev_info(dev, "probed");
	return 0;
}

static int ras_remove(struct platform_device *pdev)
{
	cpuhp_remove_state(hp_state);
	return 0;
}

static struct platform_driver ras_driver = {
	.probe = ras_probe,
	.remove = ras_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "arm64_ras",
		.of_match_table = of_match_ptr(arm64_ras_match),
	},
};

static int __init ras_init(void)
{
	return platform_driver_register(&ras_driver);
}

static void __exit ras_exit(void)
{
	platform_driver_unregister(&ras_driver);
}

arch_initcall(ras_init);
module_exit(ras_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM64 RAS Driver / FHI handler");

