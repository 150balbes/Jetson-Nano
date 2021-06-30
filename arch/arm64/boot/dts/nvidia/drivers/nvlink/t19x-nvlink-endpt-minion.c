/*
 * t19x-nvlink-endpt-minion.c:
 * This file contains code for booting and interacting with the MINION
 * microcontroller located inside the Tegra NVLINK controller.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/firmware.h>
#include <linux/clk.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define MINION_FW_PATH			"nvlink/t194_minion_ucode.bin"
#define MINION_BYTES_PER_BLOCK		256
#define MINION_WORD_SIZE		4

/* Extract a WORD from the MINION ucode */
static inline u32 minion_extract_word(struct tnvlink_dev *tdev, int idx)
{
	struct nvlink_device *ndev = tdev->ndev;
	u32 out_data = 0;
	u8 byte = 0;
	int i = 0;

	for (i = 0; i < 4; i++) {
		byte = ndev->minion_fw->data[idx + i];
		out_data |= ((u32)byte) << (8 * i);
	}

	return out_data;
}

/* Helper function for writing 1 word of data to the MINION's IMEM */
static inline void minion_write_imem(struct tnvlink_dev *tdev,
					u32 byte_offs,
					u32 word,
					u32 tag)
{
	/* Need to write tag at the start of each new 256B block */
	if ((byte_offs % MINION_BYTES_PER_BLOCK) < 4)
		nvlw_minion_writel(tdev, CMINION_FALCON_IMEMT, tag);

	nvlw_minion_writel(tdev, CMINION_FALCON_IMEMD, word);
}

/* Helper function for writing 1 word of data to the MINION's DMEM */
static inline void minion_write_dmem(struct tnvlink_dev *tdev, u32 word)
{
	nvlw_minion_writel(tdev, CMINION_FALCON_DMEMD, word);
}

/* Load a section of the MINION ucode into IMEM or DMEM */
static int minion_load_ucode_section(struct tnvlink_dev *tdev,
					int is_imem,
					u32 offset,
					u32 size,
					int use_app_tag,
					u32 app_tag)
{
	struct nvlink_device *ndev = tdev->ndev;
	u32 i = offset;
	u32 byte_pos = 0;
	u8 byte = 0;
	u32 word = 0;
	u32 tag = 0;

	if ((offset + size) > ndev->minion_hdr.ucode_data_size) {
		nvlink_err("Section size is invalid");
		return -1;
	}

	/* Extract bytes from ucode image and write them to IMEM or DMEM */
	for (i = offset; i < (offset + size); i++) {
		byte_pos = i % 4;
		byte = ndev->minion_img[i];

		/* Increment app tag at the start of each new 256B block */
		if (use_app_tag &&
		    (i != offset) &&
		    ((i % MINION_BYTES_PER_BLOCK) == 0)) {
			app_tag++;
		}

		/* Last byte */
		if (i == ndev->minion_hdr.ucode_data_size - 1) {
			if (byte_pos != 0) {
				if (is_imem) {
					if (use_app_tag)
						tag = app_tag;
					else
						tag = i/MINION_BYTES_PER_BLOCK;

					minion_write_imem(tdev, i, word, tag);
				} else {
					minion_write_dmem(tdev, word);
				}
			}
		} else {
			if (byte_pos == 0)
				word = 0;
			word |= ((u32)byte) << (8 * byte_pos);

			/* Write word to IMEM or DMEM */
			if (byte_pos == 3) {
				if (is_imem) {
					if (use_app_tag)
						tag = app_tag;
					else
						tag = i/MINION_BYTES_PER_BLOCK;

					minion_write_imem(tdev, i, word, tag);
				} else {
					minion_write_dmem(tdev, word);
				}
			}
		}
	}

	return 0;
}

/* Send a command to the MINION and wait for command completion */
int minion_send_cmd(struct tnvlink_dev *tdev,
				u32 cmd,
				u32 scratch0_val)
{
	int err = 0;
	u32 reg_val = 0;

	/* Write to minion scratch if needed by command */
	if (cmd == MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM)
		nvlw_minion_writel(tdev, MINION_MISC, scratch0_val);

	/* Send command to MINION */
	reg_val = (cmd << MINION_NVLINK_DL_CMD_COMMAND_SHIFT) &
			MINION_NVLINK_DL_CMD_COMMAND_MASK;
	reg_val |= BIT(MINION_NVLINK_DL_CMD_FAULT);
	nvlw_minion_writel(tdev, MINION_NVLINK_DL_CMD, reg_val);

	/* Wait for MINION_NVLINK_DL_CMD_READY bit to be set */
	err = wait_for_reg_cond_nvlink(tdev,
					MINION_NVLINK_DL_CMD,
					MINION_NVLINK_DL_CMD_READY,
					true,
					"MINION_NVLINK_DL_CMD_READY",
					nvlw_minion_readl,
					&reg_val,
					DEFAULT_LOOP_TIMEOUT_US);
	if (err < 0) {
		nvlink_err("MINION command (cmd = %d) failed to complete", cmd);
		return err;
	}

	if (reg_val & BIT(MINION_NVLINK_DL_CMD_FAULT)) {
		nvlink_err("MINION command (cmd = %d) faulted!", cmd);

		/* Clear the fault and return */
		nvlw_minion_writel(tdev, MINION_NVLINK_DL_CMD, reg_val);
		return -1;
	}

	nvlink_dbg("MINION command (cmd = %d) completed successfully!", cmd);
	return 0;
}

/*
 * minion_print_ucode: Dump the contents of the MINION's IMEM and DMEM
 *
 * TODO: Currently we're making assumptions about the ordering of the IMEM/DMEM
 *       sections. We need to do a run-time sort on the starting offsets of all
 *       the IMEM/DMEM sections and then dump out the contents of each section
 *       in the derived order.
 */
static void minion_print_ucode(struct tnvlink_dev *tdev)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct minion_hdr *hdr = &(ndev->minion_hdr);
	int i = 0;
	int j = 0;
	u32 reg_val = 0;
	u32 byte_num = 0;

	/* Dump the IMEM's contents */
	nvlink_dbg("");
	nvlink_dbg("MINION IMEM DUMP - START");

	/* Initialize address of IMEM to 0x0 and set auto-increment on read */
	nvlw_minion_writel(tdev,
			CMINION_FALCON_IMEMC,
			BIT(CMINION_FALCON_IMEMC_AINCR));

	/* Dump the OS code section of the IMEM */
	nvlink_dbg("");
	nvlink_dbg("OS Code Section:");
	for (i = 0; i < hdr->os_code_size/MINION_WORD_SIZE; i++) {
		reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_IMEMD);
		nvlink_dbg("Byte 0x%x, Data = 0x%x", byte_num, reg_val);
		byte_num += MINION_WORD_SIZE;
	}

	/* Dump the app code sections of the IMEM */
	for (i = 0; i < hdr->num_apps; i++) {
		nvlink_dbg("");
		nvlink_dbg("App %d Code Section:", i);
		for (j = 0; j < hdr->app_code_sizes[i]/MINION_WORD_SIZE; j++) {
			reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_IMEMD);
			nvlink_dbg("Byte 0x%x, Data = 0x%x", byte_num, reg_val);
			byte_num += MINION_WORD_SIZE;
		}
	}
	nvlink_dbg("");
	nvlink_dbg("MINION IMEM DUMP - END");

	/* Dump the DMEM's contents */
	nvlink_dbg("");
	nvlink_dbg("MINION DMEM DUMP - START");

	/* Initialize address of DMEM to 0x0 and set auto-increment on read */
	nvlw_minion_writel(tdev,
			CMINION_FALCON_DMEMC,
			BIT(CMINION_FALCON_DMEMC_AINCR));

	/* Dump the OS data section of the DMEM */
	nvlink_dbg("");
	nvlink_dbg("OS Data Section:");
	byte_num = 0;
	for (i = 0; i < hdr->os_data_size/MINION_WORD_SIZE; i++) {
		reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_DMEMD);
		nvlink_dbg("Byte 0x%x, Data = 0x%x", byte_num, reg_val);
		byte_num += MINION_WORD_SIZE;
	}

	/* Dump the app data sections of the DMEM */
	for (i = 0; i < hdr->num_apps; i++) {
		nvlink_dbg("");
		nvlink_dbg("App %d Data Section:", i);
		for (j = 0; j < hdr->app_data_sizes[i]/MINION_WORD_SIZE; j++) {
			reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_DMEMD);
			nvlink_dbg("Byte 0x%x, Data = 0x%x", byte_num, reg_val);
			byte_num += MINION_WORD_SIZE;
		}
	}

	nvlink_dbg("");
	nvlink_dbg("MINION DMEM DUMP - END");
	nvlink_dbg("");
}

/*
 * Dump the MINION PC trace. This is useful for debugging MINION
 * errors/hangs/crashes.
 */
void minion_dump_pc_trace(struct tnvlink_dev *tdev)
{
	u32 trace_pc_count = 0;
	u32 pc = 0;
	u32 traceidx = 0;
	u32 tracepc = 0;
	u32 i = 0;
	u32 idx = 0;
	u32 reg_val = 0;

	reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_SCTL);
	nvlink_err("CMINION_FALCON_SCTL = 0x%x", reg_val);
	if (reg_val & BIT(CMINION_FALCON_SCTL_HSMODE)) {
		nvlink_err("MINION is in HS mode."
			" MINION PC TRACE dump is not supported in HS mode.");
		return;
	}

	nvlink_err("");
	nvlink_err("MINION PC TRACE DUMP - START");
	nvlink_err("");

	/* Get total number of PC trace entries */
	reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_TRACEIDX);
	nvlink_err("CMINION_FALCON_TRACEIDX = 0x%x", reg_val);
	trace_pc_count = CMINION_FALCON_TRACEIDX_MAXIDX_V(reg_val);
	nvlink_err("PC TRACE (Total entries = %d - "
		"entry 0 is the most recent branch):",
		trace_pc_count);

	/* Print the entire PC trace */
	for (i = 0; i < trace_pc_count; i++) {
		idx = CMINION_FALCON_TRACEIDX_IDX_F(i);
		nvlw_minion_writel(tdev, CMINION_FALCON_TRACEIDX, idx);
		traceidx = nvlw_minion_readl(tdev, CMINION_FALCON_TRACEIDX);

		tracepc = nvlw_minion_readl(tdev, CMINION_FALCON_TRACEPC);
		pc = CMINION_FALCON_TRACEPC_PC_V(tracepc);

		nvlink_err("   - PC(%d) = %#010x", idx, pc);
		nvlink_err("        - CMINION_FALCON_TRACEIDX = 0x%x",
			traceidx);
		nvlink_err("        - CMINION_FALCON_TRACEPC = 0x%x",
			tracepc);
	}

	nvlink_err("");
	nvlink_err("MINION PC TRACE DUMP - END");
	nvlink_err("");
}


/*
 * Dump the MINION registers which are useful for debugging MINION
 * errors/hangs/crashes.
 */
void minion_dump_registers(struct tnvlink_dev *tdev)
{
	nvlink_err("");
	nvlink_err("MINION REGISTER DUMP - START");
	nvlink_err("");

	nvlink_err("CMINION_FALCON_OS = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_OS));
	nvlink_err("CMINION_FALCON_CPUCTL = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_CPUCTL));
	nvlink_err("CMINION_FALCON_IDLESTATE = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_IDLESTATE));
	nvlink_err("CMINION_FALCON_MAILBOX0 = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_MAILBOX0));
	nvlink_err("CMINION_FALCON_MAILBOX1 = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_MAILBOX1));
	nvlink_err("CMINION_FALCON_IRQSTAT = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_IRQSTAT));
	nvlink_err("CMINION_FALCON_IRQMASK = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_IRQMASK));
	nvlink_err("CMINION_FALCON_DEBUG1 = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_DEBUG1));
	nvlink_err("CMINION_FALCON_DEBUGINFO = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_DEBUGINFO));
	nvlink_err("CMINION_FALCON_BOOTVEC = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_BOOTVEC));
	nvlink_err("CMINION_FALCON_HWCFG = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_HWCFG));
	nvlink_err("CMINION_FALCON_ENGCTL = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_ENGCTL));
	nvlink_err("CMINION_FALCON_CURCTX = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_CURCTX));
	nvlink_err("CMINION_FALCON_NXTCTX = 0x%x",
		nvlw_minion_readl(tdev, CMINION_FALCON_NXTCTX));

	/* DL_CMD related registers */
	nvlink_err("MINION_MINION_DEVICES = 0x%x",
		nvlw_minion_readl(tdev, MINION_MINION_DEVICES));
	nvlink_err("MINION_MINION_INTR = 0x%x",
		nvlw_minion_readl(tdev, MINION_MINION_INTR));
	nvlink_err("MINION_MINION_INTR_STALL_EN = 0x%x",
		nvlw_minion_readl(tdev, MINION_MINION_INTR_STALL_EN));
	nvlink_err("MINION_MINION_INTR_NONSTALL_EN = 0x%x",
		nvlw_minion_readl(tdev, MINION_MINION_INTR_NONSTALL_EN));
	nvlink_err("MINION_NVLINK_LINK_INTR = 0x%x",
		nvlw_minion_readl(tdev, MINION_NVLINK_LINK_INTR));
	nvlink_err("MINION_NVLINK_DL_CMD = 0x%x",
		nvlw_minion_readl(tdev, MINION_NVLINK_DL_CMD));
	nvlink_err("MINION_NVLINK_LINK_DL_STAT = 0x%x",
		nvlw_minion_readl(tdev, MINION_NVLINK_LINK_DL_STAT));
	nvlink_err("MINION_MINION_STATUS = 0x%x",
		nvlw_minion_readl(tdev, MINION_MINION_STATUS));

	nvlink_err("");
	nvlink_err("MINION REGISTER DUMP - END");
	nvlink_err("");
}

/*
 * minion_boot:
 * ------------
 * Boot the MINION microcontroller by executing the following steps:
 *    - Get MINION ucode from the filesystem
 *    - Read ucode header
 *    - Load ucode image sections into MINION's IMEM and DMEM
 *    - Start MINION boot and wait for boot to complete
 *    - Send the SWINTR DLCMD to the MINION and poll for expected interrupt
 *
 * If all goes well, the MINION should be booted and ready to accept DLCMDs from
 * SW.
 *
 * TODO: Currently we're making assumptions about the ordering of the IMEM/DMEM
 *       sections. We need to do a run-time sort on the starting offsets of all
 *       the IMEM/DMEM sections and then load each section in the derived order.
 */
int minion_boot(struct tnvlink_dev *tdev)
{
	int ret = 0;
	struct nvlink_device *ndev = tdev->ndev;
	struct minion_hdr *hdr = &(ndev->minion_hdr);
	int data_idx = 0;
	int i = 0;
	u32 elapsed_us = 0;
	u32 reg_val = 0;
	int dmem_scrub_pending = 1;
	int imem_scrub_pending = 1;
	int dump_ucode = 0;
	u32 minion_status = 0;
	u32 intr_code = 0;

	/* Configure minion falcon Interrupts */
	nvlink_config_minion_falcon_intr(tdev);

	/* Get MINION ucode from the filesystem */
	ret = request_firmware(&(ndev->minion_fw), MINION_FW_PATH, tdev->dev);
	if (ret) {
		nvlink_err("Can't get MINION ucode binary");
		goto exit;
	}

	/* Read ucode header */
	hdr->os_code_offset = minion_extract_word(tdev, data_idx);
	data_idx += 4;
	hdr->os_code_size = minion_extract_word(tdev, data_idx);
	data_idx += 4;
	hdr->os_data_offset = minion_extract_word(tdev, data_idx);
	data_idx += 4;
	hdr->os_data_size = minion_extract_word(tdev, data_idx);
	data_idx += 4;
	hdr->num_apps = minion_extract_word(tdev, data_idx);
	data_idx += 4;

	nvlink_dbg("MINION Ucode Header Info:");
	nvlink_dbg("-------------------------");
	nvlink_dbg("  - OS Code Offset = %u", hdr->os_code_offset);
	nvlink_dbg("  - OS Code Size = %u", hdr->os_code_size);
	nvlink_dbg("  - OS Data Offset = %u", hdr->os_data_offset);
	nvlink_dbg("  - OS Data Size = %u", hdr->os_data_size);
	nvlink_dbg("  - Num Apps = %u", hdr->num_apps);

	/* Allocate offset/size arrays for all the ucode apps */
	hdr->app_code_offsets = kcalloc(hdr->num_apps, sizeof(u32), GFP_KERNEL);
	if (!hdr->app_code_offsets) {
		nvlink_err("Couldn't allocate MINION app_code_offsets array");
		ret = -ENOMEM;
		goto cleanup;
	}

	hdr->app_code_sizes = kcalloc(hdr->num_apps, sizeof(u32), GFP_KERNEL);
	if (!hdr->app_code_sizes) {
		nvlink_err("Couldn't allocate MINION app_code_sizes array");
		ret = -ENOMEM;
		goto cleanup;
	}

	hdr->app_data_offsets = kcalloc(hdr->num_apps, sizeof(u32), GFP_KERNEL);
	if (!hdr->app_data_offsets) {
		nvlink_err("Couldn't allocate MINION app_data_offsets array");
		ret = -ENOMEM;
		goto cleanup;
	}

	hdr->app_data_sizes = kcalloc(hdr->num_apps, sizeof(u32), GFP_KERNEL);
	if (!hdr->app_data_sizes) {
		nvlink_err("Couldn't allocate MINION app_data_sizes array");
		ret = -ENOMEM;
		goto cleanup;
	}

	/* Get app code offsets and sizes */
	for (i = 0; i < hdr->num_apps; i++) {
		hdr->app_code_offsets[i] = minion_extract_word(tdev, data_idx);
		data_idx += 4;
		hdr->app_code_sizes[i] = minion_extract_word(tdev, data_idx);
		data_idx += 4;

		nvlink_dbg("  - App Code:");
		nvlink_dbg("      - App #%d: Code Offset = %u, Code Size = %u",
			i,
			hdr->app_code_offsets[i],
			hdr->app_code_sizes[i]);
	}

	/* Get app data offsets and sizes */
	for (i = 0; i < hdr->num_apps; i++) {
		hdr->app_data_offsets[i] = minion_extract_word(tdev, data_idx);
		data_idx += 4;
		hdr->app_data_sizes[i] = minion_extract_word(tdev, data_idx);
		data_idx += 4;

		nvlink_dbg("  - App Data:");
		nvlink_dbg("      - App #%d: Data Offset = %u, Data Size = %u",
			i,
			hdr->app_data_offsets[i],
			hdr->app_data_sizes[i]);
	}

	hdr->ovl_offset = minion_extract_word(tdev, data_idx);
	data_idx += 4;
	hdr->ovl_size = minion_extract_word(tdev, data_idx);
	data_idx += 4;

	ndev->minion_img = &(ndev->minion_fw->data[data_idx]);
	hdr->ucode_data_size = ndev->minion_fw->size - data_idx;

	nvlink_dbg("  - Overlay Offset = %u", hdr->ovl_offset);
	nvlink_dbg("  - Overlay Size = %u", hdr->ovl_size);
	nvlink_dbg("  - Ucode Data Size = %u", hdr->ucode_data_size);

	/* Do memory scrub */
	nvlw_minion_writel(tdev, CMINION_FALCON_DMACTL, 0);
	while (dmem_scrub_pending || imem_scrub_pending) {
		reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_DMACTL);
		dmem_scrub_pending =
			reg_val & BIT(CMINION_FALCON_DMACTL_DMEM_SCRUBBING);
		imem_scrub_pending =
			reg_val & BIT(CMINION_FALCON_DMACTL_IMEM_SCRUBBING);
	}

	/* Initialize address of IMEM to 0x0 and set auto-increment on write */
	nvlw_minion_writel(tdev,
			CMINION_FALCON_IMEMC,
			BIT(CMINION_FALCON_IMEMC_AINCW));

	/* Load OS code into the IMEM */
	nvlink_dbg("Loading OS code into the IMEM");
	ret = minion_load_ucode_section(tdev,
					1,
					hdr->os_code_offset,
					hdr->os_code_size,
					0,
					0);
	if (ret < 0) {
		nvlink_err("Unable to load MINION OS code into the IMEM");
		goto cleanup;
	}

	/* Initialize address of DMEM to 0x0 and set auto-increment on write */
	nvlw_minion_writel(tdev,
			CMINION_FALCON_DMEMC,
			BIT(CMINION_FALCON_DMEMC_AINCW));

	/* Load OS data into the DMEM */
	nvlink_dbg("Loading OS data into the DMEM");
	ret = minion_load_ucode_section(tdev,
					0,
					hdr->os_data_offset,
					hdr->os_data_size,
					0,
					0);
	if (ret < 0) {
		nvlink_err("Unable to load OS data into the DMEM");
		goto cleanup;
	}

	/* Load the ucode apps */
	for (i = 0; i < hdr->num_apps; i++) {
		/* Mark the app code as secure */
		reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_IMEMC);
		reg_val |= BIT(CMINION_FALCON_IMEMC_SECURE);
		nvlw_minion_writel(tdev, CMINION_FALCON_IMEMC, reg_val);

		/* Load app code into the IMEM */
		nvlink_dbg("Loading app %d code into the IMEM", i);
		ret = minion_load_ucode_section(tdev,
						1,
						hdr->app_code_offsets[i],
						hdr->app_code_sizes[i],
						1,
						hdr->app_code_offsets[i] >> 8);
		if (ret < 0) {
			nvlink_err("Unable to load the app code"
				" for app %d into the IMEM",
				i);
			goto cleanup;
		}

		/* Load app data into the DMEM */
		nvlink_dbg("Loading app %d data into the DMEM", i);
		ret = minion_load_ucode_section(tdev,
						0,
						hdr->app_data_offsets[i],
						hdr->app_data_sizes[i],
						0,
						0);
		if (ret < 0) {
			nvlink_err("Unable to load the app data"
				" for app %d into the DMEM",
				i);
			goto cleanup;
		}
	}

	/*
	 * If needed dump out the contents of the MINION's IMEM and DMEM so that
	 * we can verify that we're loading the ucode correctly.
	 */
	if (dump_ucode)
		minion_print_ucode(tdev);

	/* Write boot vector */
	nvlw_minion_writel(tdev, CMINION_FALCON_BOOTVEC, hdr->os_code_offset);

	/* Start MINION CPU */
	reg_val = nvlw_minion_readl(tdev, CMINION_FALCON_CPUCTL);
	reg_val |= BIT(CMINION_FALCON_CPUCTL_STARTCPU);
	nvlw_minion_writel(tdev, CMINION_FALCON_CPUCTL, reg_val);

	/* Wait for MINION to boot */
	while (1) {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		reg_val = nvlw_minion_readl(tdev, MINION_MINION_STATUS);
		minion_status = (reg_val & MINION_MINION_STATUS_STATUS_MASK) >>
					MINION_MINION_STATUS_STATUS_SHIFT;
		if (minion_status != MINION_MINION_STATUS_STATUS_INIT) {
			if (minion_status != MINION_MINION_STATUS_STATUS_BOOT) {
				nvlink_err(
					"MINION ucode initialization failed!");
				nvlink_err("MINION_MINION_STATUS = 0x%x",
					reg_val);
				ret = -1;
				goto err_dump;
			} else {
				u32 os = 0;
				u32 os_maj_ver = 0;
				u32 os_min_ver = 0;
				u32 mbox = 0;
				u32 sctl = 0;

				nvlink_dbg("MINION booted successfully!");

				os = nvlw_minion_readl(tdev,
							CMINION_FALCON_OS);
				os_maj_ver =
					(os &
					CMINION_FALCON_OS_MAJOR_VER_MASK) >>
					CMINION_FALCON_OS_MAJOR_VER_SHIFT;
				os_min_ver =
					(os &
					CMINION_FALCON_OS_MINOR_VER_MASK) >>
					CMINION_FALCON_OS_MINOR_VER_SHIFT;
				mbox = nvlw_minion_readl(tdev,
						CMINION_FALCON_MAILBOX1);
				sctl = nvlw_minion_readl(tdev,
							CMINION_FALCON_SCTL);

				/* Dump the ucode ID string epilog */
				nvlink_dbg("MINION Falcon ucode version info:"
					" Ucode v%d.%d, Phy v%d",
					os_maj_ver,
					os_min_ver,
					mbox);

				/* Display security level info */
				nvlink_dbg("CMINION_FALCON_SCTL = 0x%x",
					sctl);

				break;
			}
		}

		if (elapsed_us >= DEFAULT_LOOP_TIMEOUT_US) {
			nvlink_err("Timeout waiting for MINION to boot!");
			nvlink_err("MINION_MINION_STATUS = 0x%x", reg_val);
			ret = -1;
			goto err_dump;
		}

		/* Service any pending falcon interrupts */
		minion_service_falcon_intr(tdev);
	}

	/* Wait until MINION is ready to accept commands */
	ret = wait_for_reg_cond_nvlink(tdev,
					MINION_NVLINK_DL_CMD,
					MINION_NVLINK_DL_CMD_READY,
					true,
					"MINION_NVLINK_DL_CMD_READY",
					nvlw_minion_readl,
					&reg_val,
					DEFAULT_LOOP_TIMEOUT_US);
	if (ret < 0) {
		nvlink_err("MINION booted but its not accepting commands");
		goto err_dump;
	}

	/*
	 * Send a SWINTR DLCMD to MINION to test if it’s functioning
	 * properly
	 */
	ret = minion_send_cmd(tdev, MINION_NVLINK_DL_CMD_COMMAND_SWINTR, 0);
	if (ret < 0) {
		nvlink_err("MINION SWINTR DLCMD failed!");
		goto err_dump;
	}

	/* Check interrupt register to see if interrupt was received */
	reg_val = nvlw_minion_readl(tdev, MINION_NVLINK_LINK_INTR);
	intr_code = (reg_val & MINION_NVLINK_LINK_INTR_CODE_MASK) >>
			MINION_NVLINK_LINK_INTR_CODE_SHIFT;
	if (intr_code == MINION_NVLINK_LINK_INTR_CODE_SWREQ) {
		nvlink_dbg("MINION SWINTR DLCMD succeeded!");
	} else {
		nvlink_err("MINION SWINTR DLCMD failed!"
			" SW requested interrupt was not received.");
		nvlink_err("MINION_NVLINK_LINK_INTR: 0x%x", reg_val);
		ret = -1;
		goto err_dump;
	}

	goto cleanup;

err_dump:
	/* Dump the PC trace and misc registers for error conditions */
	minion_dump_pc_trace(tdev);
	minion_dump_registers(tdev);

cleanup:
	release_firmware(ndev->minion_fw);
	kfree(hdr->app_code_offsets);
	kfree(hdr->app_code_sizes);
	kfree(hdr->app_data_offsets);
	kfree(hdr->app_data_sizes);
	memset(hdr, 0, sizeof(struct minion_hdr));

exit:
	ndev->minion_fw = NULL;
	ndev->minion_img = NULL;
	return ret;
}

/*
 * init_nvhs_phy:
 * Initialize the NVHS PHY. This encompasses the following:
 *    - Sending MINION DLCMDs for various PHY init steps
 *    - Switching the TX clock from OSC to brick PLL
 *    - RX calibration of lanes
 */
int init_nvhs_phy(struct tnvlink_dev *tdev)
{
	int ret = 0;
	bool dump_minion = true;
	u32 reg_val = 0;
	u32 link_state = 0;
	struct nvlink_device *ndev = tdev->ndev;

	ret = minion_send_cmd(tdev,
			MINION_NVLINK_DL_CMD_COMMAND_XAVIER_PLLOVERRIDE_ON,
			0);
	if (ret < 0) {
		nvlink_err("Error sending XAVIER_PLLOVERRIDE_ON command to"
			" MINION");
		goto fail;
	}

	if (tdev->is_nea) {
		ret = minion_send_cmd(tdev,
					MINION_NVLINK_DL_CMD_COMMAND_SETNEA,
					0);
		if (ret < 0) {
			nvlink_err("Error sending SETNEA command to MINION");
			goto fail;
		}
	}

	reg_val = nvlw_nvl_readl(tdev, NVL_LINK_STATE);
	link_state = (reg_val & NVL_LINK_STATE_STATE_MASK) >>
					NVL_LINK_STATE_STATE_SHIFT;
	if (link_state != NVL_LINK_STATE_STATE_INIT) {
		nvlink_err("Link is not in INIT state."
				" INITPLL can only be executed in INIT state.");
		ret = -1;

		/*
		 * This is not a MINION error condition. We don't need a MINION
		 * debug dump.
		 */
		dump_minion = false;
		goto fail;
	} else if ((tdev->refclk == NVLINK_REFCLK_150) &&
			(ndev->speed == NVLINK_SPEED_20)) {
		ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITPLL_5,
				0);
		if (ret < 0) {
			nvlink_err("Error sending INITPLL_5 command to MINION");
			goto fail;
		}

		ndev->link_bitrate = LINK_BITRATE_150MHZ_20GBPS;
	} else if ((tdev->refclk == NVLINK_REFCLK_150) &&
			(ndev->speed == NVLINK_SPEED_16)) {
		ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITPLL_9,
				0);
		if (ret < 0) {
			nvlink_err("Error sending INITPLL_9 command to MINION");
			goto fail;
		}

		ndev->link_bitrate = LINK_BITRATE_150MHZ_16GBPS;
	} else if ((tdev->refclk == NVLINK_REFCLK_156) &&
			(ndev->speed == NVLINK_SPEED_20)) {
		ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITPLL_4,
				0);
		if (ret < 0) {
			nvlink_err("Error sending INITPLL_4 command to MINION");
			goto fail;
		}

		ndev->link_bitrate = LINK_BITRATE_156MHZ_20GBPS;
	} else if ((tdev->refclk == NVLINK_REFCLK_156) &&
			 (ndev->speed == NVLINK_SPEED_16)) {
		ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITPLL_8,
				0);
		if (ret < 0) {
			nvlink_err("Error sending INITPLL_8 command to MINION");
			goto fail;
		}

		ndev->link_bitrate = LINK_BITRATE_156MHZ_16GBPS;
	} else {
		nvlink_err("Invalid speed or refclk");
		ret = -EINVAL;
		goto fail;
	}

	ret = minion_send_cmd(tdev,
			MINION_NVLINK_DL_CMD_COMMAND_XAVIER_CALIBRATEPLL,
			0);
	if (ret < 0) {
		nvlink_err("Error sending XAVIER_CALIBRATEPLL command to"
								" MINION");
		goto fail;
	}

	ret = clk_set_rate(tdev->clk_nvlink_pll_txclk, ndev->link_bitrate / 16);
	if (ret < 0) {
		nvlink_err("clk_nvlink_pll_txclk's clk_set_rate() call failed");

		/*
		 * This is not a MINION error condition. We don't need a MINION
		 * debug dump.
		 */
		dump_minion = false;
		goto fail;
	}

	/* Switch the TX clock from OSC to brick PLL */
	ret = clk_set_parent(tdev->clk_nvlink_tx, tdev->clk_nvlink_pll_txclk);
	if (ret < 0) {
		nvlink_err("clk_nvlink_tx's clk_set_parent() call failed");

		/*
		 * This is not a MINION error condition. We don't need a MINION
		 * debug dump.
		 */
		dump_minion = false;
		goto fail;
	}

	/* INITRXTERM is required if connected to nvlink 2.2 device.
	 * For RXDET functionality to succeed on 2.2 devices, the opposite
	 * endpoint must initialize the RX termination.It does no harm when
	 * connected to 2.0 device.
	 */
	ret = minion_send_cmd(tdev, MINION_NVLINK_DL_CMD_COMMAND_INITRXTERM,
									0);
	if (ret < 0) {
		nvlink_err("Error sending INITRXTERM command to MINION");
		goto fail;
	}

	ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITPHY,
				0);
	if (ret < 0) {
		nvlink_err("Error sending INITPHY command to MINION");
		goto undo_clk;
	}

	/* RX calibration */
	reg_val = BIT(NVL_BR0_CFG_CTL_CAL_RXCAL) |
			BIT(NVL_BR0_CFG_CTL_CAL_INIT_TRAIN_DONE);
	nvlw_nvl_writel(tdev, NVL_BR0_CFG_CTL_CAL, reg_val);

	/* Wait for RXCAL_DONE bit to be set */
	ret = wait_for_reg_cond_nvlink(tdev,
					NVL_BR0_CFG_STATUS_CAL,
					NVL_BR0_CFG_STATUS_CAL_RXCAL_DONE,
					true,
					"NVL_BR0_CFG_STATUS_CAL_RXCAL_DONE",
					nvlw_nvl_readl,
					&reg_val,
					DEFAULT_LOOP_TIMEOUT_US);
	if (ret < 0) {
		nvlink_err("RX calibration failed!");

		/*
		 * This is not a MINION error condition. We don't need a MINION
		 * debug dump.
		 */
		dump_minion = false;
		goto undo_clk;
	}

	ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITLANEENABLE,
				0);
	if (ret < 0) {
		nvlink_err("Error sending INITLANEENABLE command to MINION");
		goto undo_clk;
	}

	ret = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_INITDLPL,
				0);
	if (ret < 0) {
		nvlink_err("Error sending INITDLPL command to MINION");
		goto undo_clk;
	}

	nvlink_dbg("NVHS PHY init succeeded!");
	goto success;

undo_clk:
	/* Switch the TX clock from brick PLL to OSC */
	ret = clk_set_parent(tdev->clk_nvlink_tx, tdev->clk_m);
	if (ret < 0)
		nvlink_err("clk_nvlink_tx's clk_set_parent() call failed");
fail:
	nvlink_err("NVHS PHY init failed!");

	/*
	 * Dump the MINION PC trace and misc registers for MINION error
	 * conditions
	 */
	if (dump_minion) {
		minion_dump_pc_trace(tdev);
		minion_dump_registers(tdev);
	}
success:
	return ret;
}
