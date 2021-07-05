/*
 * tegracam_utils - tegra camera framework utilities
 *
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>
#include <linux/regmap.h>
#include <media/tegracam_core.h>
#include <media/tegracam_utils.h>

bool is_tvcf_supported(u32 version)
{
	/* 2.0.0 is the base tvcf version sensor driver*/
	return (version >= tegracam_version(2, 0, 0) ? true : false);
}
EXPORT_SYMBOL_GPL(is_tvcf_supported);

int format_tvcf_version(u32 version, char *buff, size_t size)
{
	if (buff == NULL)
		return -EINVAL;

	return snprintf(buff, size, "%u.%u.%u",
			(u8)(version >> 16),
			(u8)(version >> 8),
			(u8)(version));
}
EXPORT_SYMBOL_GPL(format_tvcf_version);

void conv_u32_u8arr(u32 input, u8 *output)
{
	output[0] = (input >> 24) & 0xFF;
	output[1] = (input >> 16) & 0xFF;
	output[2] = (input >> 8) & 0xFF;
	output[3] = input & 0xFF;
}
EXPORT_SYMBOL_GPL(conv_u32_u8arr);

void conv_u16_u8arr(u16 input, u8 *output)
{
	output[0] = (input >> 8) & 0xFF;
	output[1] = input & 0xFF;
}
EXPORT_SYMBOL_GPL(conv_u16_u8arr);

static inline int is_valid_blob(struct sensor_blob *blob, u32 size)
{
	if (!blob)
		return -EINVAL;

	if ((blob->num_cmds >= MAX_COMMANDS) ||
		((blob->buf_size + size) >= MAX_BLOB_SIZE))
		return -ENOMEM;

	return 0;
}

int prepare_write_cmd(struct sensor_blob *blob,
			u32 size, u32 addr, u8 *buf)
{
	struct sensor_cmd *cmd = NULL;
	int err = 0;

	err = is_valid_blob(blob, size);
	if (err)
		return err;

	cmd = &blob->cmds[blob->num_cmds++];
	cmd->opcode = ((SENSOR_OPCODE_WRITE << 24) | size);
	cmd->addr = addr;

	memcpy(&blob->buf[blob->buf_size], buf, size);

	blob->buf_size += size;

	return 0;
}
EXPORT_SYMBOL_GPL(prepare_write_cmd);

int prepare_read_cmd(struct sensor_blob *blob,
			u32 size, u32 addr)
{
	struct sensor_cmd *cmd = NULL;
	int err = 0;

	err = is_valid_blob(blob, size);
	if (err)
		return err;

	cmd = &blob->cmds[blob->num_cmds++];
	cmd->opcode = ((SENSOR_OPCODE_READ << 24) | size);
	cmd->addr = addr;

	blob->buf_size += size;

	return 0;
}
EXPORT_SYMBOL_GPL(prepare_read_cmd);

int prepare_sleep_cmd(struct sensor_blob *blob, u32 time_in_us)
{
	struct sensor_cmd *cmd = NULL;
	int err = 0;

	err = is_valid_blob(blob, 0);
	if (err)
		return err;

	cmd = &blob->cmds[blob->num_cmds++];
	cmd->opcode = (SENSOR_OPCODE_SLEEP << 24) | time_in_us;

	return 0;
}
EXPORT_SYMBOL_GPL(prepare_sleep_cmd);

int prepare_done_cmd(struct sensor_blob *blob)
{
	struct sensor_cmd *cmd = NULL;
	int err = 0;

	err = is_valid_blob(blob, 0);
	if (err)
		return err;

	cmd = &blob->cmds[blob->num_cmds++];
	cmd->opcode = SENSOR_OPCODE_DONE;

	return 0;
}
EXPORT_SYMBOL_GPL(prepare_done_cmd);

int convert_table_to_blob(struct sensor_blob *blob,
			  const struct reg_8 table[],
			  u16 wait_ms_addr, u16 end_addr)
{
	const struct reg_8 *next;
	u16 addr;
	u8 val;
	int range_start = -1;
	int range_count = 0;
	u8 buf[16];

	for (next = table;; next++) {
		val = next->val;
		addr = next->addr;
		if (range_start == -1)
			range_start = next->addr;

		if (range_count == 16 ||
			(addr != (range_start + range_count))) {
			/* write opcode and size for store index*/
			prepare_write_cmd(blob, range_count,
						range_start, &buf[0]);
			range_start = addr;
			range_count = 0;
		}

		/* Done command must be added by client */
		if (addr == end_addr)
			break;

		if (addr == wait_ms_addr) {
			prepare_sleep_cmd(blob, (next->val * 1000));
			range_start = -1;
			continue;
		}

		buf[range_count++] = val;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(convert_table_to_blob);

int write_sensor_blob(struct regmap *regmap, struct sensor_blob *blob)
{
	int err = 0;
	int cmd_idx = 0;
	int buf_index = 0;

	while (cmd_idx < blob->num_cmds) {
		struct sensor_cmd *cmd = &blob->cmds[cmd_idx++];
		u32 val;

		val = cmd->opcode;
		if ((val >> 24) == SENSOR_OPCODE_DONE)
			break;

		if ((val >> 24) == SENSOR_OPCODE_SLEEP) {
			val = val & 0x00FFFFFF;
			usleep_range(val, val + 10);
			continue;
		}

		if ((val >> 24) == SENSOR_OPCODE_WRITE) {
			int size = val & 0x00FFFFFF;

			err = regmap_bulk_write(regmap, cmd->addr,
					&blob->buf[buf_index], size);
			if (err)
				return err;
			buf_index += size;
		} else {
			pr_err("blob has been packaged with errors\n");
			return -EINVAL;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(write_sensor_blob);

int tegracam_write_blobs(struct tegracam_ctrl_handler *hdl)
{
	struct camera_common_data *s_data = hdl->tc_dev->s_data;
	struct tegracam_sensor_data *sensor_data = &hdl->sensor_data;
	struct sensor_blob *ctrl_blob = &sensor_data->ctrls_blob;
	struct sensor_blob *mode_blob = &sensor_data->mode_blob;
	const struct tegracam_ctrl_ops *ops = hdl->ctrl_ops;
	int err = 0;

	/* no blob control available */
	if (ops == NULL || !ops->is_blob_supported)
		return 0;

	/*
	 * TODO: Extend this to multiple subdevices
	 * mode blob commands can be zero for auto control updates
	 * and stop streaming cases
	 */
	if (mode_blob->num_cmds) {
		err = write_sensor_blob(s_data->regmap, mode_blob);
		if (err) {
			dev_err(s_data->dev, "Error writing mode blob\n");
			return err;
		}
	}

	err = write_sensor_blob(s_data->regmap, ctrl_blob);
	if (err) {
		dev_err(s_data->dev, "Error writing control blob\n");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegracam_write_blobs);
