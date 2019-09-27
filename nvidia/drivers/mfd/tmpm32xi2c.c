/*
 * drivers/mfd/tmpm32xi2c.c
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/mfd/tmpm32xi2c.h>

static struct mfd_cell tmpm32xi2c_cells[] = {
	{ .name = "tmpm32xi2c-gpio", },
	{ .name = "tmpm32xi2c-poweroff", },
};

static int __tmpm32xi2c_write_read(struct tmpm32xi2c_chip *chip,
			u8 *tx_buf, int tx_size, u8 *rx_buf, int rx_size)
{
	struct i2c_client *client =
			container_of(chip->dev, struct i2c_client, dev);
	struct i2c_msg msg[2];
	int msg_num = 0;
	int ret = 0;

	if (tx_size > 0) {
		msg[msg_num].addr = client->addr;
		msg[msg_num].flags = client->flags & I2C_M_TEN;
		msg[msg_num].len = tx_size;
		msg[msg_num].buf = tx_buf;
		msg_num++;
	}

	if (rx_size > 0) {
		msg[msg_num].addr = client->addr;
		msg[msg_num].flags = client->flags & I2C_M_TEN;
		msg[msg_num].flags |= I2C_M_RD;
		msg[msg_num].len = rx_size;
		msg[msg_num].buf = rx_buf;
		msg_num++;
	}

	ret = i2c_transfer(client->adapter, msg, msg_num);
	if (ret < 0)
		dev_err_ratelimited(chip->dev,
				    "Failed to transfer data, CMD[0x%02x]\n",
				    tx_buf ? tx_buf[0] : 0);

	return ret;
}

static int tmpm32xi2c_write_read(struct tmpm32xi2c_chip *chip,
			u8 *tx_buf, int tx_size, u8 *rx_buf, int rx_size)
{
	int ret;

	mutex_lock(&chip->lock);
	ret = __tmpm32xi2c_write_read(chip, tx_buf, tx_size, rx_buf, rx_size);
	mutex_unlock(&chip->lock);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static int __tmpm32xi2c_power_get_value(struct tmpm32xi2c_chip *chip,
					unsigned int cmd, unsigned int ch)
{
	int ret, i, max_bytes;
	u8 tx_buf[] = { 0 /*cmd*/, 0 /*ch*/ };
	u8 rx_buf[] = { 0 /*val*/ };
	uint32_t val = 0;
	bool err_flag = false;

	if ((cmd != CMD_RD_VOLT) && (cmd != CMD_RD_ADC)) {
		dev_err(chip->dev, "%s: Invalid CMD[0x%02x]\n", __func__, cmd);
		return 0;
	}

	dev_dbg(chip->dev, "%s: CMD[0x%02x] CH[0x%02x]\n", __func__, cmd, ch);

	/* send channel info */
	tx_buf[0] = (u8)cmd;
	tx_buf[1] = (u8)ch;
	ret = __tmpm32xi2c_write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
	if (ret < 0) {
		dev_err(chip->dev, "%s: Failed to send channel info, %d\n",
			__func__, ret);
		return 0;
	}

	/*
	 * Delay for allowing MCU to read the INA sensor and save the result
	 * into I2C return value.
	 */
	usleep_range(10000, 10100);

	/* set byte length for reading */
	if (cmd == CMD_RD_VOLT)
		max_bytes = 4;
	else
		max_bytes = 2;

	for (i = 0; i < max_bytes; i++) {
		/* send cmd only */
		ret = __tmpm32xi2c_write_read(chip, tx_buf, 1,
					      rx_buf, sizeof(rx_buf));
		if (ret < 0) {
			dev_err(chip->dev, "%s: Failed to read %dth byte, %d\n",
				__func__, i, ret);
			err_flag = true;
		}
		val <<= 8;
		val |= rx_buf[0];
	}

	if (err_flag)
		return 0;
	else
		return val;
}

static int tmpm32xi2c_ina3221_get_value(struct tmpm32xi2c_chip *chip,
					unsigned int ch)
{	int ret;

	/* Operations of reading voltage value should be executed in a row. */
	mutex_lock(&chip->lock);
	ret = __tmpm32xi2c_power_get_value(chip, CMD_RD_VOLT, ch);
	mutex_unlock(&chip->lock);

	return ret;
}

static int tmpm32xi2c_adc_get_value(struct tmpm32xi2c_chip *chip,
				    unsigned int ch)
{
	int ret;

	/* Operations of reading voltage value should be executed in a row. */
	mutex_lock(&chip->lock);
	ret = __tmpm32xi2c_power_get_value(chip, CMD_RD_ADC, ch);
	mutex_unlock(&chip->lock);

	return ret;
}

union ina_data {
	uint32_t raw;
	struct {
		uint16_t mv; /* low 2B */
		uint16_t ma; /* high 2B */
	} w;
};

static int show_tmpm32xi2c_power_stats(struct seq_file *s, void *data)
{
	struct tmpm32xi2c_chip *chip = s->private;
	uint32_t sys_9v_mv, vbat_mv, vdd_ddr_1v8_ma;
	union ina_data vdd_gpu, vdd_bcpu, vdd_mcpu;
	union ina_data vdd_soc, vdd_sram, vdd_ddr;
	const char *pwr_val_fmt = "%12s\t%6d\t%6d\t%6d\n";

	sys_9v_mv = tmpm32xi2c_adc_get_value(chip, R_SYS_9V);
	vbat_mv = tmpm32xi2c_adc_get_value(chip, R_VBAT);
	vdd_ddr_1v8_ma = tmpm32xi2c_adc_get_value(chip, R_VDD_DDR_1V8);
	vdd_gpu.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_GPU);
	vdd_bcpu.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_BCPU);
	vdd_mcpu.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_MCPU);
	vdd_soc.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_SOC);
	vdd_sram.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_SRAM);
	vdd_ddr.raw = tmpm32xi2c_ina3221_get_value(chip, R_VDD_DDR);

	seq_printf(s, "%12s\t%6s\t%6s\t%6s\n", "PWR RAIL", "mA", "mV", "mW");
	/* TMPM32X ADC */
	seq_printf(s, pwr_val_fmt, "SYS_9V", 0, sys_9v_mv, 0);
	seq_printf(s, pwr_val_fmt, "VBAT_MAIN", 0, vbat_mv, 0);
	seq_printf(s, pwr_val_fmt, "VDD_1V8_DDR",  vdd_ddr_1v8_ma, 0, 0);
	/* INA3221 */
	seq_printf(s, pwr_val_fmt, "VDD_GPU",
		vdd_gpu.w.ma, vdd_gpu.w.mv,
		(vdd_gpu.w.ma * vdd_gpu.w.mv) / 1000);
	seq_printf(s, pwr_val_fmt, "VDD_BCPU",
		vdd_bcpu.w.ma, vdd_bcpu.w.mv,
		(vdd_bcpu.w.ma * vdd_bcpu.w.mv) / 1000);
	seq_printf(s, pwr_val_fmt, "VDD_MCPU",
		vdd_mcpu.w.ma, vdd_mcpu.w.mv,
		(vdd_mcpu.w.ma * vdd_mcpu.w.mv) / 1000);
	seq_printf(s, pwr_val_fmt, "VDD_SOC",
		vdd_soc.w.ma, vdd_soc.w.mv,
		(vdd_soc.w.ma * vdd_soc.w.mv) / 1000);
	seq_printf(s, pwr_val_fmt, "VDD_SRAM",
		vdd_sram.w.ma, vdd_sram.w.mv,
		(vdd_sram.w.ma * vdd_sram.w.mv) / 1000);
	seq_printf(s, pwr_val_fmt, "VDD_DDR",
		vdd_ddr.w.ma, vdd_ddr.w.mv,
		(vdd_ddr.w.ma * vdd_ddr.w.mv) / 1000);

	return 0;
}

static int tmpm32xi2c_power_stats_dump(struct inode *inode, struct file *file)
{
	return single_open(file, show_tmpm32xi2c_power_stats, inode->i_private);
}

static const struct file_operations tmpm32xi2c_power_stats_fops = {
	.open		= tmpm32xi2c_power_stats_dump,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void tmpm32xi2c_debugfs_init(struct tmpm32xi2c_chip *chip)
{
	struct dentry *root;

	root = debugfs_create_dir("tmpm32xi2c", NULL);
	if (!root)
		return;

	if (!debugfs_create_file("pwr_stat", 0444, root, chip,
				 &tmpm32xi2c_power_stats_fops))
		goto err_root;

	return;

err_root:
	debugfs_remove_recursive(root);
}
#endif /* CONFIG_DEBUG_FS */

static int tmpm32xi2c_read_version(struct tmpm32xi2c_chip *chip)
{
	int ret = -1;
	u8 tx[] = { CMD_VERSION, 0 /*ver type*/ };

	tx[1] = V_FW;
	ret = chip->write_read(chip, tx, sizeof(tx), chip->fw_ver, 2);
	if (ret < 0)
		goto exit;

	tx[1] = V_CMD;
	ret = chip->write_read(chip, tx, sizeof(tx), &chip->cmd_ver, 1);
	if (ret < 0)
		goto exit;

	dev_info(chip->dev, "FW version [0x%x][0x%x]\n",
		 chip->fw_ver[0], chip->fw_ver[1]);
	dev_info(chip->dev, "CMD version [0x%x]\n", chip->cmd_ver);

exit:
	return ret;
}

static int tmpm32xi2c_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct tmpm32xi2c_chip *chip = i2c_get_clientdata(client);
	u32 val = 0;
	int ret;

	if (!np) {
		dev_err(&client->dev, "No device node\n");
		return -ENOENT;
	}

	chip->irq_flags = IRQF_TRIGGER_LOW;
	ret = of_property_read_u32(np, "tmpm32xi2c,irq_flags", &val);
	if (!ret)
		chip->irq_flags = val;

	return 0;
}

static int tmpm32xi2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct tmpm32xi2c_chip *chip;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(struct tmpm32xi2c_chip),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (!client->irq) {
		dev_err(&client->dev, "No IRQ\n");
		return -EINVAL;
	}

	chip->dev = &client->dev;
	chip->irq = client->irq;
	chip->write_read = tmpm32xi2c_write_read;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->lock);

	ret = tmpm32xi2c_parse_dt(client);
	if (ret < 0)
		return ret;

	ret = tmpm32xi2c_read_version(chip);
	if (ret < 0)
		return ret;

	ret =  mfd_add_devices(&client->dev, PLATFORM_DEVID_NONE,
			       tmpm32xi2c_cells, ARRAY_SIZE(tmpm32xi2c_cells),
			       NULL, 0, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to add sub devices, %d\n", ret);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	tmpm32xi2c_debugfs_init(chip);
#endif

	return 0;
}

static int tmpm32xi2c_remove(struct i2c_client *client)
{
	struct tmpm32xi2c_chip *chip = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, false);
	mutex_destroy(&chip->lock);

	return 0;
}

static const struct i2c_device_id tmpm32xi2c_id[] = {
	{ "tmpm32xi2c", 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmpm32xi2c_id);

static const struct of_device_id tmpm32xi2c_dt_ids[] = {
	{ .compatible = "toshiba,tmpm32xi2c" },
	{ .compatible = "nvidia,tmpm32xi2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, tmpm32xi2c_dt_ids);

static struct i2c_driver tmpm32xi2c_driver = {
	.driver = {
		.name	= "tmpm32xi2c",
		.of_match_table = tmpm32xi2c_dt_ids,
	},
	.probe		= tmpm32xi2c_probe,
	.remove		= tmpm32xi2c_remove,
	.id_table	= tmpm32xi2c_id,
};

static int __init tmpm32xi2c_init(void)
{
	return i2c_add_driver(&tmpm32xi2c_driver);
}
/* register after postcore initcall and subsys initcall that may rely on I2C. */
subsys_initcall_sync(tmpm32xi2c_init);

static void __exit tmpm32xi2c_exit(void)
{
	i2c_del_driver(&tmpm32xi2c_driver);
}
module_exit(tmpm32xi2c_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("MFD core driver for TMPM32x I2C");
MODULE_LICENSE("GPL");
