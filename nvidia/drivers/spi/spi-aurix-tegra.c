/*
 * SPI Protocol Driver for Aurix-Tegra communication.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <soc/tegra/virt/tegra_hv_pm_ctl.h>

#define AURIX			0x3
#define TEGRA			0x2
#define SRC(x)			((x))
#define DEST(x)			((x) << 4)
#define FUSA			0x2
#define SHUTDOWN		0x6
#define RESET			0x7
#define SUSPEND			0x8
#define CMD_SHIFT(x)	((x) << 3)
#define AURIX_TEGRA		0x0
#define TEGRA_AURIX		0x1
#define REQ_TYPE(x)		((x) << 7)
#define SPI_DEBUG		1

/*
 * Command-Response packet shall follow the structure shown below.
 * Packet contains information (e.g. class id, rsp flag) that
 * requires 4, 3 or 1 bit. Using Bit Field since we need less than
 * 1 byte memory.
 */
struct packet {
	u8 e2e_crc;
	u8 e2e_alive_counter:4;
	u8 data_id_nibble:4;
	u8 src_layer_id:4;
	u8 dest_layer_id:4;
	u8 class_id:3;
	u8 cmd_msg_id:4;
	u8 rsp_flag:1;
	u8 msg_spec_payload[12];
} __attribute__((__packed__));

struct aurix_tegra_spi_data {
	struct spi_device *spi;
	struct task_struct *thread;
	struct packet *transmit;
	struct packet *receive;
	spinlock_t lock;
	bool exited;
	u8 buf_len;
	u8 command;
};

static void print_message(struct aurix_tegra_spi_data *data)
{
#if SPI_DEBUG == 1
	struct spi_device *spi = data->spi;
	u8 *msg = (u8*) data->receive;
	int len = data->buf_len;
	int i;

	for (i = 0; i < len; i=i+4)
		dev_info(&spi->dev, "%02X %02X %02X %02X",
			msg[i], msg[i+1], msg[i+2], msg[i+3]);
#else

#endif
}

/*
 * return value: cmd_msg_id (SHUTDOWN, RESET or SUSPEND),
 * -1 otherwise (command not valid).
 */
static int read_cmd_message_id(struct aurix_tegra_spi_data *data)
{
	int ret;
	struct spi_device *spi = data->spi;
	struct packet *msg = data->receive;

	if (msg->src_layer_id != AURIX) {
		dev_err(&spi->dev, "Source is not Aurix\n");
		return -1;
	}
	if (msg->dest_layer_id != TEGRA) {
		dev_err(&spi->dev, "Destination is not Tegra\n");
		return -1;
	}
	if (msg->class_id != FUSA) {
		dev_err(&spi->dev, "Class id not valid\n");
		return -1;
	}
	if (msg->rsp_flag != AURIX_TEGRA) {
		dev_err(&spi->dev, "RSP flag not valid\n");
		return -1;
	}

	switch(msg->cmd_msg_id) {
		case SHUTDOWN:
			ret = SHUTDOWN;
			break;
		case RESET:
			ret = RESET;
			break;
		case SUSPEND:
			ret = SUSPEND;
			break;
		default:
			ret = -1;
	}

	return ret;
}

/*
 * return value: 0 on success (valid command received),
 * negative value otherwise (failure to receive or command is not valid)
 */
static int aurix_tegra_receive(struct aurix_tegra_spi_data *data)
{
	int ret;
	struct spi_device *spi = data->spi;
	struct spi_message msg;
	u8	*tx_buf = (u8*) data->transmit;
	u8	*rx_buf = (u8*) data->receive;
	u8 len = data->buf_len;
	struct spi_transfer	tr = {
		.rx_buf = rx_buf,
		.tx_buf = tx_buf,
		.len = len,
		.delay_usecs = 0, /* wait forever (until Aurix sents data) */
	};

	memset(rx_buf, 0, len);
	memset(tx_buf, 0xFF, len);

	spi_message_init(&msg);
	spi_message_add_tail(&tr, &msg);
	ret = spi_sync(spi, &msg);

	BUG_ON(ret > 0);

	if (ret < 0) {
		dev_dbg(&spi->dev, "%s: spi_sync() ret val is %d\n",
			__func__, ret);
		return ret;
	}
	print_message(data);

	ret = read_cmd_message_id(data);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Command received not valid\n",
			__func__);
		return -1;
	}

	data->command = ret;

	return 0;
}

static int trigger_command(struct aurix_tegra_spi_data *data)
{
	int err;
	struct device *dev = &data->spi->dev;
	dev_dbg(&data->spi->dev, "%s: Triggering Command %x\n",
		__func__, data->command);

	switch(data->command) {
		case SHUTDOWN:
			err = tegra_hv_pm_ctl_trigger_sys_shutdown();
			if (err < 0) {
				dev_err(dev, "%s: trigger_sys_shutdown failed\n",
					__func__);
				return err;
			}
			break;
		case RESET:
			err = tegra_hv_pm_ctl_trigger_sys_reboot();
			if (err < 0) {
				dev_err(dev, "%s: trigger_sys_reboot failed\n",
					__func__);
				return err;
			}
			break;
		case SUSPEND:
			err = tegra_hv_pm_ctl_trigger_sys_suspend();
			if (err < 0) {
				dev_err(dev, "%s: trigger_sys_suspend failed\n",
					__func__);
				return err;
			}
			break;
		default:
			BUG();
	}

	return 0;
}

static int aurix_tegra_read_thread(void *data)
{
	struct aurix_tegra_spi_data *aurix_data = (struct aurix_tegra_spi_data *) data;
	struct device *dev = &aurix_data->spi->dev;
	unsigned long flags;
	int err = 0;

	/*
	 * Error from aurix_tegra_receive() is raised in one of
	 * the following cases:
	 * i) spi_sync() failed
	 * ii) command received not valid.
	 * On aurix_tegra_stop_kthread(), SIGINT is sent to kthread
	 * ONLY if kthread is still running (exited = false). That way,
	 * kthread unblocks from spi_sync() and exits with error value.
	 * Thus, checking if kthread_should_stop is not necessary.
	 */
	err = aurix_tegra_receive(aurix_data);
	if (err < 0) {
		dev_dbg(dev, "%s: Error receiving\n", __func__);
		goto ret;
	}
	err = trigger_command(aurix_data);
	if (err < 0)
		goto ret;

ret:
	spin_lock_irqsave(&aurix_data->lock, flags);
	aurix_data->exited = true;
	spin_unlock_irqrestore(&aurix_data->lock, flags);
	do_exit(err);
}

static const struct of_device_id aurix_tegra_ids[] = {
	{ .compatible = "aurix-tegra-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, aurix_tegra_ids);

static int aurix_tegra_spi_probe(struct spi_device *spi)
{
	u32 value;
	int err = 0;
	struct aurix_tegra_spi_data *data;
	struct device_node *np = spi->dev.of_node;

	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data))
		return -ENOMEM;

	spin_lock_init(&data->lock);
	data->exited = false;

	err = of_property_read_u32(np, "spi-max-frequency", &value);
	if (err < 0) {
		dev_err(&spi->dev, "no property for frequency\n");
		goto error;
	}
	spi->max_speed_hz = value;

	err = of_property_read_u32(np, "reg", &value);
	if (err < 0) {
		dev_err(&spi->dev, "no property for reg\n");
		goto error;
	}

	/*
	 * Slave can receive and transmit data in Mode 1
	 */
	spi->mode = SPI_CPHA;

	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "spi_setup failed!\n");
		goto error;
	}

	data->spi = spi;
	data->buf_len = sizeof(struct packet);
	data->transmit = devm_kzalloc(&spi->dev, data->buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(data->transmit)) {
		err = -ENOMEM;
		goto error;
	}
	data->receive = devm_kzalloc(&spi->dev, data->buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(data->receive)) {
		err = -ENOMEM;
		goto error;
	}

	/*
	 * Kernel thread handles the communication between Aurix and Tegra.
	 * It remains idle, until a request from Aurix is sent. Upon receiving,
	 * it triggers the corresponding command.
	 */
	data->thread = kthread_run(aurix_tegra_read_thread,
		(void*) data, "aurix_tegra_kthread");
	if (IS_ERR_OR_NULL(data->thread)) {
		err = PTR_ERR(data->thread);
		goto error;
	}

	spi_set_drvdata(spi, data);
	return 0;

error:
	if (data->transmit) {
		devm_kfree(&spi->dev, (void*) data->transmit);
	}
	if (data->receive) {
		devm_kfree(&spi->dev, (void*) data->receive);
	}
	devm_kfree(&spi->dev, (void*) data);
	return err;
}

/*
 * Sending SIGINT to kthread unblocks it from spi_sync()
 * and forces it to exit. SIGINT should raised ONLY if
 * exited status is false, meaning that kthread is still
 * running. kthread_stop() is not needed since the kthread
 * does not make call to kthread_should_stop()
 */
static int aurix_tegra_stop_kthread(struct device *dev)
{
	struct aurix_tegra_spi_data *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned long flags;

	if (data->thread) {
		spin_lock_irqsave(&data->lock, flags);
		if (!data->exited)
			ret = send_sig(SIGINT, data->thread, 0);
		spin_unlock_irqrestore(&data->lock, flags);
		if (ret < 0) {
			dev_err(dev, "%s: Error sending SIGINT\n", __func__);
			return ret;
		}
	}
	return ret;
}

static int aurix_tegra_start_kthread(struct device *dev)
{
	struct aurix_tegra_spi_data *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned long flags;

	data->thread = kthread_run(aurix_tegra_read_thread,
		(void*) data, "aurix_tegra_kthread");
	if (IS_ERR_OR_NULL(data->thread)) {
		ret = PTR_ERR(data->thread);
		dev_err(dev, "%s: Error creating thread\n", __func__);
		return ret;
	}

	spin_lock_irqsave(&data->lock, flags);
	data->exited = false;
	spin_unlock_irqrestore(&data->lock, flags);
	return ret;
}

/*
 * remove, shutdown, suspend, resume functions
 */
static int aurix_tegra_spi_remove(struct spi_device *spi)
{
	struct aurix_tegra_spi_data *data = spi_get_drvdata(spi);
	int ret = 0;

	ret = aurix_tegra_stop_kthread(&spi->dev);
	if (data) {
		if (data->transmit) {
			devm_kfree(&spi->dev, (void*) data->transmit);
		}
		if (data->receive) {
			devm_kfree(&spi->dev, (void*) data->receive);
		}
		devm_kfree(&spi->dev, (void*) data);
	}
	return ret;
}

static void aurix_tegra_spi_shutdown(struct spi_device *spi)
{
	aurix_tegra_stop_kthread(&spi->dev);
}

static int aurix_tegra_spi_suspend(struct device *dev)
{
	return aurix_tegra_stop_kthread(dev);
}

static int aurix_tegra_spi_resume(struct device *dev)
{
	return aurix_tegra_start_kthread(dev);
}

static SIMPLE_DEV_PM_OPS(aurix_tegra_pm_ops,
			aurix_tegra_spi_suspend,
			aurix_tegra_spi_resume);

static struct spi_driver aurix_tegra_spi_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "aurix_tegra_spi",
		.of_match_table = of_match_ptr(aurix_tegra_ids),
		.pm = &aurix_tegra_pm_ops,
	},
	.probe		= aurix_tegra_spi_probe,
	.remove		= aurix_tegra_spi_remove,
	.shutdown	= aurix_tegra_spi_shutdown,
};

static int __init aurix_tegra_spi_init(void)
{
	return spi_register_driver(&aurix_tegra_spi_driver);
}

static void __exit aurix_tegra_spi_exit(void)
{
	spi_unregister_driver(&aurix_tegra_spi_driver);
}

module_init(aurix_tegra_spi_init);
module_exit(aurix_tegra_spi_exit);

MODULE_AUTHOR("Theodoros Marinakis <tmarinakis@nvidia.com>");
MODULE_DESCRIPTION("Aurix-Tegra SPI communication driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:aurix_tegra_spi");
