/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Aapo Vienamo	<avienamo@nvidia.com>
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

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/bpmp_abi.h>

#include "bpmp.h"

static struct bpmp_tty_buffer {
	void __iomem *bpmp_tx_buf;
	void __iomem *bpmp_tx_head;
	void __iomem *bpmp_tx_tail;
	uint32_t bpmp_tx_buf_sz;
	uint32_t bpmp_rx_space_avail;
} bpmp_tty_buffer;

struct bpmp_tty_priv {
	struct delayed_work poll_work;
	struct tty_port port;
};

static atomic_t bpmp_tty_num_readers = ATOMIC_INIT(0);
static bool write_enable;

static void bpmp_tty_poll_worker(struct work_struct *work)
{
	struct mrq_ringbuf_console_host_to_bpmp_request req;
	union mrq_ringbuf_console_bpmp_to_host_response resp;
	struct bpmp_tty_priv *priv =
		container_of(work, struct bpmp_tty_priv,
			     poll_work.work);
	int ret = 0;

	do {
		req.type = CMD_RINGBUF_CONSOLE_READ;
		req.read.len = sizeof(resp.read.data);
		ret = tegra_bpmp_send_receive(MRQ_RINGBUF_CONSOLE, &req,
					      sizeof(req), &resp, sizeof(resp));
		if (ret < 0) {
			schedule_delayed_work(&priv->poll_work,
					      msecs_to_jiffies(100));
			return;
		}
		if (resp.read.len) {
			tty_insert_flip_string(&priv->port, resp.read.data,
					       resp.read.len);
			tty_flip_buffer_push(&priv->port);
		}
	} while (resp.read.len);
	schedule_delayed_work(&priv->poll_work, msecs_to_jiffies(100));
}

static int bpmp_tty_mrq_cmd_abi_query(uint32_t cmd)
{
	struct mrq_ringbuf_console_host_to_bpmp_request req;
	union mrq_ringbuf_console_bpmp_to_host_response resp;

	req.type = CMD_RINGBUF_CONSOLE_QUERY_ABI;
	req.query_abi.cmd = cmd;
	return tegra_bpmp_send_receive(MRQ_RINGBUF_CONSOLE, &req, sizeof(req),
				       &resp, sizeof(resp));
}

static int bpmp_tty_write_msg(const unsigned char *data, uint32_t len)
{
	struct mrq_ringbuf_console_host_to_bpmp_request req;
	union mrq_ringbuf_console_bpmp_to_host_response resp;
	int ret;

	if (!write_enable)
		return -ENODEV;

	req.type = CMD_RINGBUF_CONSOLE_WRITE;
	req.write.len = min_t(size_t, len, sizeof(req.write.data));
	memcpy(req.write.data, data, req.write.len);

	ret = tegra_bpmp_send_receive(MRQ_RINGBUF_CONSOLE, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (ret)
		return ret;

	bpmp_tty_buffer.bpmp_rx_space_avail =
		resp.write.space_avail;

	return resp.write.len;
}

static int bpmp_tty_mrq_abi_probe(void)
{
	struct mrq_query_abi_request req;
	struct mrq_query_abi_response resp;
	int ret;

	req.mrq = MRQ_RINGBUF_CONSOLE;
	ret = tegra_bpmp_send_receive(MRQ_QUERY_ABI, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (ret < 0)
		return ret;
	if (resp.status == -BPMP_ENODEV)
		return -ENODEV;
	return 0;
}

static int bpmp_tty_install(struct tty_driver *drv, struct tty_struct *tty)
{
	struct bpmp_tty_priv *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	tty_port_init(&priv->port);
	tty->driver_data = priv;

	INIT_DELAYED_WORK(&priv->poll_work,
			  bpmp_tty_poll_worker);
	ret = tty_port_install(&priv->port, drv, tty);
	if (ret)
		kfree(priv);
	return ret;
}

static void bpmp_tty_cleanup(struct tty_struct *tty)
{
	struct bpmp_tty_priv *priv = tty->driver_data;

	tty->driver_data = NULL;
	tty_port_destroy(&priv->port);
	kfree(priv);
}

static int bpmp_tty_open(struct tty_struct *tty, struct file *file)
{

	struct bpmp_tty_priv *priv = tty->driver_data;

	tty_port_tty_set(&priv->port, tty);
	if (atomic_inc_return(&bpmp_tty_num_readers) == 1)
		schedule_delayed_work(&priv->poll_work, HZ);

	return 0;
}

static void bpmp_tty_close(struct tty_struct *tty, struct file *file)
{
	struct bpmp_tty_priv *priv = tty->driver_data;

	if (atomic_dec_return(&bpmp_tty_num_readers) == 0)
		cancel_delayed_work_sync(&priv->poll_work);
	tty_port_tty_set(&priv->port, NULL);
}


static int bpmp_tty_write(struct tty_struct *tty, const unsigned char *buf,
			  int count)
{
	int tx_count = 0;
	int ret;

	while (tx_count < count) {
		ret = bpmp_tty_write_msg(buf + tx_count, count - tx_count);
		if (ret < 0)
			return ret;
		tx_count += ret;
	}

	return tx_count;
}

static int bpmp_tty_write_room(struct tty_struct *tty)
{
	unsigned char dummy;
	int ret;

	if (bpmp_tty_buffer.bpmp_rx_space_avail == 0) {
		/* Update the BPMP RX buffer status. */
		ret = bpmp_tty_write_msg(&dummy, 0);
		if (ret < 0)
			return ret;
	}
	return bpmp_tty_buffer.bpmp_rx_space_avail;
}

static const struct tty_operations bpmp_tty_ops = {
	.install	= bpmp_tty_install,
	.cleanup	= bpmp_tty_cleanup,
	.write_room	= bpmp_tty_write_room,
	.open		= bpmp_tty_open,
	.close		= bpmp_tty_close,
	.write		= bpmp_tty_write,
};

static struct ktermios bpmp_tty_termios = {
	.c_iflag	= IXON,
	.c_oflag	= OPOST | CRDLY,
	.c_cflag	= B115200 | CS8 | CREAD | HUPCL,
	.c_lflag	= 0,
	.c_cc		= INIT_C_CC,
	.c_ispeed	= 115200,
	.c_ospeed	= 115200,
};

struct bpmp_tty_drvdata {
	struct dentry *debugfs_dump_entry;
	struct tty_driver *tty_driver;
};

#ifdef CONFIG_DEBUG_FS
static int bpmp_tty_debug_dump(struct seq_file *s, void *unused)
{
	struct bpmp_tty_buffer *buffer = &bpmp_tty_buffer;
	uint32_t i, len, start;
	size_t off;

	start = ioread32(buffer->bpmp_tx_tail);
	len = (ioread32(buffer->bpmp_tx_head) - start) % buffer->bpmp_tx_buf_sz;

	for (i = 0; i < len; ++i) {
		off = (start + i) % buffer->bpmp_tx_buf_sz;
		seq_putc(s, ioread8(buffer->bpmp_tx_buf + off));
	}

	return 0;
}

static int bpmp_tty_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_tty_debug_dump, inode->i_private);
}

static const struct file_operations bpmp_tty_dump_fops = {
	.open = bpmp_tty_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bpmp_tty_get_fifo(struct platform_device *pdev)
{
	struct bpmp_tty_buffer *buffer = &bpmp_tty_buffer;
	struct mrq_ringbuf_console_host_to_bpmp_request req;
	union mrq_ringbuf_console_bpmp_to_host_response resp;
	int ret;

	req.type = CMD_RINGBUF_CONSOLE_GET_FIFO;

	ret = tegra_bpmp_send_receive(MRQ_RINGBUF_CONSOLE, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (ret)
		return ret;

	buffer->bpmp_tx_buf = devm_ioremap(&pdev->dev,
					   resp.get_fifo.bpmp_tx_buf_addr,
					   resp.get_fifo.bpmp_tx_buf_len);
	if (IS_ERR(buffer->bpmp_tx_buf))
		return PTR_ERR(buffer->bpmp_tx_buf);

	buffer->bpmp_tx_head = devm_ioremap(&pdev->dev,
					    resp.get_fifo.bpmp_tx_head_addr,
					    sizeof(uint64_t));
	if (IS_ERR(buffer->bpmp_tx_head))
		return PTR_ERR(buffer->bpmp_tx_head);

	buffer->bpmp_tx_tail = devm_ioremap(&pdev->dev,
					    resp.get_fifo.bpmp_tx_tail_addr,
					    sizeof(uint64_t));
	if (IS_ERR(buffer->bpmp_tx_tail))
		return PTR_ERR(buffer->bpmp_tx_tail);

	buffer->bpmp_tx_buf_sz = resp.get_fifo.bpmp_tx_buf_len;

	return 0;
}

static void bpmp_tty_create_debugfs(struct platform_device *pdev,
		struct dentry *root)
{
	struct bpmp_tty_drvdata *drvdata = platform_get_drvdata(pdev);
	int err;

	err = bpmp_tty_mrq_cmd_abi_query(CMD_RINGBUF_CONSOLE_GET_FIFO);
	if (err)
		return;

	err = bpmp_tty_get_fifo(pdev);
	if (err)
		return;

	drvdata->debugfs_dump_entry =
		debugfs_create_file("ttyBPMP", 0444, root, NULL,
				    &bpmp_tty_dump_fops);
	if (IS_ERR_OR_NULL(drvdata->debugfs_dump_entry))
		dev_warn(&pdev->dev,
			 "failed to create a debugfs entry: %ld\n",
			 PTR_ERR(drvdata->debugfs_dump_entry));
}

static void bpmp_tty_remove_debugfs(struct bpmp_tty_drvdata *drvdata)
{
	debugfs_remove(drvdata->debugfs_dump_entry);
}
#else
static void bpmp_tty_create_debugfs(struct platform_device *pdev,
		struct dentry *root) {}
static void bpmp_tty_remove_debugfs(struct bpmp_tty_drvdata *drvdata) {}
#endif

static int bpmp_tty_probe(struct platform_device *pdev)
{
	struct bpmp_tty_drvdata *drvdata;
	struct dentry *root;
	int ret;

	ret = bpmp_tty_mrq_abi_probe();
	if (ret < 0)
		return ret;

	ret = bpmp_tty_mrq_cmd_abi_query(CMD_RINGBUF_CONSOLE_WRITE);
	if (ret == 0)
		write_enable = true;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	drvdata->tty_driver = tty_alloc_driver(1, 0);
	if (!drvdata->tty_driver)
		return -ENOMEM;

	drvdata->tty_driver->owner		= THIS_MODULE;
	drvdata->tty_driver->driver_name	= "ttyBPMP";
	drvdata->tty_driver->name		= "ttyBPMP";
	drvdata->tty_driver->major		= 0;
	drvdata->tty_driver->minor_start	= 0;
	drvdata->tty_driver->init_termios	= bpmp_tty_termios;
	drvdata->tty_driver->subtype		= SERIAL_TYPE_NORMAL;
	drvdata->tty_driver->flags		= TTY_DRIVER_REAL_RAW;
	drvdata->tty_driver->type		= TTY_DRIVER_TYPE_SERIAL;

	tty_set_operations(drvdata->tty_driver, &bpmp_tty_ops);

	ret = tty_register_driver(drvdata->tty_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed to register tty driver\n");
		goto err_put_tty_driver;
	}

	root = pdev->dev.platform_data;

	if (root)
		bpmp_tty_create_debugfs(pdev, root);

	return 0;

err_put_tty_driver:
	put_tty_driver(drvdata->tty_driver);
	return ret;
}

static int bpmp_tty_remove(struct platform_device *pdev)
{
	struct bpmp_tty_drvdata *drvdata =
		platform_get_drvdata(pdev);

	bpmp_tty_remove_debugfs(drvdata);
	tty_unregister_driver(drvdata->tty_driver);
	put_tty_driver(drvdata->tty_driver);

	return 0;
}

static struct platform_driver bpmp_tty_driver = {
	.probe = bpmp_tty_probe,
	.remove = bpmp_tty_remove,
	.driver = {
		.name = "tegra-bpmp-tty",
	},
};
module_platform_driver(bpmp_tty_driver);

MODULE_AUTHOR("Aapo Vienamo <avienamo@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra BPMP ring buffer console driver");
MODULE_LICENSE("GPL v2");
