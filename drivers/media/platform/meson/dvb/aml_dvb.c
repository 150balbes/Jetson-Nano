// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

#define ENABLE_DEMUX_DRIVER

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/fcntl.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include "c_stb_define.h"
#include "c_stb_regs_define.h"

#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/pinctrl/consumer.h>

#include <linux/reset.h>
#include "aml_dvb.h"
#include "aml_dvb_reg.h"
#include "meson_fe.h"

#define pr_dbg(fmt, args...)\
	do {\
		if (debug_dvb)\
			printk("DVB: %s: " fmt, __func__, ## args);\
	} while (0)
#define pr_error(fmt, args...) printk("DVB: %s: " fmt, __func__, ## args)

MODULE_PARM_DESC(debug_dvb, "\n\t\t Enable dvb debug information");
static int debug_dvb;
module_param(debug_dvb, int, 0644);

#define CARD_NAME "dvb_meson"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static struct aml_dvb aml_dvb_device;
static struct reset_control *aml_dvb_demux_reset_ctl;
static struct reset_control *aml_dvb_afifo_reset_ctl;
static struct reset_control *aml_dvb_ahbarb0_reset_ctl;
static struct reset_control *aml_dvb_uparsertop_reset_ctl;
extern void dmx_reset_dmx_sw(void);

static void aml_dvb_dmx_release(struct aml_dvb *advb, struct aml_dmx *dmx)
{
	dvb_net_release(&dmx->dvb_net);
	aml_dmx_hw_deinit(dmx);
	dmx->demux.dmx.close(&dmx->demux.dmx);
	dmx->demux.dmx.remove_frontend(&dmx->demux.dmx, &dmx->mem_fe);
	dmx->demux.dmx.remove_frontend(&dmx->demux.dmx, &dmx->hw_fe[dmx->id]);
	dvb_dmxdev_release(&dmx->dmxdev);
	dvb_dmx_release(&dmx->demux);
}

static int aml_dvb_dmx_init(struct aml_dvb *advb, struct aml_dmx *dmx, int id)
{
	int ret;

	dmx->id = id;
	dmx->source  = 0;
	dmx->dump_ts_select = 0;

	dmx->demux.dmx.capabilities 	= (DMX_TS_FILTERING  | DMX_SECTION_FILTERING | DMX_MEMORY_BASED_FILTERING);
	dmx->demux.filternum 		= dmx->demux.feednum = FILTER_COUNT * 2 - 5; /* channel counts + ignore channel counts */
	dmx->demux.priv 		= advb;
	dmx->demux.start_feed 		= aml_dmx_hw_start_feed;
	dmx->demux.stop_feed 		= aml_dmx_hw_stop_feed;
	dmx->demux.write_to_decoder 	= NULL;

	if ((ret = dvb_dmx_init(&dmx->demux)) < 0) {
		pr_error("dvb_dmx failed: error %d\n",ret);
		goto error_dmx_init;
	}

	dmx->dmxdev.filternum = dmx->demux.feednum;
	dmx->dmxdev.demux = &dmx->demux.dmx;
	dmx->dmxdev.capabilities = 0;
	if ((ret = dvb_dmxdev_init(&dmx->dmxdev, &advb->dvb_adapter)) < 0) {
		pr_error("dvb_dmxdev_init failed: error %d\n",ret);
		goto error_dmxdev_init;
	}

	dmx->hw_fe[id].source = id + DMX_FRONTEND_0;

	if ((ret = dmx->demux.dmx.add_frontend(&dmx->demux.dmx, &dmx->hw_fe[id])) < 0) {
		pr_error("adding hw_frontend to dmx failed: error %d",ret);
		dmx->hw_fe[id].source = 0;
		goto error_add_hw_fe;
	}

	dmx->mem_fe.source = DMX_MEMORY_FE;
	if ((ret = dmx->demux.dmx.add_frontend(&dmx->demux.dmx, &dmx->mem_fe)) < 0) {
		pr_error("adding mem_frontend to dmx failed: error %d",ret);
		goto error_add_mem_fe;
	}

	if ((ret = dmx->demux.dmx.connect_frontend(&dmx->demux.dmx, &dmx->hw_fe[id])) < 0) {
		pr_error("connect frontend failed: error %d",ret);
		goto error_connect_fe;
	}

	dmx->aud_chan = -1;
	dmx->vid_chan = -1;
	dmx->sub_chan = -1;
	dmx->pcr_chan = -1;
	dmx->smallsec.bufsize   = SS_BUFSIZE_DEF;
	dmx->smallsec.enable    = 0;
	dmx->smallsec.dmx       = dmx;
	dmx->timeout.dmx        =  dmx;
	dmx->timeout.enable     = 1;
	dmx->timeout.timeout    = DTO_TIMEOUT_DEF;
	dmx->timeout.ch_disable = DTO_CHDIS_VAS;
	dmx->timeout.match      = 1;
	dmx->timeout.trigger    = 0;

	if ((ret = aml_dmx_hw_init(dmx)) <0) {
		pr_error("demux%d hw init error %d", id, ret);
		dmx->id = -1;
		goto error_dmx_hw_init;
	}

	dvb_net_init(&advb->dvb_adapter, &dmx->dvb_net, &dmx->demux.dmx);

	return 0;

error_dmx_hw_init:
error_connect_fe:
	dmx->demux.dmx.remove_frontend(&dmx->demux.dmx, &dmx->mem_fe);

error_add_mem_fe:
error_add_hw_fe:
	if (dmx->hw_fe[id].source)
		dmx->demux.dmx.remove_frontend(&dmx->demux.dmx, &dmx->hw_fe[id]);

	dvb_dmxdev_release(&dmx->dmxdev);

error_dmxdev_init:
	dvb_dmx_release(&dmx->demux);

error_dmx_init:
	return ret;
}
struct aml_dvb* aml_get_dvb_device(void)
{
	return &aml_dvb_device;
}


static int aml_dvb_asyncfifo_init(struct aml_dvb *advb, struct aml_asyncfifo *asyncfifo, int id)
{
	asyncfifo->dvb = advb;
	asyncfifo->id = id;

	return aml_asyncfifo_hw_init(asyncfifo, id);
}


static void aml_dvb_asyncfifo_release(struct aml_dvb *advb, struct aml_asyncfifo *asyncfifo)
{
	aml_asyncfifo_hw_deinit(asyncfifo);
}

int __init aml_dvb_init(void)
{
	struct aml_dvb *advb = &aml_dvb_device;
	struct dvb_frontend_ops *ops;
	int i, ret = 0;

	get_aml_dvb(advb);
	advb->stb_source = -1;
	advb->tso_source = -1;

	ret =
	    dvb_register_adapter(&advb->dvb_adapter, CARD_NAME, THIS_MODULE,
				 advb->dev, adapter_nr);
	if (ret < 0)
		return ret;

	advb->dvb_adapter.priv = advb;
	dev_set_drvdata(advb->dev, advb);

	for (i = 0; i < DMX_DEV_COUNT; i++) {
		ret = aml_dvb_dmx_init(advb, &advb->dmx[i], i);
		if (ret < 0)
			goto error;
	}
	/*Init the async fifos */
	for (i = 0; i < ASYNCFIFO_COUNT; i++) {
		ret = aml_dvb_asyncfifo_init(advb, &advb->asyncfifo[i], i);
		if (ret < 0)
			goto error;

		aml_asyncfifo_hw_set_source(&advb->asyncfifo[i], i == 0 ? AM_DMX_0 : AM_DMX_1);
	}

	for (i = 0; i < ASYNCFIFO_COUNT; i++) {
		if (!advb->fe[i])
			continue;

		advb->fe[i]->id = i;
		if (dvb_register_frontend(&advb->dvb_adapter, advb->fe[i])) {
			pr_err("Frontend registration failed!!!\n");
			ops = &advb->fe[i]->ops;
			if (ops->release != NULL)
				ops->release(advb->fe[i]);
			advb->fe[i] = NULL;
			goto error;
		}

		platform_set_drvdata(advb->pdev, advb->fe[i]);
	}
	dmx_reset_dmx_sw();
	pr_info("Meson DVB frontend(s) registered successfully.\n");
	return ret;
error:
	for (i = 0; i < ASYNCFIFO_COUNT; i++) {
		if (advb->asyncfifo[i].id != -1)
			aml_dvb_asyncfifo_release(advb, &advb->asyncfifo[i]);
	}

	for (i = 0; i < DMX_DEV_COUNT; i++) {
		if (advb->dmx[i].id != -1)
			aml_dvb_dmx_release(advb, &advb->dmx[i]);
	}

	dvb_unregister_adapter(&advb->dvb_adapter);

	return ret;
}

void __exit aml_dvb_exit(void)
{
	struct aml_dvb *advb = &aml_dvb_device;
	int i;

	for (i = 0; i < DMX_DEV_COUNT; i++)
		aml_dvb_dmx_release(advb, &advb->dmx[i]);

	dvb_unregister_adapter(&advb->dvb_adapter);

	for (i = 0; i < TS_IN_COUNT; i++) {
		if (advb->ts[i].pinctrl)
			devm_pinctrl_put(advb->ts[i].pinctrl);
	}

	/*switch_mod_gate_by_name("demux", 0); */
	reset_control_assert(aml_dvb_uparsertop_reset_ctl);
	reset_control_assert(aml_dvb_ahbarb0_reset_ctl);
	reset_control_assert(aml_dvb_afifo_reset_ctl);					
	reset_control_assert(aml_dvb_demux_reset_ctl);
}

module_init(aml_dvb_init);
module_exit(aml_dvb_exit);

MODULE_DESCRIPTION("driver for Meson internal DVB demods");
MODULE_AUTHOR("afl1");
MODULE_LICENSE("GPL");
