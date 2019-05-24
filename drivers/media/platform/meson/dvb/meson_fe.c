// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2018-present Team CoreELEC (https://coreelec.org)

#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include "meson_fe.h"

#include "avl6862.h"
#include "r912.h"
#include "ascot3.h"
#include "cxd2841er_wetek.h"
#include "mxl603.h"
#include "mxl608.h"
#include "avl6211.h"
#include "mn88436.h"
#include "tuner_ftm4862.h"
#include "c_stb_regs_define.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
static struct clk *dvb_demux_clk_ctl;
static struct clk *dvb_afifo_clk_ctl;
static struct clk *dvb_ahbarb0_clk_ctl;
static struct clk *dvb_uparsertop_clk_ctl;
static struct reset_control *dvb_demux_reset_ctl;
static struct reset_control *dvb_async0_reset_ctl;
static struct reset_control *dvb_dmx0_reset_ctl;
#else
static struct reset_control *dvb_demux_reset_ctl;
static struct reset_control *dvb_afifo_reset_ctl;
static struct reset_control *dvb_ahbarb0_reset_ctl;
static struct reset_control *dvb_uparsertop_reset_ctl;
#endif

static struct aml_dvb meson_dvb;

static struct r912_config r912cfg = {
	.i2c_address = 0x7A,	
};
static struct avl6862_config avl6862cfg = {
	.demod_address = 0x14,
	.tuner_address = 0x7A,
	.ts_serial = 0,
};
static struct avl6862_config avl6762cfg = {
	.demod_address = 0x14,
	.tuner_address = 0,
	.ts_serial = 0,
};
static struct cxd2841er_config cxd2841cfg = {
		.i2c_addr = 0x6C,
		.if_agc = 0,
		.ifagc_adc_range = 0x39,
		.ts_error_polarity = 0,
		.clock_polarity = 1,
		.mxl603	= 0,
		.xtal = SONY_XTAL_20500,
};
struct ascot3_config ascot3cfg = {
		.i2c_address = 0x60,
};
static struct mxl603_config mxl603cfg = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_SELF,
		.xtal_cap = 16,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 1,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};
static struct mxl603_config mxl603cfg_atsc = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_EXTERNAL,
		.xtal_cap = 31,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 0,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};
static struct mxl603_config mxl608cfg = {
		.xtal_freq_hz = MXL603_XTAL_16MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_SELF,
		.xtal_cap = 25,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 1,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};

static struct avl6211_config avl6211cfg[] = {
	{
		.tuner_address = 0xC4,
		.tuner_i2c_clock = 200,	
		.demod_address = 0x0C,
		.mpeg_pol = 1,
		.mpeg_mode = 0,
		.mpeg_format = 0,
		.demod_refclk = 9,
		.mpeg_pin = 0,
		.tuner_rfagc = 1,
		.tuner_spectrum = 0,
		.use_lnb_pin59 = 1,
		.use_lnb_pin60 = 0,
		.set_external_vol_gpio = set_external_vol_gpio,
	},
};

static struct ftm4862_config ftm4862_config = {
	.reserved = 0,
};

void get_aml_dvb(struct aml_dvb *p)
{
	memcpy(p, &meson_dvb, sizeof(struct aml_dvb));
}
EXPORT_SYMBOL(get_aml_dvb);

int set_external_vol_gpio(int *demod_id, int on)
{ 
	int ret = 0;

	if (on)
		on = 1;

	if (demod_id && meson_dvb.power_ctrl[*demod_id]) 
		ret = gpio_direction_output(meson_dvb.power_ctrl[*demod_id], on);

	return ret;
}


void reset_demod(int i)
{
	if (meson_dvb.fec_reset[i]) {
		gpio_direction_output(meson_dvb.fec_reset[i], 0);
		msleep(600);
		gpio_direction_output(meson_dvb.fec_reset[i], 1);
		msleep(600);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
                           
int aml_read_cbus(unsigned int reg)
{	
	int val;
	val = readl(meson_dvb.demux_base + ((reg - STB_VERSION) << 2));
	return val;
}
void aml_write_cbus(unsigned int reg, unsigned int val)
{
	writel(val, meson_dvb.demux_base + ((reg - STB_VERSION) << 2));
}
int aml_read_vcbus(unsigned int reg)
{	
	int val;
	val = readl(meson_dvb.afifo_base + ((reg - ASYNC_FIFO_REG0) << 2));
	return val;
}
void aml_write_vcbus(unsigned int reg, unsigned int val)
{
	writel(val, meson_dvb.afifo_base + ((reg - ASYNC_FIFO_REG0) << 2));
}
EXPORT_SYMBOL(aml_read_cbus);
EXPORT_SYMBOL(aml_write_cbus);
EXPORT_SYMBOL(aml_read_vcbus);
EXPORT_SYMBOL(aml_write_vcbus);
#endif

static int fe_dvb_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;
        int i2c[2] = {1, -1};
	const char *str;
	char buf[32];
	int s2p_id = 0;
	int value;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	struct resource *r;
#endif
        struct device_node *np;
	meson_dvb.pdev = pdev;
	meson_dvb.dev  = &pdev->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "stbtop");
	meson_dvb.demux_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(meson_dvb.demux_base)) {
		dev_err(&pdev->dev, "Couldn't remap STBTOP memory\n");
		return PTR_ERR(meson_dvb.demux_base);
	}
 
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "afifo");
	meson_dvb.afifo_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(meson_dvb.afifo_base)) {
		dev_err(&pdev->dev, "Couldn't remap AFIFO memory\n");
		return PTR_ERR(meson_dvb.afifo_base);
	}
	np = pdev->dev.of_node;
        i2c[0] = 0;
#else
        np = of_find_node_by_name(NULL, "dvbfe");
#endif
	meson_dvb.s2p[0].invert = 0;
	meson_dvb.s2p[1].invert = 0;
	meson_dvb.demux_irq[2] = -1;
	for (i = 0; i < TS_IN_COUNT; i++) {
		meson_dvb.ts[i].mode   = AM_TS_DISABLE;
		meson_dvb.ts[i].s2p_id = -1;
		meson_dvb.ts[i].control = 0;
		meson_dvb.fec_reset[i] = 0;
		meson_dvb.power_ctrl[i] = 0;
		meson_dvb.lock_led[i] = 0;
		meson_dvb.demux_irq[i] = -1;
		meson_dvb.afifo_irq[i] = -1;
		meson_dvb.i2c[i] = NULL;

		if (np) {
			snprintf(buf, sizeof(buf), "dtv_demod%d_i2c_adap_id", i);
			if (of_property_read_u32(np, buf, &i2c[i]) || i2c[i] < 0)
				continue;
		}
		if (i2c[i] < 0)
			continue;

		meson_dvb.i2c[i] = i2c_get_adapter(i2c[i]);
		if (meson_dvb.i2c[i] == NULL)
			continue;

		dev_info(&pdev->dev, "Found i2c-%d adapter: %s\n", i2c[i] , meson_dvb.i2c[i]->name);

		snprintf(buf, sizeof(buf), "ts%d", i);
		ret = of_property_read_string(pdev->dev.of_node, buf, &str);
		if (ret) {
			dev_info(&pdev->dev, "No %s config...\n", buf);
			meson_dvb.i2c[i] = NULL;
			continue;
		}
		if (!strcmp(str, "parallel")) {
			dev_info(&pdev->dev, "%s: parallel\n", buf);
			snprintf(buf, sizeof(buf), "p_ts%d", i);
			meson_dvb.ts[i].mode    = AM_TS_PARALLEL;
			meson_dvb.ts[i].pinctrl = devm_pinctrl_get_select(&pdev->dev, buf);
		}
		else if (!strcmp(str, "serial")) {
			dev_info(&pdev->dev, "%s: serial s2p%d\n", buf, s2p_id);
			snprintf(buf, sizeof(buf), "s_ts%d", i);
			meson_dvb.ts[i].mode    = AM_TS_SERIAL;
			meson_dvb.ts[i].pinctrl = devm_pinctrl_get_select(&pdev->dev, buf);
			meson_dvb.ts[i].s2p_id = s2p_id;
			s2p_id++;
		}
		snprintf(buf, sizeof(buf), "ts%d_control", i);
		ret = of_property_read_u32(pdev->dev.of_node, buf, &value);
		if(!ret){
			dev_info(&pdev->dev, "%s: 0x%x\n", buf, value);
			meson_dvb.ts[i].control = value;
		}
		if(meson_dvb.ts[i].s2p_id != -1){
			snprintf(buf, sizeof(buf), "ts%d_invert", i);
			ret = of_property_read_u32(pdev->dev.of_node, buf, &value);
			if(!ret){
				dev_info(&pdev->dev, "%s: 0x%x\n", buf, value);
				meson_dvb.s2p[meson_dvb.ts[i].s2p_id].invert = value;
			}
		}
		meson_dvb.xtal[i] = MXL603_XTAL_16MHz;
		snprintf(buf, sizeof(buf), "tuner%d_xtal", i);
		ret = of_property_read_u32(pdev->dev.of_node, buf, &value);
		if(!ret){
			if (value == 24)
				meson_dvb.xtal[i] = MXL603_XTAL_24MHz;
				dev_info(&pdev->dev, "%s: %d MHz\n", buf, meson_dvb.xtal[i] == MXL603_XTAL_24MHz ? 24 : 16);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
		snprintf(buf, sizeof(buf), "demux%d", i);
		meson_dvb.demux_irq[i] = platform_get_irq_byname(pdev, buf);
		if (meson_dvb.demux_irq[i] < 0) {
			dev_err(&pdev->dev, "can't find IRQ for demux%d\n", i);
			return meson_dvb.demux_irq[i];
		}
		snprintf(buf, sizeof(buf), "asyncfifo%d", i);
		meson_dvb.afifo_irq[i] = platform_get_irq_byname(pdev, buf);
		if (meson_dvb.afifo_irq[i] < 0) {
			dev_err(&pdev->dev, "can't find IRQ for asyncfifo%d\n", i);
			return meson_dvb.afifo_irq[i];
		}
#else
		if (i == 0) {
			meson_dvb.demux_irq[i] = INT_DEMUX;
			meson_dvb.afifo_irq[i] = INT_ASYNC_FIFO_FLUSH;
		} else {
			meson_dvb.demux_irq[i] = INT_DEMUX_1;
			meson_dvb.afifo_irq[i] = INT_ASYNC_FIFO2_FLUSH;
		}
#endif
		if (i == 0) {
			meson_dvb.fec_reset[0] = of_get_named_gpio_flags(pdev->dev.of_node, "fec_reset_gpio-gpios", 0, NULL);
			meson_dvb.power_ctrl[0] = of_get_named_gpio_flags(pdev->dev.of_node, "power_ctrl_gpio-gpios", 0, NULL);
			meson_dvb.lock_led[0] = of_get_named_gpio_flags(pdev->dev.of_node, "lock_led_gpio-gpios", 0, NULL);

		} else {
			meson_dvb.fec_reset[1] = of_get_named_gpio_flags(pdev->dev.of_node, "fec_reset_gpio-gpios2", 0, NULL);
			meson_dvb.power_ctrl[1] = of_get_named_gpio_flags(pdev->dev.of_node, "power_ctrl_gpio-gpios2", 0, NULL);
			meson_dvb.lock_led[1] = of_get_named_gpio_flags(pdev->dev.of_node, "lock_led_gpio-gpios2", 0, NULL);
		}
		if (meson_dvb.fec_reset[i] > 0) {
			dev_dbg(&pdev->dev, "GPIO fec_reset%d: %d\n", i, meson_dvb.fec_reset[i]);
			gpio_request(meson_dvb.fec_reset[i], KBUILD_MODNAME);
		} else
			meson_dvb.fec_reset[i] = 0;
		if (meson_dvb.power_ctrl[i] > 0) {
			dev_dbg(&pdev->dev, "GPIO power_ctrl%d: %d\n", i, meson_dvb.power_ctrl[i]);
			gpio_request(meson_dvb.power_ctrl[i], KBUILD_MODNAME);
		} else
			meson_dvb.power_ctrl[i] = 0;
		if (meson_dvb.lock_led[i] > 0) {
			dev_dbg(&pdev->dev, "GPIO lock_led%d: %d\n", i, meson_dvb.lock_led[i]);
			gpio_request(meson_dvb.lock_led[i], KBUILD_MODNAME);
		} else
			meson_dvb.lock_led[i] = 0;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	dvb_demux_clk_ctl = devm_clk_get(&pdev->dev, "demux");
	dev_dbg(&pdev->dev, "dmx clk ctl = %p\n", dvb_demux_clk_ctl);
	clk_prepare_enable(dvb_demux_clk_ctl);

	dvb_afifo_clk_ctl = devm_clk_get(&pdev->dev, "asyncfifo");
	dev_dbg(&pdev->dev, "asyncfifo clk ctl = %p\n", dvb_afifo_clk_ctl);
	clk_prepare_enable(dvb_afifo_clk_ctl);
	
	dvb_ahbarb0_clk_ctl = devm_clk_get(&pdev->dev, "ahbarb0");
	dev_dbg(&pdev->dev, "ahbarb0 clk ctl = %p\n", dvb_ahbarb0_clk_ctl);
	clk_prepare_enable(dvb_ahbarb0_clk_ctl);
	
	dvb_uparsertop_clk_ctl = devm_clk_get(&pdev->dev, "uparsertop");
	dev_dbg(&pdev->dev, "uparsertop clk ctl = %p\n", dvb_uparsertop_clk_ctl);
	clk_prepare_enable(dvb_uparsertop_clk_ctl);

	dvb_demux_reset_ctl = devm_reset_control_get(&pdev->dev, "demux");
	dev_dbg(&pdev->dev, "dmx rst ctl = %p\n", dvb_demux_reset_ctl);
	reset_control_deassert(dvb_demux_reset_ctl);

	dvb_async0_reset_ctl = devm_reset_control_get(&pdev->dev, "async0");
	dev_dbg(&pdev->dev, "async0 rst ctl = %p\n", dvb_async0_reset_ctl);
	reset_control_deassert(dvb_async0_reset_ctl);
	
	dvb_dmx0_reset_ctl = devm_reset_control_get(&pdev->dev, "demuxreset0");
	dev_dbg(&pdev->dev, "dmx0 rst ctl = %p\n", dvb_dmx0_reset_ctl);
	reset_control_deassert(dvb_dmx0_reset_ctl);
#else
	dvb_demux_reset_ctl = devm_reset_control_get(&pdev->dev, "demux");
	dev_dbg(&pdev->dev, "dmx rst ctl = %p\n", dvb_demux_reset_ctl);
	reset_control_deassert(dvb_demux_reset_ctl);

	dvb_afifo_reset_ctl = devm_reset_control_get(&pdev->dev, "asyncfifo");
	dev_dbg(&pdev->dev, "asyncfifo rst ctl = %p\n", dvb_afifo_reset_ctl);
	reset_control_deassert(dvb_afifo_reset_ctl);
	
	dvb_ahbarb0_reset_ctl = devm_reset_control_get(&pdev->dev, "ahbarb0");
	dev_dbg(&pdev->dev, "ahbarb0 rst ctl = %p\n", dvb_ahbarb0_reset_ctl);
	reset_control_deassert(dvb_ahbarb0_reset_ctl);
	
	dvb_uparsertop_reset_ctl = devm_reset_control_get(&pdev->dev, "uparsertop");
	dev_dbg(&pdev->dev, "uparsertop rst ctl = %p\n", dvb_uparsertop_reset_ctl);
	reset_control_deassert(dvb_uparsertop_reset_ctl);
#endif
	ret = of_property_read_string(pdev->dev.of_node, "dev_name", &str);
	if (!ret)
		dev_info(&pdev->dev, "dev_name=%s\n", str);

	for (i = 0; i <  TS_IN_COUNT; i++) {

		meson_dvb.fe[i] = NULL;
		if (meson_dvb.i2c[i] == NULL)
			continue;

		reset_demod(i);
		dev_info(&pdev->dev, "DVB demod detection for i2c-%d (%s)...\n", i2c[i], meson_dvb.i2c[i]->name);

		if (strcmp(str,"wetek-dvb")) {
			set_external_vol_gpio(&i, 1);
			reset_demod(i);
			if (strcmp(str,"magicsee")) {		/* MeCool, Khadas */
				if (strcmp(str,"avl6762")) {
					dev_info(&pdev->dev, "Checking for Availink AVL6862 DVB-S2/T2/C demod ...\n");
					avl6862cfg.ts_serial = meson_dvb.ts[i].mode  == AM_TS_SERIAL ? 1 : 0;
					avl6862cfg.gpio_lock_led = meson_dvb.lock_led[i];
					meson_dvb.fe[i] = avl6862_attach(&avl6862cfg, meson_dvb.i2c[i]);
					if (meson_dvb.fe[i] == NULL) {
						dev_info(&pdev->dev, "Failed to find AVL6862 demod!\n");
						continue;
					}
					if (r912_attach(meson_dvb.fe[i], &r912cfg, meson_dvb.i2c[i]) == NULL) {
						dev_info(&pdev->dev, "Failed to find Rafael R912 tuner!\n");
						dev_info(&pdev->dev, "Detaching Availink AVL6862 frontend!\n");
						dvb_frontend_detach(meson_dvb.fe[i]);
						continue;
					}
					meson_dvb.total_nims++;
					continue;
				}
				dev_info(&pdev->dev, "Checking for Availink AVL6762 DVB-T2/C demod ...\n");
				avl6862cfg.ts_serial = meson_dvb.ts[i].mode  == AM_TS_SERIAL ? 1 : 0;
				meson_dvb.fe[i] = avl6862_attach(&avl6762cfg, meson_dvb.i2c[i]);
				if (meson_dvb.fe[i] == NULL) {
					dev_info(&pdev->dev, "Failed to find AVL6762 demod!\n");
					continue;
				}
				mxl608cfg.xtal_freq_hz = meson_dvb.xtal[i];
				if (mxl603_attach(meson_dvb.fe[i], meson_dvb.i2c[i], 0x60, &mxl608cfg) == NULL) {
					dev_info(&pdev->dev, "Failed to find MxL 608 tuner!\n");
					dev_info(&pdev->dev, "Detaching Availink AVL6762 frontend!\n");
					dvb_frontend_detach(meson_dvb.fe[i]);
					continue;
				}
				meson_dvb.total_nims++;
				continue;
			} else {  				/* MagicSee */
				dev_info(&pdev->dev, "Checking for Availink AVL6862 DVB-S2/T2/C demod ...\n");
				avl6862cfg.ts_serial = meson_dvb.ts[i].mode  == AM_TS_SERIAL ? 1 : 0;
				meson_dvb.fe[i] = avl6862_attach(&avl6862cfg, meson_dvb.i2c[i]);
				if (meson_dvb.fe[i] == NULL) {
					dev_info(&pdev->dev, "Failed to find AVL6862 demod!\n");
					continue;
				}
				if (ftm4862_attach(meson_dvb.fe[i], &ftm4862_config, meson_dvb.i2c[i]) == NULL) {
					dev_info(&pdev->dev, "Failed to find FTM4862 dual tuner!\n");
					dev_info(&pdev->dev, "Detaching Availink AVL6862 frontend!\n");
					dvb_frontend_detach(meson_dvb.fe[i]);
					continue;
				}
				meson_dvb.total_nims++;
				continue;
			}
		}					/* WeTek */
		reset_demod(i);
		meson_dvb.fe[i] = NULL;
		dev_info(&pdev->dev, "Checking for AVL6211 DVB-S/S2 demod ...\n");
		meson_dvb.fe[i] = avl6211_attach( meson_dvb.i2c[i], &avl6211cfg[0], 0);
		if (meson_dvb.fe[i] == NULL) {
			dev_info(&pdev->dev, "Failed to find AVL6211 demod!\n");
			goto panasonic;
		}
		meson_dvb.total_nims++;
		continue;
panasonic:
		reset_demod(i);
		meson_dvb.fe[i] = NULL;
		dev_info(&pdev->dev, "Checking for Panasonic MN88436 ATSC demod ...\n");	
			
		meson_dvb.fe[i] =  mn88436_attach(meson_dvb.i2c[i], 0);

		if (meson_dvb.fe[i] != NULL) {
												
			if (mxl603_attach(meson_dvb.fe[i], meson_dvb.i2c[i], 0x60, &mxl603cfg_atsc) == NULL) {
				dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
				dev_info(&pdev->dev, "Detaching Panasonic MN88436 ATSC frontend!\n");
				dvb_frontend_detach(meson_dvb.fe[i]);
				goto sony;
			}
				
			meson_dvb.total_nims++;
			continue;
		}
sony:
		reset_demod(i);
		meson_dvb.fe[i] = NULL;
		dev_info(&pdev->dev, "Checking for Sony CXD2841ER DVB-C/T/T2 demod ...\n");

		meson_dvb.fe[i] =  cxd2841er_attach_wetek(&cxd2841cfg,meson_dvb.i2c[i]);

		if (meson_dvb.fe[i] != NULL) {
			if (mxl603_attach(meson_dvb.fe[i], meson_dvb.i2c[i], 0x60, &mxl603cfg) == NULL) {
				dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
				cxd2841cfg.if_agc = 1;
				cxd2841cfg.ifagc_adc_range = 0x50;
				if (ascot3_attach(meson_dvb.fe[i], &ascot3cfg, meson_dvb.i2c[i]) == NULL) {
					dev_info(&pdev->dev, "Failed to find Sony ASCOT3 tuner!\n");
					dev_info(&pdev->dev, "Detaching Sony CXD2841ER DVB-C/T/T2 frontend!\n");
					dvb_frontend_detach(meson_dvb.fe[i]);
					continue;
				}
			}
			meson_dvb.total_nims++;
			continue;
		}
		i2c_put_adapter(meson_dvb.i2c[i]);
	        meson_dvb.i2c[i] = NULL;
	}
	dev_info(&pdev->dev, "Total DVB modules found: %d\n", meson_dvb.total_nims);
	
	return 0;
}
static int meson_dvb_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i <  TS_IN_COUNT; i++) {
		if (meson_dvb.i2c[i] == NULL)
			continue;

		if (meson_dvb.fe[i] != NULL)
			dvb_frontend_detach(meson_dvb.fe[i]);

		i2c_put_adapter(meson_dvb.i2c[i]);
		gpio_free(meson_dvb.fec_reset[i]);
		gpio_free(meson_dvb.power_ctrl[i]);
		devm_pinctrl_put(meson_dvb.ts[i].pinctrl);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	clk_disable_unprepare(dvb_uparsertop_clk_ctl);
	clk_disable_unprepare(dvb_ahbarb0_clk_ctl);
	clk_disable_unprepare(dvb_afifo_clk_ctl);
	clk_disable_unprepare(dvb_demux_clk_ctl);
#else
	reset_control_assert(dvb_uparsertop_reset_ctl);
	reset_control_assert(dvb_ahbarb0_reset_ctl);
	reset_control_assert(dvb_afifo_reset_ctl);
	reset_control_assert(dvb_demux_reset_ctl);
#endif
	return 0;
}
static const struct of_device_id meson_dvb_dt_match[] = {
	{
		.compatible = "amlogic,dvb",
	},
	{},
};
static struct platform_driver meson_dvb_detection = {
	.probe		= fe_dvb_probe,
	.remove		= meson_dvb_remove,
	.driver		= {
		.name	= "dvb_meson",
		.owner	= THIS_MODULE,
		.of_match_table = meson_dvb_dt_match,
	}
};

int __init meson_dvb_init(void)
{
	int ret;
	
	memset(&meson_dvb, 0, sizeof(struct aml_dvb));
	
	ret = platform_driver_register(&meson_dvb_detection);
	return ret;
}
void __exit meson_dvb_exit(void)
{
	platform_driver_unregister(&meson_dvb_detection);
}

module_init(meson_dvb_init);
module_exit(meson_dvb_exit);

MODULE_DESCRIPTION("Meson DVB demods detection");
MODULE_AUTHOR("afl1 <afl2001@gmail.com>");
MODULE_LICENSE("GPL");
