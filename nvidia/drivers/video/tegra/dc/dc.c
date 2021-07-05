/*
 * dc.c: dc driver.
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (c) 2010-2020, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/tegra-pm.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/nvhost.h>
#include <linux/clk/tegra.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <uapi/video/tegrafb.h>
#include <drm/drm_fixed.h>
#include <linux/dma-buf.h>
#include <linux/extcon/extcon-disp.h>
#include <linux/extcon.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/tegra_pm_domains.h>
#include <linux/uaccess.h>
#if defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_TRUSTY)
#include <linux/ote_protocol.h>
#endif
#include <linux/version.h>

#include <clocksource/arm_arch_timer.h>

#define CREATE_TRACE_POINTS
#include <trace/events/display.h>
EXPORT_TRACEPOINT_SYMBOL(display_writel);
EXPORT_TRACEPOINT_SYMBOL(display_readl);

#include <linux/nvhost.h>
#include <uapi/linux/nvhost_ioctl.h>

#include <linux/platform/tegra/latency_allowance.h>
#include <linux/platform/tegra/mc.h>
#include <soc/tegra/tegra_bpmp.h>
#include <uapi/video/tegra_dc_ext.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dc_shared_isr.h"
#include "nvhost_sync.h"
#include "nvhost_syncpt.h"	/* Preset and flush vblank_syncpt*/
#include "dpaux.h"
#include "lvds.h"
#include "dc_common.h"

#include "edid.h"

#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
#include "fake_panel.h"
#endif /*CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT*/


/* HACK! This needs to come from DT */
#include "../../../../arch/arm/mach-tegra/iomap.h"

#define TEGRA_CRC_LATCHED_DELAY		34

#define DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL	0x01000000
#define DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL	0x0

#define MAX_VRR_V_FRONT_PORCH			0x1000

static struct tegra_dc_hw_data *hw_data;
static struct tegra_dc_hw_data t21x_hw_data;
static struct tegra_dc_hw_data t18x_hw_data;
static struct tegra_dc_hw_data t19x_hw_data;

static const struct of_device_id tegra_display_of_match[] = {
	{.compatible = "nvidia,tegra210-dc", .data = &t21x_hw_data },
	{.compatible = "nvidia,tegra186-dc", .data = &t18x_hw_data },
	{.compatible = "nvidia,tegra194-dc", .data = &t19x_hw_data },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_display_of_match);

/* Used only on T21x*/
static struct of_device_id tegra_disa_pd[] = {
	{ .compatible = "nvidia,tegra210-disa-pd", },
	{},
};

/* Used only on T21x */
static struct of_device_id tegra_disb_pd[] = {
	{ .compatible = "nvidia,tegra210-disb-pd", },
	{},
};

static unsigned int display_la_emc_client_id[] = {
	TEGRA_BWMGR_CLIENT_DISP1_LA_EMC,
	TEGRA_BWMGR_CLIENT_DISP2_LA_EMC
};

struct fb_videomode tegra_dc_vga_mode = {
	.refresh = 60,
	.xres = 640,
	.yres = 480,
	.pixclock = KHZ2PICOS(25200),
	.hsync_len = 96,	/* h_sync_width */
	.vsync_len = 2,		/* v_sync_width */
	.left_margin = 48,	/* h_back_porch */
	.upper_margin = 33,	/* v_back_porch */
	.right_margin = 16,	/* h_front_porch */
	.lower_margin = 10,	/* v_front_porch */
	.vmode = 0,
	.sync = 0,
};

/* needs to be big enough to be index by largest supported out->type */
static struct tegra_dc_mode override_disp_mode[TEGRA_DC_OUT_NULL + 1];

static void _tegra_dc_controller_disable(struct tegra_dc *dc);
static void tegra_dc_disable_irq_ops(struct tegra_dc *dc, bool from_irq);
static int _tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable);

static int tegra_dc_set_out(struct tegra_dc *dc, struct tegra_dc_out *out,
		bool initialized);
#ifdef PM
static int tegra_dc_suspend(struct platform_device *ndev, pm_message_t state);
static int tegra_dc_resume(struct platform_device *ndev);
#endif

static struct tegra_dc **tegra_dcs;

/* Used only on Nvdisplay */
static struct tegra_dc_win	tegra_dc_windows[DC_N_WINDOWS];

static u64 tegra_dc_get_scanline_timestamp(struct tegra_dc *dc,
						const u32 scanline);

static void tegra_dc_collect_latency_data(struct tegra_dc *dc);

static DEFINE_MUTEX(tegra_dc_lock);
/* Lock to serialize extcon switch reporting across heads*/
static DEFINE_MUTEX(tegra_dc_extcon_lock);
/* Lock to serialize dc registration during probe */
static DEFINE_MUTEX(tegra_dc_registration_lock);

static struct device_dma_parameters tegra_dc_dma_parameters = {
	.max_segment_size = UINT_MAX,
};

static const struct {
	bool h;
	bool v;
} can_filter[] = {
	/* Window A has no filtering */
	{ false, false },
	/* Window B has both H and V filtering */
	{ true,  true  },
	/* Window C has only H filtering */
	{ false, true  },
};

static struct tegra_dc_cmu default_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0,    1,    2,    4,    5,    6,    7,    9,
		10,   11,   12,   14,   15,   16,   18,   20,
		21,   23,   25,   27,   29,   31,   33,   35,
		37,   40,   42,   45,   48,   50,   53,   56,
		59,   62,   66,   69,   72,   76,   79,   83,
		87,   91,   95,   99,   103,  107,  112,  116,
		121,  126,  131,  136,  141,  146,  151,  156,
		162,  168,  173,  179,  185,  191,  197,  204,
		210,  216,  223,  230,  237,  244,  251,  258,
		265,  273,  280,  288,  296,  304,  312,  320,
		329,  337,  346,  354,  363,  372,  381,  390,
		400,  409,  419,  428,  438,  448,  458,  469,
		479,  490,  500,  511,  522,  533,  544,  555,
		567,  578,  590,  602,  614,  626,  639,  651,
		664,  676,  689,  702,  715,  728,  742,  755,
		769,  783,  797,  811,  825,  840,  854,  869,
		884,  899,  914,  929,  945,  960,  976,  992,
		1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
		1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
		1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
		1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
		1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
		1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
		1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
		2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
		2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
		2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
		2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
		3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
		3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
		3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
		3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0x100, 0x0,   0x0,
		0x0,   0x100, 0x0,
		0x0,   0x0,   0x100,
	},
	/* lut2 maps linear space to sRGB*/
	{
		0,    1,    2,    2,    3,    4,    5,    6,
		6,    7,    8,    9,    10,   10,   11,   12,
		13,   13,   14,   15,   15,   16,   16,   17,
		18,   18,   19,   19,   20,   20,   21,   21,
		22,   22,   23,   23,   23,   24,   24,   25,
		25,   25,   26,   26,   27,   27,   27,   28,
		28,   29,   29,   29,   30,   30,   30,   31,
		31,   31,   32,   32,   32,   33,   33,   33,
		34,   34,   34,   34,   35,   35,   35,   36,
		36,   36,   37,   37,   37,   37,   38,   38,
		38,   38,   39,   39,   39,   40,   40,   40,
		40,   41,   41,   41,   41,   42,   42,   42,
		42,   43,   43,   43,   43,   43,   44,   44,
		44,   44,   45,   45,   45,   45,   46,   46,
		46,   46,   46,   47,   47,   47,   47,   48,
		48,   48,   48,   48,   49,   49,   49,   49,
		49,   50,   50,   50,   50,   50,   51,   51,
		51,   51,   51,   52,   52,   52,   52,   52,
		53,   53,   53,   53,   53,   54,   54,   54,
		54,   54,   55,   55,   55,   55,   55,   55,
		56,   56,   56,   56,   56,   57,   57,   57,
		57,   57,   57,   58,   58,   58,   58,   58,
		58,   59,   59,   59,   59,   59,   59,   60,
		60,   60,   60,   60,   60,   61,   61,   61,
		61,   61,   61,   62,   62,   62,   62,   62,
		62,   63,   63,   63,   63,   63,   63,   64,
		64,   64,   64,   64,   64,   64,   65,   65,
		65,   65,   65,   65,   66,   66,   66,   66,
		66,   66,   66,   67,   67,   67,   67,   67,
		67,   67,   68,   68,   68,   68,   68,   68,
		68,   69,   69,   69,   69,   69,   69,   69,
		70,   70,   70,   70,   70,   70,   70,   71,
		71,   71,   71,   71,   71,   71,   72,   72,
		72,   72,   72,   72,   72,   72,   73,   73,
		73,   73,   73,   73,   73,   74,   74,   74,
		74,   74,   74,   74,   74,   75,   75,   75,
		75,   75,   75,   75,   75,   76,   76,   76,
		76,   76,   76,   76,   77,   77,   77,   77,
		77,   77,   77,   77,   78,   78,   78,   78,
		78,   78,   78,   78,   78,   79,   79,   79,
		79,   79,   79,   79,   79,   80,   80,   80,
		80,   80,   80,   80,   80,   81,   81,   81,
		81,   81,   81,   81,   81,   81,   82,   82,
		82,   82,   82,   82,   82,   82,   83,   83,
		83,   83,   83,   83,   83,   83,   83,   84,
		84,   84,   84,   84,   84,   84,   84,   84,
		85,   85,   85,   85,   85,   85,   85,   85,
		85,   86,   86,   86,   86,   86,   86,   86,
		86,   86,   87,   87,   87,   87,   87,   87,
		87,   87,   87,   88,   88,   88,   88,   88,
		88,   88,   88,   88,   88,   89,   89,   89,
		89,   89,   89,   89,   89,   89,   90,   90,
		90,   90,   90,   90,   90,   90,   90,   90,
		91,   91,   91,   91,   91,   91,   91,   91,
		91,   91,   92,   92,   92,   92,   92,   92,
		92,   92,   92,   92,   93,   93,   93,   93,
		93,   93,   93,   93,   93,   93,   94,   94,
		94,   94,   94,   94,   94,   94,   94,   94,
		95,   95,   95,   95,   95,   95,   95,   95,
		95,   95,   96,   96,   96,   96,   96,   96,
		96,   96,   96,   96,   96,   97,   97,   97,
		97,   97,   97,   97,   97,   97,   97,   98,
		98,   98,   98,   98,   98,   98,   98,   98,
		98,   98,   99,   99,   99,   99,   99,   99,
		99,   100,  101,  101,  102,  103,  103,  104,
		105,  105,  106,  107,  107,  108,  109,  109,
		110,  111,  111,  112,  113,  113,  114,  115,
		115,  116,  116,  117,  118,  118,  119,  119,
		120,  120,  121,  122,  122,  123,  123,  124,
		124,  125,  126,  126,  127,  127,  128,  128,
		129,  129,  130,  130,  131,  131,  132,  132,
		133,  133,  134,  134,  135,  135,  136,  136,
		137,  137,  138,  138,  139,  139,  140,  140,
		141,  141,  142,  142,  143,  143,  144,  144,
		145,  145,  145,  146,  146,  147,  147,  148,
		148,  149,  149,  150,  150,  150,  151,  151,
		152,  152,  153,  153,  153,  154,  154,  155,
		155,  156,  156,  156,  157,  157,  158,  158,
		158,  159,  159,  160,  160,  160,  161,  161,
		162,  162,  162,  163,  163,  164,  164,  164,
		165,  165,  166,  166,  166,  167,  167,  167,
		168,  168,  169,  169,  169,  170,  170,  170,
		171,  171,  172,  172,  172,  173,  173,  173,
		174,  174,  174,  175,  175,  176,  176,  176,
		177,  177,  177,  178,  178,  178,  179,  179,
		179,  180,  180,  180,  181,  181,  182,  182,
		182,  183,  183,  183,  184,  184,  184,  185,
		185,  185,  186,  186,  186,  187,  187,  187,
		188,  188,  188,  189,  189,  189,  189,  190,
		190,  190,  191,  191,  191,  192,  192,  192,
		193,  193,  193,  194,  194,  194,  195,  195,
		195,  196,  196,  196,  196,  197,  197,  197,
		198,  198,  198,  199,  199,  199,  200,  200,
		200,  200,  201,  201,  201,  202,  202,  202,
		202,  203,  203,  203,  204,  204,  204,  205,
		205,  205,  205,  206,  206,  206,  207,  207,
		207,  207,  208,  208,  208,  209,  209,  209,
		209,  210,  210,  210,  211,  211,  211,  211,
		212,  212,  212,  213,  213,  213,  213,  214,
		214,  214,  214,  215,  215,  215,  216,  216,
		216,  216,  217,  217,  217,  217,  218,  218,
		218,  219,  219,  219,  219,  220,  220,  220,
		220,  221,  221,  221,  221,  222,  222,  222,
		223,  223,  223,  223,  224,  224,  224,  224,
		225,  225,  225,  225,  226,  226,  226,  226,
		227,  227,  227,  227,  228,  228,  228,  228,
		229,  229,  229,  229,  230,  230,  230,  230,
		231,  231,  231,  231,  232,  232,  232,  232,
		233,  233,  233,  233,  234,  234,  234,  234,
		235,  235,  235,  235,  236,  236,  236,  236,
		237,  237,  237,  237,  238,  238,  238,  238,
		239,  239,  239,  239,  240,  240,  240,  240,
		240,  241,  241,  241,  241,  242,  242,  242,
		242,  243,  243,  243,  243,  244,  244,  244,
		244,  244,  245,  245,  245,  245,  246,  246,
		246,  246,  247,  247,  247,  247,  247,  248,
		248,  248,  248,  249,  249,  249,  249,  249,
		250,  250,  250,  250,  251,  251,  251,  251,
		251,  252,  252,  252,  252,  253,  253,  253,
		253,  253,  254,  254,  254,  254,  255,  255,
	},
};

static struct tegra_dc_cmu default_limited_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0,    1,    2,    4,    5,    6,    7,    9,
		10,   11,   12,   14,   15,   16,   18,   20,
		21,   23,   25,   27,   29,   31,   33,   35,
		37,   40,   42,   45,   48,   50,   53,   56,
		59,   62,   66,   69,   72,   76,   79,   83,
		87,   91,   95,   99,   103,  107,  112,  116,
		121,  126,  131,  136,  141,  146,  151,  156,
		162,  168,  173,  179,  185,  191,  197,  204,
		210,  216,  223,  230,  237,  244,  251,  258,
		265,  273,  280,  288,  296,  304,  312,  320,
		329,  337,  346,  354,  363,  372,  381,  390,
		400,  409,  419,  428,  438,  448,  458,  469,
		479,  490,  500,  511,  522,  533,  544,  555,
		567,  578,  590,  602,  614,  626,  639,  651,
		664,  676,  689,  702,  715,  728,  742,  755,
		769,  783,  797,  811,  825,  840,  854,  869,
		884,  899,  914,  929,  945,  960,  976,  992,
		1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
		1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
		1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
		1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
		1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
		1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
		1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
		2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
		2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
		2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
		2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
		3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
		3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
		3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
		3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0x100, 0x000, 0x000,
		0x000, 0x100, 0x000,
		0x000, 0x000, 0x100,
	},
	/*
	 * lut2 maps linear space back to sRGB, where
	 * the output range is [16...235] (limited).
	 */
	{
		16,  17,  18,  18,  19,  19,  20,  21,
		21,  22,  23,  24,  25,  25,  25,  26,
		27,  27,  28,  28,  29,  30,  30,  31,
		31,  31,  31,  32,  32,  33,  33,  34,
		34,  35,  35,  36,  36,  37,  37,  37,
		37,  37,  38,  38,  38,  39,  39,  39,
		40,  40,  41,  41,  41,  42,  42,  42,
		43,  43,  43,  43,  43,  43,  44,  44,
		44,  44,  45,  45,  45,  46,  46,  46,
		47,  47,  47,  47,  48,  48,  48,  49,
		49,  49,  49,  49,  49,  49,  49,  50,
		50,  50,  50,  51,  51,  51,  51,  52,
		52,  52,  52,  53,  53,  53,  53,  54,
		54,  54,  54,  55,  55,  55,  55,  55,
		56,  56,  56,  56,  56,  56,  56,  56,
		56,  57,  57,  57,  57,  57,  58,  58,
		58,  58,  58,  59,  59,  59,  59,  59,
		60,  60,  60,  60,  60,  61,  61,  61,
		61,  61,  62,  62,  62,  62,  62,  62,
		62,  62,  62,  62,  63,  63,  63,  63,
		63,  63,  64,  64,  64,  64,  64,  64,
		65,  65,  65,  65,  65,  66,  66,  66,
		66,  66,  66,  67,  67,  67,  67,  67,
		67,  68,  68,  68,  68,  68,  68,  68,
		68,  68,  68,  68,  68,  69,  69,  69,
		69,  69,  69,  69,  70,  70,  70,  70,
		70,  70,  71,  71,  71,  71,  71,  71,
		72,  72,  72,  72,  72,  72,  72,  73,
		73,  73,  73,  73,  73,  73,  74,  74,
		74,  74,  74,  74,  74,  74,  74,  74,
		74,  74,  74,  74,  75,  75,  75,  75,
		75,  75,  75,  76,  76,  76,  76,  76,
		76,  76,  77,  77,  77,  77,  77,  77,
		77,  78,  78,  78,  78,  78,  78,  78,
		78,  79,  79,  79,  79,  79,  79,  79,
		80,  80,  80,  80,  80,  80,  80,  80,
		80,  80,  80,  80,  80,  80,  80,  80,
		81,  81,  81,  81,  81,  81,  81,  81,
		82,  82,  82,  82,  82,  82,  82,  82,
		83,  83,  83,  83,  83,  83,  83,  83,
		84,  84,  84,  84,  84,  84,  84,  84,
		84,  85,  85,  85,  85,  85,  85,  85,
		85,  86,  86,  86,  86,  86,  86,  86,
		86,  86,  86,  86,  86,  86,  86,  86,
		86,  86,  87,  87,  87,  87,  87,  87,
		87,  87,  87,  88,  88,  88,  88,  88,
		88,  88,  88,  88,  89,  89,  89,  89,
		89,  89,  89,  89,  89,  90,  90,  90,
		90,  90,  90,  90,  90,  90,  91,  91,
		91,  91,  91,  91,  91,  91,  91,  91,
		92,  92,  92,  92,  92,  92,  92,  92,
		92,  92,  92,  92,  92,  92,  92,  92,
		92,  92,  92,  93,  93,  93,  93,  93,
		93,  93,  93,  93,  94,  94,  94,  94,
		94,  94,  94,  94,  94,  94,  95,  95,
		95,  95,  95,  95,  95,  95,  95,  95,
		96,  96,  96,  96,  96,  96,  96,  96,
		96,  96,  97,  97,  97,  97,  97,  97,
		97,  97,  97,  97,  97,  98,  98,  98,
		98,  98,  98,  98,  98,  98,  98,  98,
		98,  98,  98,  98,  98,  98,  98,  98,
		98,  98,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99, 100, 100, 100, 100,
		100, 100, 100, 100, 100, 100, 100, 101,
		101, 102, 103, 103, 104, 104, 105, 105,
		106, 107, 107, 108, 109, 109, 110, 110,
		110, 111, 111, 112, 113, 113, 114, 115,
		115, 116, 116, 116, 117, 117, 118, 118,
		119, 120, 120, 121, 121, 122, 122, 122,
		122, 123, 124, 124, 125, 125, 126, 126,
		127, 127, 128, 128, 129, 129, 129, 129,
		130, 130, 131, 131, 132, 132, 133, 133,
		134, 134, 135, 135, 135, 135, 136, 136,
		137, 137, 138, 138, 139, 139, 140, 140,
		141, 141, 141, 141, 141, 142, 142, 143,
		143, 144, 144, 144, 145, 145, 146, 146,
		147, 147, 147, 147, 147, 148, 148, 149,
		149, 149, 150, 150, 151, 151, 151, 152,
		152, 153, 153, 153, 153, 153, 154, 154,
		154, 155, 155, 156, 156, 156, 157, 157,
		158, 158, 158, 159, 159, 159, 159, 159,
		160, 160, 160, 161, 161, 162, 162, 162,
		163, 163, 163, 164, 164, 165, 165, 165,
		165, 165, 165, 166, 166, 166, 167, 167,
		167, 168, 168, 169, 169, 169, 170, 170,
		170, 171, 171, 171, 171, 171, 171, 172,
		172, 172, 173, 173, 173, 174, 174, 174,
		175, 175, 175, 176, 176, 176, 177, 177,
		177, 177, 177, 177, 178, 178, 178, 179,
		179, 179, 180, 180, 180, 181, 181, 181,
		181, 182, 182, 182, 183, 183, 183, 183,
		183, 183, 184, 184, 184, 185, 185, 185,
		185, 186, 186, 186, 187, 187, 187, 188,
		188, 188, 188, 189, 189, 189, 189, 189,
		189, 190, 190, 190, 190, 191, 191, 191,
		192, 192, 192, 193, 193, 193, 193, 194,
		194, 194, 195, 195, 195, 195, 195, 195,
		195, 196, 196, 196, 196, 197, 197, 197,
		197, 198, 198, 198, 199, 199, 199, 199,
		200, 200, 200, 201, 201, 201, 201, 202,
		202, 202, 202, 202, 202, 202, 203, 203,
		203, 203, 204, 204, 204, 204, 205, 205,
		205, 205, 206, 206, 206, 207, 207, 207,
		207, 208, 208, 208, 208, 208, 208, 208,
		208, 209, 209, 209, 209, 210, 210, 210,
		210, 211, 211, 211, 211, 212, 212, 212,
		212, 213, 213, 213, 213, 214, 214, 214,
		214, 214, 214, 214, 214, 215, 215, 215,
		215, 216, 216, 216, 216, 217, 217, 217,
		217, 218, 218, 218, 218, 219, 219, 219,
		219, 220, 220, 220, 220, 220, 220, 220,
		220, 221, 221, 221, 221, 221, 222, 222,
		222, 222, 223, 223, 223, 223, 224, 224,
		224, 224, 225, 225, 225, 225, 225, 226,
		226, 226, 226, 226, 226, 226, 226, 227,
		227, 227, 227, 227, 228, 228, 228, 228,
		229, 229, 229, 229, 230, 230, 230, 230,
		230, 231, 231, 231, 231, 232, 232, 232,
		232, 232, 232, 232, 232, 232, 233, 233,
		233, 233, 233, 234, 234, 234, 234, 235
	},
};

static struct tegra_dc_nvdisp_cmu default_nvdisp_cmu = {
	{},
};
static struct tegra_dc_nvdisp_cmu default_limited_nvdisp_cmu = {
	{},
};

#define DSC_MAX_RC_BUF_THRESH_REGS	4
static int dsc_rc_buf_thresh_regs[DSC_MAX_RC_BUF_THRESH_REGS] = {
	DC_COM_DSC_RC_BUF_THRESH_0,
	DC_COM_DSC_RC_BUF_THRESH_1,
	DC_COM_DSC_RC_BUF_THRESH_2,
	DC_COM_DSC_RC_BUF_THRESH_3,
};

/*
 * Always set the first two values to 0. This is to ensure that RC threshold
 * values are programmed in the correct registers.
 */
static int dsc_rc_buf_thresh[] = {
	0, 0, 14, 28, 42, 56, 70, 84, 98, 105, 112, 119, 121,
	123, 125, 126,
};

#define DSC_MAX_RC_RANGE_CFG_REGS	8
static int dsc_rc_range_config[DSC_MAX_RC_RANGE_CFG_REGS] = {
	DC_COM_DSC_RC_RANGE_CFG_0,
	DC_COM_DSC_RC_RANGE_CFG_1,
	DC_COM_DSC_RC_RANGE_CFG_2,
	DC_COM_DSC_RC_RANGE_CFG_3,
	DC_COM_DSC_RC_RANGE_CFG_4,
	DC_COM_DSC_RC_RANGE_CFG_5,
	DC_COM_DSC_RC_RANGE_CFG_6,
	DC_COM_DSC_RC_RANGE_CFG_7,
};

static int dsc_rc_ranges_8bpp_8bpc[16][3] = {
	{0, 4, 2},
	{0, 4, 0},
	{1, 5, 0},
	{1, 6, -2},
	{3, 7, -4},
	{3, 7, -6},
	{3, 7, -8},
	{3, 8, -8},
	{3, 9, -8},
	{3, 10, -10},
	{5, 11, -10},
	{5, 12, -12},
	{5, 13, -12},
	{7, 13, -12},
	{13, 15, -12},
	{0, 0, 0},
};

static void tegra_dc_t21x_activate_general_channel(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */
}

void tegra_dc_activate_general_channel(struct tegra_dc *dc)
{
	if (tegra_dc_is_t21x())
		tegra_dc_t21x_activate_general_channel(dc);
	else
		tegra_nvdisp_activate_general_channel(dc);
}

unsigned long tegra_dc_readl_exported(struct tegra_dc *dc, unsigned long reg)
{
	return tegra_dc_readl(dc, reg);
}
EXPORT_SYMBOL(tegra_dc_readl_exported);

void tegra_dc_writel_exported(struct tegra_dc *dc,
			unsigned long val, unsigned long reg)
{
	tegra_dc_writel(dc, val, reg);
}
EXPORT_SYMBOL(tegra_dc_writel_exported);

void tegra_dc_clk_enable(struct tegra_dc *dc)
{
	tegra_disp_clk_prepare_enable(dc->clk);
#ifdef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_set_rate(dc->clk, dc->mode.pclk);
#endif
}

void tegra_dc_clk_disable(struct tegra_dc *dc)
{
	tegra_disp_clk_disable_unprepare(dc->clk);
#ifdef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_set_rate(dc->clk, 0);
#endif
}

void tegra_dc_get(struct tegra_dc *dc)
{
	int enable_count = atomic_inc_return(&dc->enable_count);

	BUG_ON(enable_count  < 1);
	if (enable_count == 1) {
		tegra_dc_io_start(dc);

		/* extra reference to dc clk */
		tegra_disp_clk_prepare_enable(dc->clk);
	}
}
EXPORT_SYMBOL(tegra_dc_get);

void tegra_dc_put(struct tegra_dc *dc)
{
	if (WARN_ONCE(atomic_read(&dc->enable_count) == 0,
		"unbalanced clock calls"))
		return;
	if (atomic_dec_return(&dc->enable_count) == 0) {
		/* balance extra dc clk reference */
		tegra_disp_clk_disable_unprepare(dc->clk);

		tegra_dc_io_end(dc);
	}
}
EXPORT_SYMBOL(tegra_dc_put);

unsigned tegra_dc_out_flags_from_dev(struct device *dev)
{
	struct platform_device *ndev = NULL;
	struct tegra_dc *dc = NULL;

	if (dev)
		ndev = to_platform_device(dev);
	if (ndev)
		dc = platform_get_drvdata(ndev);
	if (dc)
		return dc->out->flags;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_out_flags_from_dev);

inline bool tegra_dc_in_cmode(struct tegra_dc *dc)
{
	u32 nc_mode_flags = (TEGRA_DC_OUT_ONE_SHOT_MODE |
				TEGRA_DC_OUT_N_SHOT_MODE |
				TEGRA_DC_OUT_ONE_SHOT_LP_MODE);

	return !(dc->out->flags & nc_mode_flags);
}
EXPORT_SYMBOL(tegra_dc_in_cmode);

bool tegra_dc_initialized(struct device *dev)
{
	struct platform_device *ndev = NULL;
	struct tegra_dc *dc = NULL;

	if (dev)
		ndev = to_platform_device(dev);
	if (ndev)
		dc = platform_get_drvdata(ndev);
	if (dc)
		return dc->initialized;
	else
		return false;
}
EXPORT_SYMBOL(tegra_dc_initialized);

void tegra_dc_hold_dc_out(struct tegra_dc *dc)
{
	if (1 == atomic_inc_return(&dc->holding)) {
		tegra_dc_get(dc);
		if (dc->out_ops && dc->out_ops->hold)
			dc->out_ops->hold(dc);
	}
}

void tegra_dc_release_dc_out(struct tegra_dc *dc)
{
	if (0 == atomic_dec_return(&dc->holding)) {
		if (dc->out_ops && dc->out_ops->release)
			dc->out_ops->release(dc);
		tegra_dc_put(dc);
	}
}

bool tegra_dc_hotplug_supported(struct tegra_dc *dc)
{
	/* For HDMI|DP, hotplug always supported
	 * For eDP, hotplug is never supported
	 * For fake DP, SW hotplug is supported
	 * Else GPIO# determines if hotplug supported
	 */
	if (dc->out->type == TEGRA_DC_OUT_HDMI)
		return true;
	else if (dc->out->type == TEGRA_DC_OUT_DP ||
			dc->out->type == TEGRA_DC_OUT_FAKE_DP ||
			dc->out->type == TEGRA_DC_OUT_DSI)
		return tegra_dc_is_ext_panel(dc);
	else
		return (dc->out->hotplug_gpio > 0 ? true : false);
}

#define DUMP_REG(a) do {			\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08lx\n",  \
		 #a, a, tegra_dc_readl(dc, a));		      \
	print(data, buff);				      \
	} while (0)

void tegra_dc_reg_dump(struct tegra_dc *dc, void *data,
		       void (* print)(void *data, const char *str))
{
	int i;
	char buff[256];
	const char winname[] = "ABCDHT";
	/* for above, see also: DC_CMD_DISPLAY_WINDOW_HEADER and DC_N_WINDOWS */
	unsigned long cmd_state;

	/* If gated, quietly return. */
	if (!tegra_dc_is_powered(dc))
		return;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	cmd_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
		DC_CMD_STATE_ACCESS);

	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);

	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS1);
	DUMP_REG(DC_DISP_DISP_WIN_OPTIONS);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY_TIMER);
	DUMP_REG(DC_DISP_DISP_TIMING_OPTIONS);
	DUMP_REG(DC_DISP_REF_TO_SYNC);
	DUMP_REG(DC_DISP_SYNC_WIDTH);
	DUMP_REG(DC_DISP_BACK_PORCH);
	DUMP_REG(DC_DISP_DISP_ACTIVE);
	DUMP_REG(DC_DISP_FRONT_PORCH);
	DUMP_REG(DC_DISP_H_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_D);
	DUMP_REG(DC_DISP_V_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE3_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE3_POSITION_A);
	DUMP_REG(DC_DISP_M0_CONTROL);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_DISP_DI_CONTROL);
	DUMP_REG(DC_DISP_PP_CONTROL);
	DUMP_REG(DC_DISP_PP_SELECT_A);
	DUMP_REG(DC_DISP_PP_SELECT_B);
	DUMP_REG(DC_DISP_PP_SELECT_C);
	DUMP_REG(DC_DISP_PP_SELECT_D);
	DUMP_REG(DC_DISP_DISP_CLOCK_CONTROL);
	DUMP_REG(DC_DISP_DISP_INTERFACE_CONTROL);
	DUMP_REG(DC_DISP_DISP_COLOR_CONTROL);
	DUMP_REG(DC_DISP_SHIFT_CLOCK_OPTIONS);
	DUMP_REG(DC_DISP_DATA_ENABLE_OPTIONS);
	DUMP_REG(DC_DISP_SERIAL_INTERFACE_OPTIONS);
	DUMP_REG(DC_DISP_LCD_SPI_OPTIONS);
	DUMP_REG(DC_DISP_COLOR_KEY0_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY0_UPPER);
	DUMP_REG(DC_DISP_COLOR_KEY1_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY1_UPPER);
	DUMP_REG(DC_DISP_CURSOR_FOREGROUND);
	DUMP_REG(DC_DISP_CURSOR_BACKGROUND);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_NS);
	if (tegra_dc_is_t21x()) {
		DUMP_REG(DC_DISP_CURSOR_START_ADDR_HI);
		DUMP_REG(DC_DISP_CURSOR_START_ADDR_HI_NS);
	}
	DUMP_REG(DC_DISP_CURSOR_POSITION);
	DUMP_REG(DC_DISP_CURSOR_POSITION_NS);
	DUMP_REG(DC_DISP_INIT_SEQ_CONTROL);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_A);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_B);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_C);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_D);
	DUMP_REG(DC_DISP_DC_MCCIF_FIFOCTRL);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0B_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0C_HYST);
	DUMP_REG(DC_DISP_DAC_CRT_CTRL);
	DUMP_REG(DC_DISP_DISP_MISC_CONTROL);
	DUMP_REG(DC_DISP_INTERLACE_CONTROL);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_REF_TO_SYNC);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_SYNC_WIDTH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_BACK_PORCH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_FRONT_PORCH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_DISP_ACTIVE);

	DUMP_REG(DC_CMD_DISPLAY_POWER_CONTROL);
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY2);
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA2);
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT5);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_COM_PM1_CONTROL);
	DUMP_REG(DC_COM_PM1_DUTY_CYCLE);
	DUMP_REG(DC_DISP_SD_CONTROL);

	if (tegra_dc_is_t21x()) {
		DUMP_REG(DC_COM_CMU_CSC_KRR);
		DUMP_REG(DC_COM_CMU_CSC_KGR);
		DUMP_REG(DC_COM_CMU_CSC_KBR);
		DUMP_REG(DC_COM_CMU_CSC_KRG);
		DUMP_REG(DC_COM_CMU_CSC_KGG);
		DUMP_REG(DC_COM_CMU_CSC_KBG);
		DUMP_REG(DC_COM_CMU_CSC_KRB);
		DUMP_REG(DC_COM_CMU_CSC_KGB);
		DUMP_REG(DC_COM_CMU_CSC_KBB);
	}

	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		print(data, "\n");
		snprintf(buff, sizeof(buff), "WINDOW %c:\n", winname[i]);
		print(data, buff);

		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_WIN_WIN_OPTIONS);
		DUMP_REG(DC_WIN_BYTE_SWAP);
		DUMP_REG(DC_WIN_BUFFER_CONTROL);
		DUMP_REG(DC_WIN_COLOR_DEPTH);
		DUMP_REG(DC_WIN_POSITION);
		DUMP_REG(DC_WIN_SIZE);
		DUMP_REG(DC_WIN_PRESCALED_SIZE);
		DUMP_REG(DC_WIN_H_INITIAL_DDA);
		DUMP_REG(DC_WIN_V_INITIAL_DDA);
		DUMP_REG(DC_WIN_DDA_INCREMENT);
		DUMP_REG(DC_WIN_LINE_STRIDE);
		DUMP_REG(DC_WIN_BLEND_NOKEY);
		DUMP_REG(DC_WIN_BLEND_1WIN);
		DUMP_REG(DC_WIN_BLEND_2WIN_X);
		DUMP_REG(DC_WIN_BLEND_2WIN_Y);
		DUMP_REG(DC_WIN_BLEND_3WIN_XY);
		DUMP_REG(DC_WIN_GLOBAL_ALPHA);
		DUMP_REG(DC_WINBUF_BLEND_LAYER_CONTROL);
		DUMP_REG(DC_WINBUF_START_ADDR);
		DUMP_REG(DC_WINBUF_START_ADDR_U);
		DUMP_REG(DC_WINBUF_START_ADDR_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET);
		DUMP_REG(DC_WINBUF_START_ADDR_HI);
		DUMP_REG(DC_WINBUF_START_ADDR_HI_U);
		DUMP_REG(DC_WINBUF_START_ADDR_HI_V);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_U);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_V);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI_U);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET_FIELD2);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET_FIELD2);
		DUMP_REG(DC_WINBUF_UFLOW_STATUS);
		DUMP_REG(DC_WIN_CSC_YOF);
		DUMP_REG(DC_WIN_CSC_KYRGB);
		DUMP_REG(DC_WIN_CSC_KUR);
		DUMP_REG(DC_WIN_CSC_KVR);
		DUMP_REG(DC_WIN_CSC_KUG);
		DUMP_REG(DC_WIN_CSC_KVG);
		DUMP_REG(DC_WIN_CSC_KUB);
		DUMP_REG(DC_WIN_CSC_KVB);
		if (tegra_dc_is_t21x()) {
			DUMP_REG(DC_WINBUF_CDE_CONTROL);
			DUMP_REG(DC_WINBUF_CDE_COMPTAG_BASE_0);
			DUMP_REG(DC_WINBUF_CDE_COMPTAG_BASEHI_0);
			DUMP_REG(DC_WINBUF_CDE_ZBC_COLOR_0);
			DUMP_REG(DC_WINBUF_CDE_SURFACE_OFFSET_0);
			DUMP_REG(DC_WINBUF_CDE_CTB_ENTRY_0);
			DUMP_REG(DC_WINBUF_CDE_CG_SW_OVR);
			DUMP_REG(DC_WINBUF_CDE_PM_CONTROL);
			DUMP_REG(DC_WINBUF_CDE_PM_COUNTER);
		}
	}

	tegra_dc_writel(dc, cmd_state, DC_CMD_STATE_ACCESS);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

#undef DUMP_REG

#ifdef DEBUG
static void dump_regs_print(void *data, const char *str)
{
	struct tegra_dc *dc = data;
	dev_dbg(&dc->ndev->dev, "%s", str);
}

void dump_regs(struct tegra_dc *dc)
{
	reg_dump(dc, dc, dump_regs_print);
}
#else /* !DEBUG */

void dump_regs(struct tegra_dc *dc) {}

#endif /* DEBUG */

#ifdef CONFIG_DEBUG_FS

static int dbg_timestamp_show(struct seq_file *s, void *unused)
{
	u32 tmp;
	u64 timestamp;
	u32 frame_cnt;
	struct tegra_dc *dc;

	dc = s->private;

	do {
		tmp = (u32)(tegra_dc_readl(dc, DC_COM_RG_DPCA) >> 16);
		timestamp = tegra_dc_get_vsync_timestamp(dc);
		frame_cnt = tegra_dc_get_frame_cnt(dc);
	} while (tmp != frame_cnt);

	seq_printf(s, "vsync_timestamp : %llu\n"
			"frame_no : %u\n", timestamp, frame_cnt);

	return 0;
}

static int dbg_timestamp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_timestamp_show, inode->i_private);
}

static const struct file_operations timestamp_fops = {
	.open		= dbg_timestamp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void dbg_regs_print(void *data, const char *str)
{
	struct seq_file *s = data;

	seq_printf(s, "%s", str);
}

#undef DUMP_REG

static int dbg_dc_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	reg_dump(dc, s, dbg_regs_print);

	return 0;
}


static int dbg_dc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= dbg_dc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_mode_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	struct tegra_dc_mode *m;

	mutex_lock(&dc->lock);
	m = &dc->mode;
	seq_printf(s,
		"pclk: %d\n"
		"h_ref_to_sync: %d\n"
		"v_ref_to_sync: %d\n"
		"h_sync_width: %d\n"
		"v_sync_width: %d\n"
		"h_back_porch: %d\n"
		"v_back_porch: %d\n"
		"h_active: %d\n"
		"v_active: %d\n"
		"h_front_porch: %d\n"
		"v_front_porch: %d\n"
		"flags: 0x%x\n"
		"stereo_mode: %d\n"
		"avi_m: 0x%x\n"
		"vmode: 0x%x\n",
		m->pclk, m->h_ref_to_sync, m->v_ref_to_sync,
		m->h_sync_width, m->v_sync_width,
		m->h_back_porch, m->v_back_porch,
		m->h_active, m->v_active,
		m->h_front_porch, m->v_front_porch,
		m->flags, m->stereo_mode, m->avi_m, m->vmode);
	mutex_unlock(&dc->lock);
	return 0;
}

static int dbg_dc_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_mode_show, inode->i_private);
}

static const struct file_operations mode_fops = {
	.open		= dbg_dc_mode_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_stats_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	mutex_lock(&dc->lock);
	seq_printf(s,
		"underflows: %llu\n",
		dc->stats.underflows);
	if (tegra_dc_is_nvdisplay()) {
		seq_printf(s,
			"underflow_frames: %llu\n",
			dc->stats.underflow_frames);
	} else {
		seq_printf(s,
			"underflows_a: %llu\n"
			"underflows_b: %llu\n"
			"underflows_c: %llu\n",
			dc->stats.underflows_a,
			dc->stats.underflows_b,
			dc->stats.underflows_c);
		seq_printf(s,
			"underflows_d: %llu\n"
			"underflows_h: %llu\n"
			"underflows_t: %llu\n",
			dc->stats.underflows_d,
			dc->stats.underflows_h,
			dc->stats.underflows_t);
	}
	mutex_unlock(&dc->lock);

	return 0;
}

static int dbg_dc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_stats_show, inode->i_private);
}

static int dbg_dc_event_inject_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t dbg_dc_event_inject_write(struct file *file,
	const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc *dc = m ? m->private : NULL;
	long event;
	int ret;

	if (!dc)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &event);
	if (ret < 0)
		return ret;

	/*
	 * ADF has two seperate events for hotplug connect and disconnect.
	 * We map event 0x0, and 0x1 for them accordingly.  For DC_EXT,
	 * both events map to HOTPLUG.
	 */
	if (event == 0x0 || event == 0x1) /* TEGRA_DC_EXT_EVENT_HOTPLUG */
		tegra_dc_ext_process_hotplug(dc->ndev->id);
	else if (event == 0x2) /* TEGRA_DC_EXT_EVENT_BANDWIDTH_DEC */
		tegra_dc_ext_process_bandwidth_renegotiate(
				dc->ndev->id, NULL);
	else {
		dev_err(&dc->ndev->dev, "Unknown event 0x%lx\n", event);
		return -EINVAL; /* unknown event number */
	}
	return len;
}

/* Update the strings as dc.h get updated for new output types*/
static const char * const dc_outtype_strings[] = {
	"TEGRA_DC_OUT_RGB",
	"TEGRA_DC_OUT_HDMI",
	"TEGRA_DC_OUT_DSI",
	"TEGRA_DC_OUT_DP",
	"TEGRA_DC_OUT_LVDS",
	"TEGRA_DC_OUT_NVSR_DP",
	"TEGRA_DC_OUT_FAKE_DP",
	"TEGRA_DC_OUT_FAKE_DSIA",
	"TEGRA_DC_OUT_FAKE_DSIB",
	"TEGRA_DC_OUT_FAKE_DSI_GANGED",
	"TEGRA_DC_OUT_NULL",
	"TEGRA_DC_OUT_UNKNOWN"
};

static int dbg_dc_outtype_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	mutex_lock(&dc->lock);
	seq_puts(s, "\n");
	seq_printf(s,
		"\tDC OUTPUT: \t%s (%d)\n",
		dc_outtype_strings[dc->out->type], dc->out->type);
	seq_puts(s, "\n");
	mutex_unlock(&dc->lock);
	return 0;
}

/* Add specific variable related to each output type.
 * Save and reuse on changing the output type
 */
#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT)
/* array for saving the out_type for each head */
static int *boot_out_type;

static int is_invalid_dc_out(struct tegra_dc *dc, long dc_outtype)
{
	if ((dc_outtype != boot_out_type[dc->ndev->id]) &&
		(dc_outtype != TEGRA_DC_OUT_FAKE_DP) &&
		(dc_outtype != TEGRA_DC_OUT_FAKE_DSIA) &&
		(dc_outtype != TEGRA_DC_OUT_FAKE_DSIB) &&
		(dc_outtype != TEGRA_DC_OUT_FAKE_DSI_GANGED) &&
		(dc_outtype != TEGRA_DC_OUT_NULL)) {
		dev_err(&dc->ndev->dev, "Request 0x%lx is unsupported target out_type\n",
			 dc_outtype);
		dev_err(&dc->ndev->dev, "boot_out_type[%d] is 0x%x\n",
			 dc->ndev->id, boot_out_type[dc->ndev->id]);
		return -EINVAL;
	}

	return 0;
}

static int is_valid_dsi_out(struct tegra_dc *dc, long dc_outtype)
{
	if (((dc_outtype >= TEGRA_DC_OUT_FAKE_DSIA) &&
		(dc_outtype <= TEGRA_DC_OUT_FAKE_DSI_GANGED)) ||
		(dc_outtype == TEGRA_DC_OUT_DSI))
			return 1;

	return 0;
}


static int is_valid_fake_support(struct tegra_dc *dc, long dc_outtype)
{
	if ((dc_outtype == TEGRA_DC_OUT_FAKE_DP) ||
		(dc_outtype == TEGRA_DC_OUT_FAKE_DSIA) ||
		(dc_outtype == TEGRA_DC_OUT_FAKE_DSIB) ||
		(dc_outtype == TEGRA_DC_OUT_FAKE_DSI_GANGED) ||
		(dc_outtype == TEGRA_DC_OUT_NULL))
		return 1;

	return 0;
}

static int set_avdd(struct tegra_dc *dc, long cur_out, long new_out)
{
	/* T210 macro_clk is failing SOR access
	 * if avdd_lcd is not enabled
	 */
	bool is_enable = false;
	struct tegra_dc_out *dc_out =
		&dc->dbg_dc_out_info[boot_out_type[dc->ndev->id]].out;

	if (tegra_dc_is_nvdisplay())
		return 0;

	/* cur is fake and new is fake - skip */
	if (is_valid_fake_support(dc, cur_out) &&
		is_valid_fake_support(dc, new_out))
		return 0;

	/* cur is valid and new is fake - enable */
	if (!is_valid_fake_support(dc, cur_out) &&
		is_valid_fake_support(dc, new_out))
		is_enable = true;

	if (is_enable) {
		if (dc_out && dc_out->enable)
			dc_out->enable(&dc->ndev->dev);
	} else {
		if (dc_out && dc_out->disable)
			dc_out->disable(&dc->ndev->dev);
	}
	return 0;
}
static ssize_t dbg_dc_out_type_set(struct file *file,
	const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc *dc = m ? m->private : NULL;
	long cur_dc_out;
	long out_type;
	int ret = 0;
	bool  allocate = false;

	if (!dc)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &out_type);
	if (ret < 0)
		return ret;

	if (!dc->pdata->default_out)
		return -EINVAL;

	/* check out type is out of range then skip */
	if (out_type < TEGRA_DC_OUT_RGB || out_type >= TEGRA_DC_OUT_MAX) {
		dev_err(&dc->ndev->dev, "Unknown out_type 0x%lx\n", out_type);
		return -EINVAL;
	}

	if (boot_out_type[dc->ndev->id] == -1)
		boot_out_type[dc->ndev->id] = dc->pdata->default_out->type;

	cur_dc_out = dc->pdata->default_out->type;

	/* Nothing to do if new outtype is same as old
	 * Allow to switch between booted out type and fake panel out
	 */
	if ((cur_dc_out == out_type) || is_invalid_dc_out(dc, out_type))
		return -EINVAL;

	/* disable the dc and output controllers */
	if (dc->enabled)
		tegra_dc_disable(dc);

	/* Clear EDID error flags */
	if (dc->edid)
		dc->edid->errors = 0;

	/* If output is already created - save it */
	if (dc->out_data) {
		dc->dbg_dc_out_info[cur_dc_out].out_data = dc->out_data;
		dc->dbg_dc_out_info[cur_dc_out].out_ops  = dc->out_ops;
		memcpy(&dc->dbg_dc_out_info[cur_dc_out].out, dc->out,
					sizeof(struct tegra_dc_out));
		dc->dbg_dc_out_info[cur_dc_out].mode = dc->mode;
		dc->dbg_dc_out_info[cur_dc_out].edid = dc->edid;

		if (is_valid_dsi_out(dc, cur_dc_out) &&
		    dc->dbg_dc_out_info[cur_dc_out].out_data)
			tegra_dc_destroy_dsi_resources(dc, cur_dc_out);

		if (!is_valid_fake_support(dc, cur_dc_out))
			dc->dbg_dc_out_info[cur_dc_out].fblistindex =
						tegra_fb_update_modelist(dc, 0);

		set_avdd(dc, cur_dc_out, out_type);
	}

	/* If output already created - reuse it */
	if (dc->dbg_dc_out_info[out_type].out_data) {
		mutex_lock(&dc->lp_lock);
		mutex_lock(&dc->lock);

		/* Change the out type */
		dc->pdata->default_out->type = out_type;
		dc->out_ops = dc->dbg_dc_out_info[out_type].out_ops;
		dc->out_data = dc->dbg_dc_out_info[out_type].out_data;
		memcpy(dc->out, &dc->dbg_dc_out_info[out_type].out,
						sizeof(struct tegra_dc_out));
		dc->mode = dc->dbg_dc_out_info[out_type].mode;
		dc->edid = dc->dbg_dc_out_info[out_type].edid;

		/* Re-init the resources that are destroyed for dsi */
		if (is_valid_dsi_out(dc, out_type))
			ret = tegra_dc_reinit_dsi_resources(dc, out_type);

		if (!is_valid_fake_support(dc, out_type))
			tegra_fb_update_modelist(dc,
				dc->dbg_dc_out_info[out_type].fblistindex);

		mutex_unlock(&dc->lock);
		mutex_unlock(&dc->lp_lock);

		if (ret) {
			dev_err(&dc->ndev->dev, "Failed to reinit!!!\n");
			return -EINVAL;
		}

	} else {
		/* Change the out type */
		dc->pdata->default_out->type = out_type;

		/* create new - now restricted to fake_dp only */
		if (out_type == TEGRA_DC_OUT_FAKE_DP) {
			/* set to default bpp */
			if (!dc->pdata->default_out->depth)
				dc->pdata->default_out->depth = 24;

			/* DP and Fake_DP use same data
			*  Reuse DP data for fake_DP */
			if (cur_dc_out != TEGRA_DC_OUT_DP) {
				allocate = true;
			}
		} else if ((out_type >= TEGRA_DC_OUT_FAKE_DSIA) &&
				(out_type <= TEGRA_DC_OUT_FAKE_DSI_GANGED)) {
			/* DSI and fake DSI use same data
			 * create new if not created yet
			 */
			if (!dc->pdata->default_out->depth)
				dc->pdata->default_out->depth = 18;

			allocate = true;
			tegra_dc_init_fakedsi_panel(dc, out_type);

		} else if (out_type == TEGRA_DC_OUT_NULL) {
			if (!dc->dbg_dc_out_info[TEGRA_DC_OUT_NULL].out_data) {
				allocate = true;
			}
		} else {
			/* set  back to existing one */
			dc->pdata->default_out->type = cur_dc_out;
			dev_err(&dc->ndev->dev, "Unknown type is asked: %ld\n",
					out_type);
			goto by_pass;
		}

		if (allocate) {
			ret = tegra_dc_set_out(dc, dc->pdata->default_out,
				true);
				if (ret < 0) {
					dev_err(&dc->ndev->dev,
					"Failed to initialize DC out ops\n");
					return -EINVAL;
				}
		}

		dc->dbg_dc_out_info[out_type].out_ops = dc->out_ops;
		dc->dbg_dc_out_info[out_type].out_data = dc->out_data;
		memcpy(&dc->dbg_dc_out_info[out_type].out, dc->out,
						sizeof(struct tegra_dc_out));
		dc->dbg_dc_out_info[out_type].mode = dc->mode;
		dc->dbg_dc_out_info[out_type].edid = dc->edid;

	}
	if (tegra_fb_is_console_enabled(dc->pdata))
		switch (out_type) {
		case TEGRA_DC_OUT_DSI:
		case TEGRA_DC_OUT_DP:
			if (dc->out_ops->detect)
				dc->out_ops->detect(dc);
			break;

		case TEGRA_DC_OUT_FAKE_DP:
		case TEGRA_DC_OUT_FAKE_DSIA:
		case TEGRA_DC_OUT_FAKE_DSIB:
		case TEGRA_DC_OUT_FAKE_DSI_GANGED:
			tegra_fb_update_monspecs(dc->fb, NULL, NULL);
			break;
		}

by_pass:
	/*enable the dc and output controllers */
	if (!dc->enabled)
		tegra_dc_enable(dc);

	return len;
}
#else
static ssize_t dbg_dc_out_type_set(struct file *file,
	const char __user *addr, size_t len, loff_t *pos)
{
	return -EINVAL;
}
#endif /*CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT*/

static const struct file_operations stats_fops = {
	.open		= dbg_dc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_event_inject_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_event_inject_show, inode->i_private);
}

static const struct file_operations event_inject_fops = {
	.open		= dbg_dc_event_inject_open,
	.read		= seq_read,
	.write		= dbg_dc_event_inject_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_outtype_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_outtype_show, inode->i_private);
}

static const struct file_operations outtype_fops = {
	.open		= dbg_dc_outtype_open,
	.read		= seq_read,
	.write		= dbg_dc_out_type_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_edid_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	struct tegra_edid *edid = dc->edid;
	struct tegra_dc_edid *data;
	u8 *buf;
	int i;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	if (dc->out->type == TEGRA_DC_OUT_DSI ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED) {
		seq_puts(s, "No EDID\n");
		return 0;
	}

	if (WARN_ON(!dc->edid))
		return -EINVAL;

	data = tegra_edid_get_data(edid);
	if (!data) {
		seq_puts(s, "No EDID\n");
		return 0;
	}

	buf = data->buf;

	for (i = 0; i < data->len; i++) {
#ifdef DEBUG
		if (i % 16 == 0)
			seq_printf(s, "edid[%03x] =", i);
#endif

		seq_printf(s, " %02x", buf[i]);

		if (i % 16 == 15)
			seq_puts(s, "\n");

	}

	tegra_edid_put_data(data);

	return 0;
}

static int dbg_edid_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_edid_show, inode->i_private);
}

static ssize_t dbg_edid_write(struct file *file,
const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc *dc = m ? m->private : NULL;
	int ret;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	dc->vedid = false;

	kfree(dc->vedid_data);
	dc->vedid_data = NULL;

	if (len < 128) /* invalid edid, turn off vedid */
		return 1;

	dc->vedid_data = kmalloc(sizeof(char) * len, GFP_KERNEL);
	if (!dc->vedid_data) {
		dev_err(&dc->ndev->dev, "no memory for edid\n");
		return 0; /* dc->vedid is false */
	}

	ret = copy_from_user(dc->vedid_data, addr, len);
	if (ret < 0) {
		dev_err(&dc->ndev->dev, "error copying edid\n");
		kfree(dc->vedid_data);
		dc->vedid_data = NULL;
		return ret; /* dc->vedid is false */
	}

	dc->vedid = true;

	return len;
}

static const struct file_operations edid_fops = {
	.open		= dbg_edid_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= dbg_edid_write,
	.release	= single_release,
};

static int dbg_hotplug_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	rmb();
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	seq_put_decimal_ll(s, '\0', dc->out->hotplug_state);
#else
	seq_put_decimal_ll(s, "", dc->out->hotplug_state);
#endif
	seq_putc(s, '\n');
	return 0;
}

static int dbg_hotplug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hotplug_show, inode->i_private);
}

static ssize_t dbg_hotplug_write(struct file *file, const char __user *addr,
	size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc *dc = m ? m->private : NULL;
	int ret;
	long new_state;
	int hotplug_state;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_state);
	if (ret < 0)
		return ret;

	mutex_lock(&dc->lock);
	rmb();
	hotplug_state = dc->out->hotplug_state;

	if (tegra_platform_is_sim() || tegra_platform_is_fpga())
		goto skip_gpio;

	if (hotplug_state == TEGRA_HPD_STATE_NORMAL &&
	    new_state != TEGRA_HPD_STATE_NORMAL     &&
	    dc->hotplug_supported) {
		/* we are overriding the hpd GPIO, so ignore the interrupt. */
		int gpio_irq = gpio_to_irq(dc->out->hotplug_gpio);

		disable_irq(gpio_irq);
	} else if (hotplug_state != TEGRA_HPD_STATE_NORMAL &&
		   new_state == TEGRA_HPD_STATE_NORMAL     &&
		   dc->hotplug_supported) {
		/* restore the interrupt for hpd GPIO. */
		int gpio_irq = gpio_to_irq(dc->out->hotplug_gpio);

		enable_irq(gpio_irq);
	}

skip_gpio:
	dc->out->hotplug_state = new_state;
	wmb();

	/* retrigger the hotplug
	 * exception: dont trigger for simulator with hotplug
	 * state TEGRA_HPD_STATE_NORMAL since hpd_state
	 * callback for sim always returns true
	 * */
	reinit_completion(&dc->hpd_complete);
	if (dc->out_ops->detect)
		dc->connected = dc->out_ops->detect(dc);
	mutex_unlock(&dc->lock);
	wait_for_completion(&dc->hpd_complete);
	return len;
}

static const struct file_operations dbg_hotplug_fops = {
	.open		= dbg_hotplug_open,
	.read		= seq_read,
	.write		= dbg_hotplug_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_color_expand_enable_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_win *win = m->private;

	if (!win)
		return -EINVAL;

	seq_printf(m, "%d\n", win->color_expand_enable);

	return 0;
}

static int dbg_color_expand_enable_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, dbg_color_expand_enable_show,
		inode->i_private);
}

static ssize_t dbg_color_expand_enable_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc_win *win = m->private;
	long   new_state;
	int    ret;

	if (!win)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_state);
	if (ret < 0)
		return ret;

	if (new_state == 1)
		win->color_expand_enable = true;
	else if (new_state == 0)
		win->color_expand_enable = false;

	return len;
}

static const struct file_operations dbg_color_expand_enable_fops = {
	.open = dbg_color_expand_enable_open,
	.read = seq_read,
	.write = dbg_color_expand_enable_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_degamma_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_win *win = m->private;
	long degamma_setting = 0;

	if (!win)
		return -EINVAL;

	degamma_setting = tegra_nvdisp_get_degamma_user_config(win);
	seq_printf(m, "%ld\n", degamma_setting);

	return 0;
}

static int dbg_degamma_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, dbg_degamma_show,
		inode->i_private);
}

/* Note that degamma programmed here will not be persistent across
 * window attach/detach.
 */
static ssize_t dbg_degamma_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc_win *win = m->private;
	long degamma_setting;
	int ret;

	if (!win)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &degamma_setting);
	if (ret < 0)
		return ret;

	tegra_nvdisp_set_degamma_user_config(win, degamma_setting);

	return len;
}

static int dbg_force_user_degamma_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_win *win = m->private;

	if (!win)
		return -EINVAL;

	seq_printf(m, "%d\n", win->force_user_degamma);

	return 0;
}

static int dbg_force_user_degamma_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, dbg_force_user_degamma_show,
		inode->i_private);
}

static ssize_t dbg_force_user_degamma_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc_win *win = m->private;
	long   new_state;
	int    ret;

	if (!win)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_state);
	if (ret < 0)
		return ret;

	if (new_state == 1)
		win->force_user_degamma = true;
	else if (new_state == 0)
		win->force_user_degamma = false;

	return len;
}

static const struct file_operations dbg_force_user_degamma_fops = {
	.open = dbg_force_user_degamma_open,
	.read = seq_read,
	.write = dbg_force_user_degamma_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations dbg_degamma_fops = {
	.open = dbg_degamma_open,
	.read = seq_read,
	.write = dbg_degamma_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static bool tegra_dc_or_is_dsi(struct tegra_dc *dc)
{
	if (dc->out_ops && !dc->out_ops->get_connector_instance &&
			dc->out->type == TEGRA_DC_OUT_DSI)
		return true;
	else
		return false;
}

static int dbg_nvdisp_topology_show(struct seq_file *m, void *unused)
{
	int i;
	struct tegra_dc *dc;

	for (i = 0; i < tegra_dc_get_numof_reg_disps(); i++) {
		dc = tegra_dc_get_dc(i);

		if (!dc->current_topology.valid) {
			seq_printf(m, "DISPLAY ID:%d HEAD:%d DANGLING TOPOLOGY\n",
				i, tegra_dc_get_head(dc));
			continue;
		}

		if (tegra_dc_or_is_dsi(dc))
			seq_printf(m, "DISPLAY ID:%d HEAD:%d OUT_TYPE:%d DSI\n",
				i, tegra_dc_get_head(dc),
				dc->current_topology.protocol);
		else
			seq_printf(m, "DISPLAY ID:%d HEAD:%d OUT_TYPE:%d SOR%d\n",
				i, tegra_dc_get_head(dc),
				dc->current_topology.protocol,
				dc->current_topology.conn_inst);
	}
	return 0;
}

static int dbg_nvdisp_topology_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, dbg_nvdisp_topology_show,
		inode->i_private);
}

static bool is_topology_same(struct tegra_dc_topology topology1,
		struct tegra_dc_topology topology2)
{
	if (topology1.disp_id != topology2.disp_id)
		return false;

	if (topology1.protocol != topology2.protocol)
		return false;

	if (topology1.conn_inst != topology2.conn_inst)
		return false;

	return true;
}

static bool is_topology_reset(struct tegra_dc_topology topology)
{
	if ((topology.disp_id == TEGRA_DC_TOPOLOGY_RESTORE) &&
		(topology.protocol == TEGRA_DC_TOPOLOGY_RESTORE) &&
		(topology.conn_inst == TEGRA_DC_TOPOLOGY_RESTORE))
		return true;

	return false;
}

static bool is_topology_possible(struct tegra_dc_topology topology)
{
	struct tegra_dc *dc = NULL;

	/* Check to see if restore boot topology is called */
	if (is_topology_reset(topology))
		return true;

	if (topology.disp_id >= tegra_dc_get_numof_reg_disps())
		return false;

	if (topology.conn_inst >= tegra_dc_get_numof_dispsors())
		return false;

	/* Prevent crossbar on DSI display until support is added in sw */
	dc = tegra_dc_get_dc(topology.disp_id);
	if (tegra_dc_or_is_dsi(dc))
		return false;

	/* Currently supports only DP,FAKE_DP and HDMI in SW */
	if ((topology.protocol == TEGRA_DC_OUT_FAKE_DP) ||
		(topology.protocol == TEGRA_DC_OUT_DP) ||
		(topology.protocol == TEGRA_DC_OUT_HDMI)) {
		return true;
	}

	return false;
}

static int tegra_dc_topology_parse(char *buf, int nargs,
		struct tegra_dc_topology *topology)
{
	char *b = NULL, *orig_b = NULL, *c = NULL;
	int i = 0, args[TEGRA_DC_TOPOLOGY_NARGS] = {TEGRA_DC_TOPOLOGY_INVALID};

	orig_b = kstrdup(buf, GFP_KERNEL);
	b = orig_b;

	for (i = 0; i < nargs; i++) {
		if (!b)
			break;
		b = strim(b);
		c = strsep(&b, ":");
		if (!strlen(c))
			break;
		args[i] = simple_strtol(c, NULL, 10);
		if (args[i] < TEGRA_DC_TOPOLOGY_ARG_MIN ||
				args[i] > TEGRA_DC_TOPOLOGY_ARG_MAX)
			break;
	}

	kfree(orig_b);

	if (i == nargs) {
		topology->disp_id = args[0];
		topology->protocol = args[1];
		topology->conn_inst = args[2];
		return 0;
	} else {
		return -EINVAL;
	}
}

static int tegra_dc_crossbar_display_reinit(struct tegra_dc *dc,
	struct tegra_dc_topology topology)
{
	int ret = 0;

	dc->pdata->default_out->type = topology.protocol;
	dc->current_topology = topology;

	/* Parse the platform data again */
	dc->pdata = of_dc_parse_platform_data(dc->ndev, dc->pdata);
	if (IS_ERR_OR_NULL(dc->pdata)) {
		pr_warn("crossbar: parsing platform data for new topology failed\n");
		return -EFAULT;
	}

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init(&dc->ndev->dev);

	ret = tegra_dc_set_out(dc, dc->pdata->default_out, true);
	if (ret < 0) {
		pr_warn("crossbar: initialize DC out ops for new topology failed\n");
		return -EFAULT;
	}

	if (dc->out_ops->get_connector_instance) {
		char sor_path[CHAR_BUF_SIZE_MAX];
		int ctrl_num = -1;

		ctrl_num = dc->out_ops->get_connector_instance(dc);
		if (ctrl_num < 0) {
			pr_err("crossbar: SOR controller instance not found\n");
			return -EFAULT;
		}

		snprintf(sor_path, sizeof(sor_path),
				"/sys/kernel/debug/tegra_sor%d", ctrl_num);
		dc->sor_link = debugfs_create_symlink("sor", dc->debugdir, sor_path);
		if (!dc->sor_link) {
			pr_err("crossbar: couldn't create symbolic link to SOR%d for DC%d\n",
				ctrl_num, dc->ctrl_num);
			return -EFAULT;
		}
	}

	if (dc->out_ops && dc->out_ops->hotplug_init)
		dc->out_ops->hotplug_init(dc);

	if (dc->out_ops && dc->out_ops->detect)
		dc->connected = dc->out_ops->detect(dc);

	dc->current_topology.valid = true;

	return ret;
}

static ssize_t dbg_nvdisp_topology_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	int res = 0, ret = 0;
	int i = 0;
	char buf[CHAR_BUF_SIZE_MAX+1] = {'\0'};
	struct tegra_dc *primary =  NULL;
	struct tegra_dc *curr = NULL;
	bool dangling = false;
	struct tegra_dc_topology topology = {false, TEGRA_DC_TOPOLOGY_INVALID,
		TEGRA_DC_TOPOLOGY_INVALID, TEGRA_DC_TOPOLOGY_INVALID};

	res = copy_from_user(buf, addr, CHAR_BUF_SIZE_MAX);

	ret = tegra_dc_topology_parse(buf, TEGRA_DC_TOPOLOGY_NARGS, &topology);
	if (ret < 0) {
		pr_warn("crossbar: invalid input format for topology\n");
		return -EINVAL;
	}

	/* Check to see if all registered displays are disabled */
	for (i = 0; i < tegra_dc_get_numof_reg_disps(); i++) {
		curr = tegra_dc_get_dc(i);

		if (curr->enabled) {
			pr_warn("crossbar: all displays are not disabled\n");
			return -EINVAL;
		}
	}

	curr = NULL;

	if (!is_topology_possible(topology)) {
		pr_warn("crossbar: topology %d:%d:%d not possible\n",
			topology.disp_id, topology.protocol,
			topology.conn_inst);
		return -EINVAL;
	}

	if (is_topology_reset(topology)) {
		/* Reset topology */
		/* Destroy out_type and sor for reconfigured displays */
		for (i = 0; i < tegra_dc_get_numof_reg_disps(); i++) {
			curr = tegra_dc_get_dc(i);
			if (curr->current_topology.valid &&
				!is_topology_same(curr->current_topology,
				curr->boot_topology)) {
				/* disable the dc and output controllers */
				tegra_dc_shutdown(curr->ndev);
				if (curr->out_ops && curr->out_ops->destroy)
					curr->out_ops->destroy(curr);
			}
		}

		/* Reinitialize only reconfigured displays */
		for (i = 0; i < tegra_dc_get_numof_reg_disps(); i++) {
			curr = tegra_dc_get_dc(i);
			if (!curr->current_topology.valid ||
				!is_topology_same(curr->current_topology,
				curr->boot_topology)) {
				ret = tegra_dc_crossbar_display_reinit(curr,
					topology);
				curr->current_topology = curr->boot_topology;
				if (ret < 0)
					break;
			}
		}
	} else {
		/* Single display workflow */
		primary = tegra_dc_get_dc(topology.disp_id);
		for (i = 0; i < tegra_dc_get_numof_reg_disps(); i++) {
			curr = tegra_dc_get_dc(i);
			if (tegra_dc_or_is_dsi(curr)) {
				continue;
			} else {
				if (curr->out_ops &&
					curr->out_ops->get_connector_instance &&
					(curr->out_ops->get_connector_instance(curr)
					== topology.conn_inst)) {
					dangling = true;
					break;
				}
			}
		}

		tegra_dc_shutdown(primary->ndev);
		if (primary->out_ops && primary->out_ops->destroy)
			primary->out_ops->destroy(primary);
		if (dangling) {
			tegra_dc_shutdown(curr->ndev);
			if (curr->out_ops && curr->out_ops->destroy)
				curr->out_ops->destroy(curr);
		}
		ret = tegra_dc_crossbar_display_reinit(primary, topology);
	}

	if (ret < 0) {
		pr_warn("crossbar: switching to topology %d:%d:%d failed\n",
			topology.disp_id, topology.protocol,
			topology.conn_inst);
		return -EINVAL;
	}

	pr_info("crossbar: switching to topology %d:%d:%d success\n",
		topology.disp_id, topology.protocol, topology.conn_inst);

	return len;

}

static const struct file_operations dbg_nvdisp_topology_fops = {
	.open = dbg_nvdisp_topology_open,
	.read = seq_read,
	.write = dbg_nvdisp_topology_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_vrr_enable_show(struct seq_file *m, void *unused)
{
	struct tegra_vrr *vrr = m->private;

	if (!vrr)
		return -EINVAL;

	seq_printf(m, "vrr enable state: %d\n", vrr->enable);

	return 0;
}

static int dbg_vrr_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_vrr_enable_show, inode->i_private);
}

static const struct file_operations dbg_vrr_enable_ops = {
	.open = dbg_vrr_enable_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_vrr_dcb_show(struct seq_file *m, void *unused)
{
	struct tegra_vrr *vrr = m->private;

	if (!vrr)
		return -EINVAL;

	seq_printf(m, "vrr dc balance: %d\n", vrr->dcb);

	return 0;
}

static int dbg_vrr_dcb_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_vrr_dcb_show, inode->i_private);
}

static const struct file_operations dbg_vrr_dcb_ops = {
	.open = dbg_vrr_dcb_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_vrr_db_tolerance_show(struct seq_file *m, void *unused)
{
	struct tegra_vrr *vrr = m->private;

	if (!vrr)
		return -EINVAL;

	seq_printf(m, "vrr db tolerance: %d\n", vrr->db_tolerance);

	return 0;
}

static ssize_t dbg_vrr_db_tolerance_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_vrr *vrr = m->private;
	long   new_value;
	int    ret;

	if (!vrr)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_value);
	if (ret < 0)
		return ret;

	vrr->db_tolerance = new_value;

	return len;
}

static int dbg_vrr_db_tolerance_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_vrr_db_tolerance_show, inode->i_private);
}

static const struct file_operations dbg_vrr_db_tolerance_ops = {
	.open = dbg_vrr_db_tolerance_open,
	.read = seq_read,
	.write = dbg_vrr_db_tolerance_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_vrr_frame_avg_pct_show(struct seq_file *m, void *unused)
{
	struct tegra_vrr *vrr = m->private;

	if (!vrr)
		return -EINVAL;

	seq_printf(m, "vrr frame average percent: %d\n", vrr->frame_avg_pct);

	return 0;
}

static ssize_t dbg_vrr_frame_avg_pct_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_vrr *vrr = m->private;
	long   new_pct;
	int    ret;

	if (!vrr)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_pct);
	if (ret < 0)
		return ret;

	vrr->frame_avg_pct = new_pct;

	return len;
}

static int dbg_vrr_frame_avg_pct_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_vrr_frame_avg_pct_show, inode->i_private);
}

static const struct file_operations dbg_vrr_frame_avg_pct_ops = {
	.open = dbg_vrr_frame_avg_pct_open,
	.read = seq_read,
	.write = dbg_vrr_frame_avg_pct_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_vrr_fluct_avg_pct_show(struct seq_file *m, void *unused)
{
	struct tegra_vrr *vrr = m->private;

	if (!vrr)
		return -EINVAL;

	seq_printf(m, "vrr fluct average percent: %d\n", vrr->fluct_avg_pct);

	return 0;
}

static ssize_t dbg_vrr_fluct_avg_pct_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_vrr *vrr = m->private;
	long   new_pct;
	int    ret;

	if (!vrr)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_pct);
	if (ret < 0)
		return ret;

	vrr->fluct_avg_pct = new_pct;

	return len;
}

static int dbg_vrr_fluct_avg_pct_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_vrr_fluct_avg_pct_show, inode->i_private);
}

static const struct file_operations dbg_vrr_fluct_avg_pct_ops = {
	.open = dbg_vrr_fluct_avg_pct_open,
	.read = seq_read,
	.write = dbg_vrr_fluct_avg_pct_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_tegrahw_type_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	/* All platforms other than real silicon are taken
		as simulation */
	seq_printf(m,
		"real_silicon: %d\n",
		tegra_platform_is_silicon());

	return 0;
}

static int dbg_tegrahw_type_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_tegrahw_type_show, inode->i_private);
}

static const struct file_operations dbg_tegrahw_type_ops = {
	.open = dbg_tegrahw_type_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t dbg_background_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc *dc = m->private;
	unsigned long background;
	u32 old_state;

	if (!dc)
		return -EINVAL;

	if (kstrtoul_from_user(addr, len, 0, &background) < 0)
		return -EINVAL;

	if (!dc->enabled)
		return -EBUSY;

	tegra_dc_get(dc);
	mutex_lock(&dc->lock);
	old_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	/* write active version */
	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);
	tegra_dc_readl(dc, DC_CMD_STATE_ACCESS); /* flush */
	tegra_dc_writel(dc, background, DC_DISP_BLEND_BACKGROUND_COLOR);
	/* write assembly version */
	tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
			DC_CMD_STATE_ACCESS);
	tegra_dc_readl(dc, DC_CMD_STATE_ACCESS); /* flush */
	tegra_dc_writel(dc, background, DC_DISP_BLEND_BACKGROUND_COLOR);
	/* cycle the values through assemby -> arm -> active */
	tegra_dc_writel(dc, GENERAL_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */
	tegra_dc_writel(dc, NC_HOST_TRIG | GENERAL_ACT_REQ,
			DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, old_state, DC_CMD_STATE_ACCESS);
	tegra_dc_readl(dc, DC_CMD_STATE_ACCESS); /* flush */
	mutex_unlock(&dc->lock);
	tegra_dc_put(dc);

	return len;
}

static int dbg_background_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;
	u32 old_state;
	u32 background;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	if (!dc->enabled)
		return -EBUSY;

	tegra_dc_get(dc);
	mutex_lock(&dc->lock);
	old_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);
	background = tegra_dc_readl(dc, DC_DISP_BLEND_BACKGROUND_COLOR);
	tegra_dc_writel(dc, old_state, DC_CMD_STATE_ACCESS);
	mutex_unlock(&dc->lock);
	tegra_dc_put(dc);

	seq_printf(m, "%#x\n", (unsigned)background);

	return 0;
}

static int dbg_background_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_background_show, inode->i_private);
}

static const struct file_operations dbg_background_ops = {
	.open = dbg_background_open,
	.write = dbg_background_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* toggly the enable/disable for any windows with 1 bit set */
static ssize_t dbg_window_toggle_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc *dc = m->private;
	unsigned long windows;
	int i;
	u32 status;
	int retries;

	if (!dc)
		return -EINVAL;

	if (kstrtoul_from_user(addr, len, 0, &windows) < 0)
		return -EINVAL;

	if (!dc->enabled)
		return 0;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	/* limit the request only to valid windows */
	windows &= dc->valid_windows;
	for_each_set_bit(i, &windows, tegra_dc_get_numof_dispwindows()) {
		u32 val;
		/* select the assembly registers for window i */
		tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
				DC_CMD_STATE_ACCESS);
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		/* toggle the enable bit */
		val = tegra_dc_readl(dc, DC_WIN_WIN_OPTIONS);
		val ^= WIN_ENABLE;
		dev_dbg(&dc->ndev->dev, "%s window #%d\n",
			(val & WIN_ENABLE) ? "enabling" : "disabling", i);
		tegra_dc_writel(dc, val, DC_WIN_WIN_OPTIONS);

		/* post the update */
		tegra_dc_writel(dc, WIN_A_UPDATE << i, DC_CMD_STATE_CONTROL);
		retries = 8;
		do {
			status = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
			retries--;
		} while (retries && (status & (WIN_A_UPDATE << i)));
		tegra_dc_writel(dc, WIN_A_ACT_REQ << i, DC_CMD_STATE_CONTROL);

	}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return len;
}

/* reading shows the enabled windows */
static int dbg_window_toggle_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;
	int i;
	unsigned long windows;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	/* limit the request only to valid windows */
	windows = 0;
	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		u32 val;

		/* select the active registers for window i */
		tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
				DC_CMD_STATE_ACCESS);
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		/* add i to a bitmap if WIN_ENABLE is set */
		val = tegra_dc_readl(dc, DC_WIN_WIN_OPTIONS);
		if (val & WIN_ENABLE)
			set_bit(i, &windows);

	}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	seq_printf(m, "%#lx %#lx\n", dc->valid_windows, windows);

	return 0;
}

static int dbg_window_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_window_toggle_show, inode->i_private);
}

static const struct file_operations dbg_window_toggle_ops = {
	.open = dbg_window_toggle_open,
	.write = dbg_window_toggle_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_dc_cmu_lut1_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 val;
	int i;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	/* Disable CMU while reading LUTs */
	val = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, val & ~CMU_ENABLE, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	_tegra_dc_wait_for_frame_end(dc,
		div_s64(dc->frametime_ns, 1000000ll) * 2);

	for (i = 0; i < 256; i++) {
		tegra_dc_writel(dc, LUT1_READ_EN | LUT1_READ_ADDR(i),
			DC_COM_CMU_LUT1_READ);

		seq_printf(s, "%lu\n",
			LUT1_READ_DATA(tegra_dc_readl(dc, DC_COM_CMU_LUT1)));
	}
	tegra_dc_writel(dc, 0, DC_COM_CMU_LUT1_READ);

	tegra_dc_writel(dc, val, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return 0;
}

static int dbg_dc_cmu_lut1_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_cmu_lut1_show, inode->i_private);
}

static const struct file_operations cmu_lut1_fops = {
	.open		= dbg_dc_cmu_lut1_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_cmu_lut2_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	u32 val;
	int i;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	/* Disable CMU while reading LUTs */
	val = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, val & ~CMU_ENABLE, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	_tegra_dc_wait_for_frame_end(dc,
		div_s64(dc->frametime_ns, 1000000ll) * 2);

	for (i = 0; i < 960; i++) {
		tegra_dc_writel(dc, LUT2_READ_EN | LUT2_READ_ADDR(i),
			DC_COM_CMU_LUT2_READ);

		seq_printf(s, "%lu\n",
			LUT2_READ_DATA(tegra_dc_readl(dc, DC_COM_CMU_LUT2)));
	}
	tegra_dc_writel(dc, 0, DC_COM_CMU_LUT2_READ);

	tegra_dc_writel(dc, val, DC_DISP_DISP_COLOR_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return 0;
}

static int dbg_dc_cmu_lut2_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_cmu_lut2_show, inode->i_private);
}

static const struct file_operations cmu_lut2_fops = {
	.open		= dbg_dc_cmu_lut2_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define NVDISP_WIN_CSC_FILEOPS(name)					\
static int dbg_nvdisp_win_csc_##name##_show(struct seq_file *m, void *unused)\
{									\
	struct tegra_dc_win *win = m->private;				\
									\
	if (!win)							\
		return -EINVAL;						\
	seq_printf(m, "%u\n", win->nvdisp_win_csc.name);		\
	return 0;							\
}									\
									\
static int dbg_nvdisp_win_csc_##name##_open(struct inode *inode,	\
	struct file *file)						\
{									\
	return single_open(file, dbg_nvdisp_win_csc_##name##_show,	\
			inode->i_private);				\
}									\
									\
static ssize_t dbg_nvdisp_win_csc_##name##_write(struct file *file,	\
		const char __user *addr, size_t len, loff_t *pos)	\
{									\
	struct tegra_dc_win *win;					\
	struct seq_file *m = file->private_data;			\
	u32 user_csc;							\
									\
	win = m ? m->private : NULL;					\
	if (kstrtou32_from_user(addr, len, 10, &user_csc) < 0)		\
		return -EINVAL;						\
									\
	win->nvdisp_win_csc.name = user_csc;				\
	win->csc_dirty = 1;						\
	return len;							\
}									\
									\
static const struct file_operations dbg_nvdisp_win_csc_##name##_fops = {\
	.open		= dbg_nvdisp_win_csc_##name##_open,		\
	.read		= seq_read,					\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
	.write		= dbg_nvdisp_win_csc_##name##_write,		\
}

NVDISP_WIN_CSC_FILEOPS(r2r);
NVDISP_WIN_CSC_FILEOPS(g2r);
NVDISP_WIN_CSC_FILEOPS(b2r);
NVDISP_WIN_CSC_FILEOPS(const2r);
NVDISP_WIN_CSC_FILEOPS(r2g);
NVDISP_WIN_CSC_FILEOPS(g2g);
NVDISP_WIN_CSC_FILEOPS(b2g);
NVDISP_WIN_CSC_FILEOPS(const2g);
NVDISP_WIN_CSC_FILEOPS(r2b);
NVDISP_WIN_CSC_FILEOPS(g2b);
NVDISP_WIN_CSC_FILEOPS(b2b);
NVDISP_WIN_CSC_FILEOPS(const2b);
#undef NVDISP_WIN_CSC_FILEOPS

static int dbg_nvdisp_win_csc_force_user_csc_show(struct seq_file *m,
						void *unused)
{
	struct tegra_dc_win *win = m ? m->private : NULL;

	if (win == NULL)
		return -EINVAL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	seq_put_decimal_ll(m, '\0', win->force_user_csc);
#else
	seq_put_decimal_ll(m, "", win->force_user_csc);
#endif
	seq_putc(m, '\n');

	return 0;
}

static int dbg_nvdisp_win_csc_force_user_csc_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, dbg_nvdisp_win_csc_force_user_csc_show,
			inode->i_private);
}

static ssize_t dbg_nvdisp_win_csc_force_user_csc_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	int ret = 0;
	int force_user_csc = 0;
	struct tegra_dc_win *win;
	struct seq_file *m = file->private_data;

	win = m ? m->private : NULL;

	if (!win)
		return -EFAULT;

	ret = kstrtoint_from_user(addr, len, 10,
			&force_user_csc);
	if (ret < 0)
		return ret;
	win->force_user_csc = (force_user_csc == 0)?0:1;

	return len;
}

static const struct file_operations nvdisp_win_csc_force_user_csc_fops = {
	.open		= dbg_nvdisp_win_csc_force_user_csc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= dbg_nvdisp_win_csc_force_user_csc_write,
};

static int dbg_measure_refresh_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	seq_puts(m, "Write capture time in seconds to this node.\n");
	seq_puts(m, "Results will show up in dmesg.\n");

	return 0;
}

static ssize_t dbg_measure_refresh_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dc *dc = m->private;
	s32 seconds;
	u32 fe_count;
	int ret;
	fixed20_12 refresh_rate;
	fixed20_12 seconds_fixed;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	ret = kstrtoint_from_user(addr, len, 10, &seconds);
	if (ret < 0 || seconds < 1) {
		dev_info(&dc->ndev->dev,
				"specify integer number of seconds greater than 0\n");
		return -EINVAL;
	}

	dev_info(&dc->ndev->dev, "measuring for %d seconds\n", seconds);

	mutex_lock(&dc->lock);
	_tegra_dc_config_frame_end_intr(dc, true);
	dc->dbg_fe_count = 0;
	mutex_unlock(&dc->lock);

	msleep(1000 * seconds);

	mutex_lock(&dc->lock);
	_tegra_dc_config_frame_end_intr(dc, false);
	fe_count = dc->dbg_fe_count;
	mutex_unlock(&dc->lock);

	refresh_rate.full = dfixed_const(fe_count);
	seconds_fixed.full = dfixed_const(seconds);
	refresh_rate.full = dfixed_div(refresh_rate, seconds_fixed);

	/* Print fixed point 20.12 in decimal, truncating the 12-bit fractional
	   part to 2 decimal points */
	dev_info(&dc->ndev->dev, "refresh rate: %d.%dHz\n",
		dfixed_trunc(refresh_rate),
		dfixed_frac(refresh_rate) * 100 / 4096);

	return len;
}

static int dbg_measure_refresh_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_measure_refresh_show, inode->i_private);
}

static const struct file_operations dbg_measure_refresh_ops = {
	.open = dbg_measure_refresh_open,
	.read = seq_read,
	.write = dbg_measure_refresh_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_hw_index_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	seq_printf(m, "Hardware index: %d\n", dc->ctrl_num);

	return 0;
}

static int dbg_hw_index_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hw_index_show, inode->i_private);
}

static const struct file_operations dbg_hw_index_ops = {
	.open = dbg_hw_index_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_flip_stats_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	seq_printf(m, "Flips queued: %ld\n",
		atomic64_read(&dc->flip_stats.flips_queued));
	seq_printf(m, "Flips skipped: %ld\n",
		atomic64_read(&dc->flip_stats.flips_skipped));
	seq_printf(m, "Flips completed: %ld\n",
		atomic64_read(&dc->flip_stats.flips_cmpltd));

	return 0;
}

static int dbg_flip_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_flip_stats_show, inode->i_private);
}

static const struct file_operations dbg_flip_stats_ops = {
	.open = dbg_flip_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dbg_measure_latency_show(struct seq_file *m, void *unused)
{
	struct tegra_dc *dc = m->private;

	if (WARN_ON(!dc))
		return -EINVAL;

	mutex_lock(&dc->msrmnt_info.lock);
	seq_printf(m, "%d\n", dc->msrmnt_info.enabled);
	mutex_unlock(&dc->msrmnt_info.lock);

	return 0;
}

static ssize_t dbg_measure_latency_write(struct file *file,
		const char __user *addr, size_t len, loff_t *pos)
{
	int ret;
	int enable_val;
	struct seq_file *m = file->private_data;
	struct tegra_dc *dc = m->private;

	ret = kstrtoint_from_user(addr, len, 10, &enable_val);
	if (ret < 0)
		return ret;

	ret = tegra_dc_en_dis_latency_msrmnt_mode(dc, enable_val);

	return len;
}

static int dbg_measure_latency_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_measure_latency_show, inode->i_private);
}

static const struct file_operations dbg_measure_latency_ops = {
	.open = dbg_measure_latency_open,
	.read = seq_read,
	.write = dbg_measure_latency_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void tegra_dc_remove_debugfs(struct tegra_dc *dc)
{
	if (dc->debugdir)
		debugfs_remove_recursive(dc->debugdir);
	dc->debugdir = NULL;
	if (tegra_dc_is_nvdisplay()) {
		debugfs_remove_recursive(dc->debug_common_dir);
		dc->debug_common_dir = NULL;
	}
}

/*Create file for all elements of nvdc_nvdisp_cmu per window*/
#define CREATE_NVDISP_WIN_CSC_SYSFS(name)                               \
do {                                                                    \
	retval = debugfs_create_file(#name, 0444, wincscdir, win,    \
		&dbg_nvdisp_win_csc_##name##_fops);                     \
	if (!retval)                                                    \
		goto remove_out;                                        \
}                                                                       \
while (0)

static void tegra_dc_create_debugfs(struct tegra_dc *dc)
{
	struct dentry *retval, *vrrdir;
	struct dentry *windir, *wincscdir, *windegammadir;
	char   winname[50];
	u32 i;
	char   devname[50];

	snprintf(devname, sizeof(devname), "tegradc.%d", dc->ndev->id);
	dc->debugdir = debugfs_create_dir(devname, NULL);
	if (!dc->debugdir)
		goto remove_out;

	retval = debugfs_create_file("timestamp", 0444, dc->debugdir, dc,
		&timestamp_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("regs", 0444, dc->debugdir, dc,
		&regs_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("mode", 0444, dc->debugdir, dc,
		&mode_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("stats", 0444, dc->debugdir, dc,
		&stats_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("event_inject", 0444, dc->debugdir, dc,
		&event_inject_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("out_type", 0444, dc->debugdir, dc,
		&outtype_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("edid", 0444, dc->debugdir, dc,
		&edid_fops);
	if (!retval)
		goto remove_out;

	if (dc->out_ops->detect) {
		/* only create the file if hotplug is supported */
		retval = debugfs_create_file("hotplug", 0444, dc->debugdir,
			dc, &dbg_hotplug_fops);
		if (!retval)
			goto remove_out;
	}

	vrrdir = debugfs_create_dir("vrr",  dc->debugdir);
	if (!vrrdir)
		goto remove_out;

	retval = debugfs_create_file("enable", 0444, vrrdir,
				dc->out->vrr, &dbg_vrr_enable_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("dcb", 0444, vrrdir,
				dc->out->vrr, &dbg_vrr_dcb_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("frame_avg_pct", 0444, vrrdir,
				dc->out->vrr, &dbg_vrr_frame_avg_pct_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("fluct_avg_pct", 0444, vrrdir,
				dc->out->vrr, &dbg_vrr_fluct_avg_pct_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("tegrahw_type", 0444, dc->debugdir,
				dc, &dbg_tegrahw_type_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("background", 0444, dc->debugdir,
				dc, &dbg_background_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("window_toggle", 0444, dc->debugdir,
				dc, &dbg_window_toggle_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("cmu_lut1", 0444, dc->debugdir, dc,
		&cmu_lut1_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("cmu_lut2", 0444, dc->debugdir, dc,
		&cmu_lut2_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("measure_refresh", 0444, dc->debugdir,
				dc, &dbg_measure_refresh_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("hw_index", 0444, dc->debugdir,
				dc, &dbg_hw_index_ops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("flip_stats", 0444, dc->debugdir,
				dc, &dbg_flip_stats_ops);
	if (!retval)
		goto remove_out;

	if (dc->out_ops->get_connector_instance) {
		char sor_path[CHAR_BUF_SIZE_MAX];
		int ctrl_num = -1;

		ctrl_num = dc->out_ops->get_connector_instance(dc);
		if (ctrl_num < 0)
			goto remove_out;

		snprintf(sor_path, sizeof(sor_path),
				"/sys/kernel/debug/tegra_sor%d", ctrl_num);
		dc->sor_link = debugfs_create_symlink("sor", dc->debugdir, sor_path);
		if (!dc->sor_link)
			goto remove_out;
	}
	if (tegra_dc_is_nvdisplay()) {
		/*Create directory for elements common to all DC heads*/
		if (!dc->ndev->id) {
			dc->debug_common_dir = debugfs_create_dir(
							"tegradc.common",
							NULL);
			if (!dc->debug_common_dir)
				goto remove_out;
			for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
				struct tegra_dc_win *win = &tegra_dc_windows[i];

				snprintf(winname, sizeof(winname),
						"tegra_win.%d", i);
				windir = debugfs_create_dir(winname,
						dc->debug_common_dir);
				if (!windir)
					goto remove_out;

				retval = debugfs_create_file(
						"color_expand_enable",
						0444, windir, win,
						&dbg_color_expand_enable_fops);
				if (!retval)
					goto remove_out;

				windegammadir = debugfs_create_dir("degamma",
								   windir);
				if (!windegammadir)
					goto remove_out;

				retval = debugfs_create_file("degamma",
						0444, windegammadir, win,
						&dbg_degamma_fops);

				retval = debugfs_create_file(
						"force_user_degamma",
						0444, windegammadir, win,
						&dbg_force_user_degamma_fops);
				if (!retval)
					goto remove_out;

				wincscdir = debugfs_create_dir("csc", windir);
				if (!wincscdir)
					goto remove_out;

				CREATE_NVDISP_WIN_CSC_SYSFS(r2r);
				CREATE_NVDISP_WIN_CSC_SYSFS(g2r);
				CREATE_NVDISP_WIN_CSC_SYSFS(b2r);
				CREATE_NVDISP_WIN_CSC_SYSFS(const2r);
				CREATE_NVDISP_WIN_CSC_SYSFS(r2g);
				CREATE_NVDISP_WIN_CSC_SYSFS(g2g);
				CREATE_NVDISP_WIN_CSC_SYSFS(b2g);
				CREATE_NVDISP_WIN_CSC_SYSFS(const2g);
				CREATE_NVDISP_WIN_CSC_SYSFS(r2b);
				CREATE_NVDISP_WIN_CSC_SYSFS(g2b);
				CREATE_NVDISP_WIN_CSC_SYSFS(b2b);
				CREATE_NVDISP_WIN_CSC_SYSFS(const2b);

				/* Create file to use user-defined CSC values as
				 * override over user-space CSC
				 */
				retval = debugfs_create_file("force_user_csc",
					0444, wincscdir, win,
					&nvdisp_win_csc_force_user_csc_fops);
				if (!retval)
					goto remove_out;
			}
			retval = debugfs_create_file("nvdisp_topology",
					0444, dc->debug_common_dir, NULL,
					&dbg_nvdisp_topology_fops);
			if (!retval)
				goto remove_out;

			retval = tegra_nvdisp_create_imp_lock_debugfs(dc);
			if (!retval)
				goto remove_out;
		}
	}

	if (tegra_dc_is_nvdisplay()) {
		retval = debugfs_create_file("measure_latency", 0444,
				dc->debugdir, dc, &dbg_measure_latency_ops);
		if (!retval)
			goto remove_out;
	}

	return;

remove_out:
	dev_err(&dc->ndev->dev, "could not create debugfs\n");
	tegra_dc_remove_debugfs(dc);
}

#undef CREATE_NVDISP_WIN_CSC_SYSFS

#else /* !CONFIG_DEBUGFS */
static inline void tegra_dc_create_debugfs(struct tegra_dc *dc) { };
static inline void tegra_dc_remove_debugfs(struct tegra_dc *dc) { };
#endif /* CONFIG_DEBUGFS */

s32 tegra_dc_calc_v_front_porch(struct tegra_dc_mode *mode,
				int desired_fps)
{
	int vfp = 0;

	if (desired_fps > 0) {
		int line = mode->h_sync_width + mode->h_back_porch +
			mode->h_active + mode->h_front_porch;
		int lines_per_frame = mode->pclk / line / desired_fps;
		vfp = lines_per_frame - mode->v_sync_width -
			mode->v_active - mode->v_back_porch;
	}

	return vfp;
}

static void tegra_dc_setup_vrr(struct tegra_dc *dc)
{
	int lines_per_frame_max, lines_per_frame_min;

	struct tegra_dc_mode *m;
	struct tegra_vrr *vrr  = dc->out->vrr;

	if (!vrr)
		return;

	m = &dc->out->modes[dc->out->n_modes-1];
	vrr->v_front_porch = m->v_front_porch;
	vrr->v_back_porch = m->v_back_porch;
	vrr->pclk = m->pclk;

	if (vrr->vrr_min_fps > 0)
		vrr->v_front_porch_max = tegra_dc_calc_v_front_porch(m,
				vrr->vrr_min_fps);

	vrr->vrr_max_fps =
		(s32)div_s64(NSEC_PER_SEC, dc->frametime_ns);

	vrr->v_front_porch_min = m->v_front_porch;

	vrr->line_width = m->h_sync_width + m->h_back_porch +
			m->h_active + m->h_front_porch;
	vrr->lines_per_frame_common = m->v_sync_width +
			m->v_back_porch + m->v_active;
	lines_per_frame_max = vrr->lines_per_frame_common +
			vrr->v_front_porch_max;
	lines_per_frame_min = vrr->lines_per_frame_common +
			vrr->v_front_porch_min;

	if (lines_per_frame_max < 2*lines_per_frame_min) {
		pr_err("max fps is less than 2 times min fps.\n");
		return;
	}

	vrr->frame_len_max = vrr->line_width * lines_per_frame_max /
					(m->pclk / 1000000);
	vrr->frame_len_min = vrr->line_width * lines_per_frame_min /
					(m->pclk / 1000000);
	vrr->vfp_extend = vrr->v_front_porch_max;
	vrr->vfp_shrink = vrr->v_front_porch_min;

	vrr->frame_type = 0;
	vrr->frame_delta_us = 0;

	vrr->max_adj_pct = 50;
	vrr->max_flip_pct = 20;
	vrr->max_dcb = 20000;
	vrr->max_inc_pct = 5;

	vrr->dcb = 0;
	vrr->frame_avg_pct = 75;
	vrr->fluct_avg_pct = 75;
	vrr->db_tolerance = 5000;
}

unsigned long tegra_dc_poll_register(struct tegra_dc *dc, u32 reg, u32 mask,
		u32 exp_val, u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	if (tegra_platform_is_vdk())
		return 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
/*		usleep_range(1000, 1500);*/
		reg_val = tegra_dc_readl(dc, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;       /* success */
	dev_err(&dc->ndev->dev,
		"dc_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}


void tegra_dc_enable_general_act(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	if (tegra_dc_poll_register(dc, DC_CMD_STATE_CONTROL,
		GENERAL_ACT_REQ, 0, 1,
		TEGRA_DC_POLL_TIMEOUT_MS))
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for DC to stop\n");
}

int tegra_dc_enable_update_and_act(struct tegra_dc *dc, u32 update_mask,
							u32 act_req_mask)
{
	tegra_dc_writel(dc, update_mask, DC_CMD_STATE_CONTROL);
	tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */
	tegra_dc_writel(dc, act_req_mask, DC_CMD_STATE_CONTROL);
	tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */

	return tegra_dc_poll_register(dc, DC_CMD_STATE_CONTROL, act_req_mask,
						0, 1, TEGRA_DC_POLL_TIMEOUT_MS);

}

void tegra_dc_disable_disp_ctrl_mode(struct tegra_dc *dc)
{
	tegra_dc_get(dc);

	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);
	tegra_dc_enable_general_act(dc);

	tegra_dc_put(dc);
}

void tegra_dc_enable_disp_ctrl_mode(struct tegra_dc *dc)
{
	tegra_dc_get(dc);
	/* Enable DC */
	if (dc->out->flags & TEGRA_DC_OUT_NVSR_MODE)
		tegra_dc_writel(dc, DISP_CTRL_MODE_NC_DISPLAY,
			DC_CMD_DISPLAY_COMMAND);
	else
		tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY,
			DC_CMD_DISPLAY_COMMAND);

	tegra_dc_enable_general_act(dc);
	tegra_dc_put(dc);
}

/* Set dc at the next available index in the tegra_dcs array. */
static int tegra_dc_set(struct tegra_dc *dc)
{
	int ret = -EBUSY;
	int i;

	if (!dc)
		return -EINVAL;

	mutex_lock(&tegra_dc_lock);

	for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
		if (tegra_dcs[i] == NULL) {
			tegra_dcs[i] = dc;
			ret = i;
			break;
		}
	}

	mutex_unlock(&tegra_dc_lock);

	return ret;
}

/* Clear this dc from its current slot in the tegra_dcs array. */
static void tegra_dc_clear(struct tegra_dc *dc)
{
	int i;

	if (!dc)
		return;

	mutex_lock(&tegra_dc_lock);

	for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
		if (tegra_dcs[i] == dc) {
			tegra_dcs[i] = NULL;
			break;
		}
	}

	mutex_unlock(&tegra_dc_lock);
}

unsigned int tegra_dc_get_numof_reg_disps(void)
{
	unsigned int idx;
	unsigned int cnt = 0;
	struct tegra_dc *dc;

	mutex_lock(&tegra_dc_lock);
	for (idx = 0; idx < tegra_dc_get_numof_dispheads(); idx++)
		cnt += ((dc = tegra_dcs[idx]) != NULL) ? 1 : 0;
	mutex_unlock(&tegra_dc_lock);

	return cnt;
}

unsigned int tegra_dc_has_multiple_dc(void)
{
	unsigned int idx;
	unsigned int cnt = 0;
	struct tegra_dc *dc;

	mutex_lock(&tegra_dc_lock);
	for (idx = 0; idx < tegra_dc_get_numof_dispheads(); idx++)
		cnt += ((dc = tegra_dcs[idx]) != NULL && dc->enabled) ? 1 : 0;
	mutex_unlock(&tegra_dc_lock);

	return (cnt > 1);
}

static const char * const extcon_cable_strings[] = {
	[TEGRA_DC_OUT_HDMI] = "HDMI",
	[TEGRA_DC_OUT_DSI] = "DSI",
	[TEGRA_DC_OUT_DP] = "DP"
};

static const int hdmi_extcon_cable_id[] = {
	EXTCON_DISP_HDMI,
	EXTCON_DISP_HDMI2
};
/* the map of dc->ctrl_num to the index of tegra_hdmi_extcon_cable_id[] */
unsigned long extcon_hdmi_dc_map[ARRAY_SIZE(hdmi_extcon_cable_id)] = {
	[0 ... (ARRAY_SIZE(hdmi_extcon_cable_id) - 1)] = -1};

void tegra_dc_extcon_hpd_notify(struct tegra_dc *dc)
{
	unsigned int cable = 0;
	int i;

	mutex_lock(&tegra_dc_extcon_lock);
	if (dc && dc->out) {
		switch (dc->out->type) {
		case TEGRA_DC_OUT_HDMI:
			cable = EXTCON_NONE;
			for (i = 0; i < ARRAY_SIZE(hdmi_extcon_cable_id); i++) {
				if (extcon_hdmi_dc_map[i] == dc->ctrl_num) {
					cable = hdmi_extcon_cable_id[i];
					break;
				}
			}
			break;
		case TEGRA_DC_OUT_DP:
			cable = EXTCON_DISP_DP;
			break;
		case TEGRA_DC_OUT_DSI:
			cable = EXTCON_DISP_DSIHPD;
			break;
		default:
			mutex_unlock(&tegra_dc_extcon_lock);
			return;
		}

		if (dc->connected) {
			disp_state_extcon_switch_report(cable,
				EXTCON_DISP_HPD_STATE_ENABLED);
			pr_info("Extcon %s: HPD enabled\n",
				extcon_cable_strings[dc->out->type]);
		} else {
			disp_state_extcon_switch_report(cable,
				EXTCON_DISP_HPD_STATE_DISABLED);
			pr_info("Extcon %s: HPD disabled\n",
				extcon_cable_strings[dc->out->type]);
		}
	}
	mutex_unlock(&tegra_dc_extcon_lock);
}

/* get the stride size of a window.
 * return: stride size in bytes for window win. or 0 if unavailble. */
int tegra_dc_get_stride(struct tegra_dc *dc, unsigned win)
{
	u32 stride;

	if (!dc->enabled)
		return 0;
	WARN_ON(win > tegra_dc_get_numof_dispwindows());
	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	if (tegra_dc_is_nvdisplay()) {
		stride = tegra_nvdisp_get_linestride(dc, win);
	} else {
		tegra_dc_writel(dc, WINDOW_A_SELECT << win,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		stride = tegra_dc_readl(dc, DC_WIN_LINE_STRIDE);
	}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return GET_LINE_STRIDE(stride);
}
EXPORT_SYMBOL(tegra_dc_get_stride);

struct tegra_dc *tegra_dc_get_dc(unsigned idx)
{
	if (idx < tegra_dc_get_numof_dispheads())
		return tegra_dcs[idx];
	else
		return NULL;
}
EXPORT_SYMBOL(tegra_dc_get_dc);

struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win)
{
	if (win >= tegra_dc_get_numof_dispwindows() ||
		!test_bit(win, &dc->valid_windows))
		return NULL;

	if (tegra_dc_is_nvdisplay())
		return &tegra_dc_windows[win];
	else
		return &dc->windows[win];
}
EXPORT_SYMBOL(tegra_dc_get_window);

bool tegra_dc_get_connected(struct tegra_dc *dc)
{
	return dc->connected;
}
EXPORT_SYMBOL(tegra_dc_get_connected);

bool tegra_dc_hpd(struct tegra_dc *dc)
{
	int hpd = false;
	int hotplug_state;

	if (WARN_ON(!dc || !dc->out))
		return false;

	rmb();
	hotplug_state = dc->out->hotplug_state;

	if (hotplug_state != TEGRA_HPD_STATE_NORMAL) {
		if (hotplug_state == TEGRA_HPD_STATE_FORCE_ASSERT)
			return true;
		if (hotplug_state == TEGRA_HPD_STATE_FORCE_DEASSERT)
			return false;
	}

	if (!dc->hotplug_supported)
		return true;

	if (dc->out_ops && dc->out_ops->hpd_state)
		hpd = dc->out_ops->hpd_state(dc);

	if (dc->out->hotplug_report)
		dc->out->hotplug_report(hpd);

	return hpd;
}
EXPORT_SYMBOL(tegra_dc_hpd);

/* Used only on T21x */
static void tegra_dc_set_scaling_filter(struct tegra_dc *dc)
{
	unsigned i;
	unsigned v0 = 128;
	unsigned v1 = 0;

	/* linear horizontal and vertical filters */
	for (i = 0; i < 16; i++) {
		tegra_dc_writel(dc, (v1 << 16) | (v0 << 8),
				DC_WIN_H_FILTER_P(i));

		tegra_dc_writel(dc, v0,
				DC_WIN_V_FILTER_P(i));
		v0 -= 8;
		v1 += 8;
	}
}

static int _tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable)
{
	tegra_dc_get(dc);
	if (enable) {
		atomic_inc(&dc->frame_end_ref);
		tegra_dc_unmask_interrupt(dc, FRAME_END_INT);
	} else if (!atomic_dec_return(&dc->frame_end_ref))
		tegra_dc_mask_interrupt(dc, FRAME_END_INT);
	tegra_dc_put(dc);

	return 0;
}

static struct tegra_dc_cmu *tegra_dc_get_cmu(struct tegra_dc *dc)
{
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED ||
		dc->out->type == TEGRA_DC_OUT_NULL) {
		return &default_cmu;
	}
	if (dc->pdata->cmu && !dc->pdata->default_clr_space)
		return dc->pdata->cmu;
	else if (dc->pdata->cmu_adbRGB && dc->pdata->default_clr_space)
		return dc->pdata->cmu_adbRGB;
	else if (dc->out->type == TEGRA_DC_OUT_HDMI)
		return &default_limited_cmu;
	else
		return &default_cmu;
}

static struct tegra_dc_nvdisp_cmu *tegra_dc_get_nvdisp_cmu(struct tegra_dc *dc)
{
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED ||
		dc->out->type == TEGRA_DC_OUT_NULL) {
		tegra_nvdisp_get_default_cmu(&default_nvdisp_cmu);
		return &default_nvdisp_cmu;
	}
	if (dc->pdata->cmu && !dc->pdata->default_clr_space) {
		return dc->pdata->nvdisp_cmu;
	} else if (dc->out->type == TEGRA_DC_OUT_HDMI) {
		tegra_nvdisp_get_default_cmu(&default_limited_nvdisp_cmu);
		return &default_limited_nvdisp_cmu;
	}
	tegra_nvdisp_get_default_cmu(&default_nvdisp_cmu);
	return &default_nvdisp_cmu;
}

void tegra_dc_cmu_enable(struct tegra_dc *dc, bool cmu_enable)
{
	dc->cmu_enabled = cmu_enable;
	dc->pdata->cmu_enable = cmu_enable;
	if (tegra_dc_is_nvdisplay()) {
		tegra_dc_cache_nvdisp_cmu(dc, tegra_dc_get_nvdisp_cmu(dc));
		tegra_nvdisp_update_cmu(dc, &dc->nvdisp_postcomp_lut);
	} else if (tegra_dc_is_t21x()) {
		tegra_dc_update_cmu(dc, tegra_dc_get_cmu(dc));
	}
}
EXPORT_SYMBOL(tegra_dc_cmu_enable);

static void tegra_dc_cache_cmu(struct tegra_dc *dc,
				struct tegra_dc_cmu *src_cmu)
{
	if (&dc->cmu != src_cmu) /* ignore if it would require memmove() */
		memcpy(&dc->cmu, src_cmu, sizeof(*src_cmu));
	dc->cmu_dirty = true;
}

static void tegra_dc_set_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	u32 val;
	u32 i;

	for (i = 0; i < 256; i++) {
		val = LUT1_ADDR(i) | LUT1_DATA(cmu->lut1[i]);
		tegra_dc_writel(dc, val, DC_COM_CMU_LUT1);
	}

	tegra_dc_writel(dc, cmu->csc.krr, DC_COM_CMU_CSC_KRR);
	tegra_dc_writel(dc, cmu->csc.kgr, DC_COM_CMU_CSC_KGR);
	tegra_dc_writel(dc, cmu->csc.kbr, DC_COM_CMU_CSC_KBR);
	tegra_dc_writel(dc, cmu->csc.krg, DC_COM_CMU_CSC_KRG);
	tegra_dc_writel(dc, cmu->csc.kgg, DC_COM_CMU_CSC_KGG);
	tegra_dc_writel(dc, cmu->csc.kbg, DC_COM_CMU_CSC_KBG);
	tegra_dc_writel(dc, cmu->csc.krb, DC_COM_CMU_CSC_KRB);
	tegra_dc_writel(dc, cmu->csc.kgb, DC_COM_CMU_CSC_KGB);
	tegra_dc_writel(dc, cmu->csc.kbb, DC_COM_CMU_CSC_KBB);

	for (i = 0; i < 960; i++) {
		val = LUT2_ADDR(i) | LUT1_DATA(cmu->lut2[i]);
		tegra_dc_writel(dc, val, DC_COM_CMU_LUT2);
	}

	dc->cmu_dirty = false;
}

static void _tegra_dc_update_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	u32 val;

	if (!dc->cmu_enabled)
		return;

	tegra_dc_cache_cmu(dc, cmu);

	if (dc->cmu_dirty) {
		/* Disable CMU to avoid programming it while it is in use */
		val = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
		if (val & CMU_ENABLE) {
			val &= ~CMU_ENABLE;
			tegra_dc_writel(dc, val,
					DC_DISP_DISP_COLOR_CONTROL);
			val = GENERAL_ACT_REQ;
			tegra_dc_writel(dc, val, DC_CMD_STATE_CONTROL);
			/*TODO: Sync up with vsync */
			mdelay(20);
		}
		dev_dbg(&dc->ndev->dev, "updating CMU cmu_dirty=%d\n",
			dc->cmu_dirty);

		tegra_dc_set_cmu(dc, &dc->cmu);
	}
}

void _tegra_dc_cmu_enable(struct tegra_dc *dc, bool cmu_enable)
{
	dc->cmu_enabled = cmu_enable;
	_tegra_dc_update_cmu(dc, tegra_dc_get_cmu(dc));
	tegra_dc_set_color_control(dc);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
}
EXPORT_SYMBOL(_tegra_dc_cmu_enable);

int tegra_dc_update_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}

	tegra_dc_get(dc);

	_tegra_dc_update_cmu(dc, cmu);
	tegra_dc_set_color_control(dc);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_update_cmu);

static int _tegra_dc_update_cmu_aligned(struct tegra_dc *dc,
				struct tegra_dc_cmu *cmu,
				bool force)
{
	memcpy(&dc->cmu_shadow, cmu, sizeof(dc->cmu));
	dc->cmu_shadow_dirty = true;
	dc->cmu_shadow_force_update = dc->cmu_shadow_force_update || force;
	_tegra_dc_config_frame_end_intr(dc, true);

	return 0;
}

int tegra_dc_update_cmu_aligned(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	int ret;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}
	ret = _tegra_dc_update_cmu_aligned(dc, cmu, false);
	mutex_unlock(&dc->lock);

	return ret;
}

EXPORT_SYMBOL(tegra_dc_update_cmu_aligned);

int tegra_dc_set_hdr(struct tegra_dc *dc, struct tegra_dc_hdr *hdr,
						bool cache_dirty)
{
	int ret;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}
	trace_hdr_data_update(dc, hdr);
	if (cache_dirty) {
		dc->hdr.eotf = hdr->eotf;
		dc->hdr.static_metadata_id = hdr->static_metadata_id;
		memcpy(dc->hdr.static_metadata, hdr->static_metadata,
					sizeof(dc->hdr.static_metadata));
	} else if (dc->hdr.enabled == hdr->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}
	dc->hdr.enabled = hdr->enabled;
	dc->hdr_cache_dirty = true;
	if (!dc->hdr.enabled)
		memset(&dc->hdr, 0, sizeof(dc->hdr));
	ret = _tegra_dc_config_frame_end_intr(dc, true);

	mutex_unlock(&dc->lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dc_set_hdr);

/* disable_irq() blocks until handler completes, calling this function while
 * holding dc->lock can deadlock. */
static inline void disable_dc_irq(const struct tegra_dc *dc)
{
	disable_irq(dc->irq);
}

u32 tegra_dc_get_syncpt_id(struct tegra_dc *dc, int i)
{
	struct tegra_dc_win *win = tegra_dc_get_window(dc, i);
	BUG_ON(!win);
	return win->syncpt.id;
}
EXPORT_SYMBOL(tegra_dc_get_syncpt_id);

static u32 tegra_dc_incr_syncpt_max_locked(struct tegra_dc *dc, int i)
{
	u32 max;
	struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

	BUG_ON(!win);
	max = nvhost_syncpt_incr_max_ext(dc->ndev,
		win->syncpt.id, ((dc->enabled) ? 1 : 0));
	win->syncpt.max = max;

	return max;
}

u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc, int i)
{
	u32 max;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	max = tegra_dc_incr_syncpt_max_locked(dc, i);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return max;
}

void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, int i, u32 val)
{
	struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

	BUG_ON(!win);
	mutex_lock(&dc->lock);

	tegra_dc_get(dc);
	while (win->syncpt.min < val) {
		win->syncpt.min++;
		nvhost_syncpt_cpu_incr_ext(dc->ndev, win->syncpt.id);
		}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

struct sync_fence *tegra_dc_create_fence(struct tegra_dc *dc, int i, u32 val)
{
	struct nvhost_ctrl_sync_fence_info syncpt;
	u32 id = tegra_dc_get_syncpt_id(dc, i);

	syncpt.id = id;
	syncpt.thresh = val;
	return nvhost_sync_create_fence(
			to_platform_device(dc->ndev->dev.parent),
			&syncpt, 1, dev_name(&dc->ndev->dev));
}

void
tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg)
{
	unsigned int ctrl;
	unsigned long out_sel;
	unsigned long cmd_state;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	ctrl = ((cfg->period << PM_PERIOD_SHIFT) |
		(cfg->clk_div << PM_CLK_DIVIDER_SHIFT) |
		cfg->clk_select);

	/* The new value should be effected immediately */
	cmd_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, (cmd_state | (1 << 2)), DC_CMD_STATE_ACCESS);

	switch (cfg->which_pwm) {
	case TEGRA_PWM_PM0:
		/* Select the LM0 on PM0 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 0);
		out_sel |= (3 << 0);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM0_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM0_DUTY_CYCLE);
		break;
	case TEGRA_PWM_PM1:
		/* Select the LM1 on PM1 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 4);
		out_sel |= (3 << 4);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM1_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM1_DUTY_CYCLE);
		break;
	default:
		dev_err(&dc->ndev->dev, "Error: Need which_pwm\n");
		break;
	}
	tegra_dc_writel(dc, cmd_state, DC_CMD_STATE_ACCESS);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_config_pwm);

void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins)
{
	unsigned int i;

	int name;
	int pol;

	u32 pol1, pol3;

	u32 set1, unset1;
	u32 set3, unset3;

	set1 = set3 = unset1 = unset3 = 0;

	for (i = 0; i < n_pins; i++) {
		name = (pins + i)->name;
		pol  = (pins + i)->pol;

		/* set polarity by name */
		switch (name) {
		case TEGRA_DC_OUT_PIN_DATA_ENABLE:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set3 |= LSPI_OUTPUT_POLARITY_LOW;
			else
				unset3 |= LSPI_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_H_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LHS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LHS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_V_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LVS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LVS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_PIXEL_CLOCK:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LSC0_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LSC0_OUTPUT_POLARITY_LOW;
			break;
		default:
			printk("Invalid argument in function %s\n",
			       __FUNCTION__);
			break;
		}
	}

	pol1 = DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL;
	pol3 = DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL;

	pol1 |= set1;
	pol1 &= ~unset1;

	pol3 |= set3;
	pol3 &= ~unset3;

	tegra_dc_writel(dc, pol1, DC_COM_PIN_OUTPUT_POLARITY1);
	tegra_dc_writel(dc, pol3, DC_COM_PIN_OUTPUT_POLARITY3);
}

static struct tegra_dc_mode *tegra_dc_get_override_mode(struct tegra_dc *dc)
{
	unsigned long refresh;

	if (((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		(dc->out->type == TEGRA_DC_OUT_DP)) &&
		   tegra_is_bl_display_initialized(dc->ctrl_num)) {

		/* For seamless HDMI, read mode parameters from bootloader
		 * set DC configuration
		 */
		u32 val = 0;
		struct tegra_dc_mode *mode = &override_disp_mode[dc->out->type];
		struct clk *parent_clk = NULL;

		if (tegra_dc_is_nvdisplay())
			parent_clk = tegra_disp_clk_get(&dc->ndev->dev,
					dc->out->parent_clk ? : "plld2");
		else
			parent_clk = clk_get_sys(NULL,
					dc->out->parent_clk ? : "pll_d2");

		memset(mode, 0, sizeof(struct tegra_dc_mode));
		mode->pclk = clk_get_rate(parent_clk);
		mode->rated_pclk = 0;

		tegra_dc_get(dc);

		/* {V,H}_REF_TO_SYNC do NOT exist on nvdisplay. */
		if (!tegra_dc_is_nvdisplay()) {
			val = tegra_dc_readl(dc, DC_DISP_REF_TO_SYNC);
			mode->h_ref_to_sync = val & 0xffff;
			mode->v_ref_to_sync = (val >> 16) & 0xffff;
		}

		val = tegra_dc_readl(dc, DC_DISP_SYNC_WIDTH);
		mode->h_sync_width = val & 0xffff;
		mode->v_sync_width = (val >> 16) & 0xffff;

		val = tegra_dc_readl(dc, DC_DISP_BACK_PORCH);
		mode->h_back_porch = val & 0xffff;
		mode->v_back_porch = (val >> 16) & 0xffff;

		val = tegra_dc_readl(dc, DC_DISP_FRONT_PORCH);
		mode->h_front_porch = val & 0xffff;
		mode->v_front_porch = (val >> 16) & 0xffff;

		val = tegra_dc_readl(dc, DC_DISP_DISP_ACTIVE);
		mode->h_active = val & 0xffff;
		mode->v_active = (val >> 16) & 0xffff;

		/* Check the freq setup by the BL, 59.94 or 60Hz
		 * If 59.94, vmode needs to be FB_VMODE_1000DIV1001
		 * for seamless
		 */
		refresh = tegra_dc_calc_refresh(mode);
		if (refresh % 1000)
			mode->vmode |= FB_VMODE_1000DIV1001;

		/*
		 * Implicit contract between BL and us. If CMU is enabled,
		 * assume limited range. This sort of works because we know
		 * BL doesn't support YUV
		 */
		val = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
		if (val & CMU_ENABLE)
			mode->vmode |= FB_VMODE_LIMITED_RANGE;

		tegra_dc_put(dc);
	}

	if (dc->out->type == TEGRA_DC_OUT_RGB  ||
		dc->out->type == TEGRA_DC_OUT_HDMI ||
		dc->out->type == TEGRA_DC_OUT_DP ||
		dc->out->type == TEGRA_DC_OUT_DSI  ||
		dc->out->type == TEGRA_DC_OUT_NULL)
		return override_disp_mode[dc->out->type].pclk ?
			&override_disp_mode[dc->out->type] : NULL;
	else
		return NULL;
}

static int tegra_dc_set_out(struct tegra_dc *dc, struct tegra_dc_out *out,
		bool initialized)
{
	struct tegra_dc_mode *mode = NULL;
	int err = 0;
	int i, free_slot = -1;

	dc->out = out;
	dc->hotplug_supported = tegra_dc_hotplug_supported(dc);

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {
		for (i = 0; i < ARRAY_SIZE(hdmi_extcon_cable_id); i++) {
			/* bail out if the map has already been done */
			if (extcon_hdmi_dc_map[i] == dc->ctrl_num) {
				free_slot = i;
				break;
			} else if (extcon_hdmi_dc_map[i] == -1) {
				free_slot = i;
			}
		}

		if (unlikely(free_slot == -1)) {
			dev_err(&dc->ndev->dev,
				"No free extcon HDMI slot for DC %d\n",
				dc->ctrl_num);
		} else {
			extcon_hdmi_dc_map[free_slot] = dc->ctrl_num;
		}
	}

	if (initialized) {
		dc->initialized = false;
		goto bypass_init_check;
	}

	if (((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		(dc->out->type == TEGRA_DC_OUT_DP)) &&
			tegra_is_bl_display_initialized(dc->ctrl_num)) {
		/*
		 * Bootloader enables clk and host1x in seamless
		 * usecase. Below extra reference accounts for it
		 */
		tegra_dc_get(dc);
	}
/*
 * This config enables seamless feature only for
 * android usecase as a WAR for improper DSI initialization
 * in bootloader for L4T usecase.
 * Bug 200122858
 */
#ifdef CONFIG_ANDROID
	/*
	 * Seamless supporting panels can work in seamless mode
	 * only if BL initializes DC/DSI. If not, panel should
	 * go with complete initialization.
	 */
	if (dc->out->type == TEGRA_DC_OUT_DSI &&
			!tegra_is_bl_display_initialized(dc->ctrl_num)) {
		dc->initialized = false;
	} else if (dc->out->type == TEGRA_DC_OUT_DSI &&
			tegra_is_bl_display_initialized(dc->ctrl_num)) {
		/*
		 * In case of dsi->csi loopback support, force re-initialize
		 * all DSI controllers. So, set dc->initialized to false.
		 */
		if (dc->out->dsi->dsi_csi_loopback)
			dc->initialized = false;
		else
			dc->initialized = true;
	}
#endif
	mode = tegra_dc_get_override_mode(dc);
bypass_init_check:
	if (mode && tegra_is_bl_display_initialized(dc->ctrl_num)) {
		tegra_dc_set_mode(dc, mode);

		/*
		 * Bootloader should and should only pass disp_params if
		 * it has initialized display controller.  Whenever we see
		 * override modes, we should skip things cause display resets.
		 */
		dev_info(&dc->ndev->dev, "Bootloader disp_param detected. "
				"Detected mode: %dx%d (on %dx%dmm) pclk=%d\n",
				dc->mode.h_active, dc->mode.v_active,
				dc->out->h_size, dc->out->v_size,
				dc->mode.pclk);
		dc->initialized = true;
	} else if (out->n_modes > 0) {
		/* For VRR panels, default mode is first in the list,
		 * and native panel mode is the last.
		 * Initialization must occur using the native panel mode. */
		if (dc->out->vrr) {
			tegra_dc_set_mode(dc,
				&dc->out->modes[dc->out->n_modes-1]);
			tegra_dc_setup_vrr(dc);
		} else
			tegra_dc_set_mode(dc, &dc->out->modes[0]);
	}

	switch (out->type) {
	case TEGRA_DC_OUT_HDMI:
#if	defined(CONFIG_TEGRA_HDMI2_0)
		dc->out_ops = &tegra_dc_hdmi2_0_ops;
#endif
		break;

	case TEGRA_DC_OUT_DSI:
	case TEGRA_DC_OUT_FAKE_DSIA:
	case TEGRA_DC_OUT_FAKE_DSIB:
	case TEGRA_DC_OUT_FAKE_DSI_GANGED:
		dc->out_ops = &tegra_dc_dsi_ops;
		break;

#ifdef CONFIG_TEGRA_DP
	case TEGRA_DC_OUT_FAKE_DP:
	case TEGRA_DC_OUT_DP:
		dc->out_ops = &tegra_dc_dp_ops;
		break;
#endif
#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
	case TEGRA_DC_OUT_NULL:
		break;
#endif /*CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT*/

	default:
		dc->out_ops = NULL;
		break;
	}

	if (tegra_dc_is_t21x())
		tegra_dc_cache_cmu(dc, tegra_dc_get_cmu(dc));

	if (dc->out_ops && dc->out_ops->init) {
		err = dc->out_ops->init(dc);
		if (err < 0) {
			dc->out = NULL;
			dc->out_ops = NULL;
			dev_err(&dc->ndev->dev,
				"Error: out->type:%d out_ops->init() failed. err=%d\n",
				out->type, err);
			return err;
		}
	}

	return err;
}

void tegra_dc_out_destroy(struct tegra_dc *dc)
{
	if (dc->out->hdmi_out) {
		if (dc->out->hdmi_out->spd_infoframe)
			devm_kfree(&dc->ndev->dev,
				dc->out->hdmi_out->spd_infoframe);
		dc->out->hdmi_out->spd_infoframe = NULL;
		devm_kfree(&dc->ndev->dev, dc->out->hdmi_out);
	}
	dc->out->hdmi_out = NULL;

	if (dc->out->dp_out)
		devm_kfree(&dc->ndev->dev, dc->out->dp_out);
	dc->out->dp_out = NULL;

	if (dc->out->modes)
		devm_kfree(&dc->ndev->dev, dc->out->modes);
	dc->out->modes = NULL;
	dc->out->n_modes = 0;

	if (dc->out->vrr)
		devm_kfree(&dc->ndev->dev, dc->out->vrr);
	dc->out->vrr = NULL;

	if (dc->out->out_pins)
		devm_kfree(&dc->ndev->dev, dc->out->out_pins);
	dc->out->out_pins = NULL;

	tegra_panel_unregister_ops(dc->out);
}

int tegra_dc_get_head(const struct tegra_dc *dc)
{
	if (dc)
		return dc->ctrl_num;
	return -EINVAL;
}

/* returns on error: -EINVAL
 * on success: TEGRA_DC_OUT_RGB, TEGRA_DC_OUT_HDMI, ... */
int tegra_dc_get_out(const struct tegra_dc *dc)
{
	if (dc && dc->out)
		return dc->out->type;
	return -EINVAL;
}

int tegra_dc_get_source_physical_address(u8 *phy_address)
{
	int i;
	struct tegra_dc *dc;

	if (!phy_address)
		return -EFAULT;

	for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
		dc = tegra_dc_get_dc(i);

		if (dc && dc->enabled && dc->edid && dc->out &&
			(dc->out->type == TEGRA_DC_OUT_HDMI))
			return tegra_edid_get_source_physical_address(dc->edid,
				phy_address);
	}
	return -ENODEV;

}

bool tegra_dc_is_ext_panel(const struct tegra_dc *dc)
{
	if (dc && dc->out)
		return dc->out->is_ext_panel;
	return false;
}

unsigned tegra_dc_get_out_height(const struct tegra_dc *dc)
{
	unsigned height = 0;

	if (dc->out) {
		if (dc->out->height)
			height = dc->out->height;
		else if (dc->out->h_size && dc->out->v_size)
			height = dc->out->v_size;
	}

	return height;
}
EXPORT_SYMBOL(tegra_dc_get_out_height);

unsigned tegra_dc_get_out_width(const struct tegra_dc *dc)
{
	unsigned width = 0;

	if (dc->out) {
		if (dc->out->width)
			width = dc->out->width;
		else if (dc->out->h_size && dc->out->v_size)
			width = dc->out->h_size;
	}

	return width;
}
EXPORT_SYMBOL(tegra_dc_get_out_width);

unsigned tegra_dc_get_out_max_pixclock(const struct tegra_dc *dc)
{
	if (dc && dc->out)
		return dc->out->max_pixclock;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_max_pixclock);

/*
 * Check if mode's pixel clock requirement can be satisfied. Note that
 * the pixclock value is in pico seconds.
 */
bool tegra_dc_valid_pixclock(const struct tegra_dc *dc,
					const struct fb_videomode *mode)
{
	unsigned max_pixclock = tegra_dc_get_out_max_pixclock(dc);

	if (max_pixclock)
		return mode->pixclock >= max_pixclock;
	else
		return true;
}
EXPORT_SYMBOL(tegra_dc_valid_pixclock);

void tegra_dc_sysfs_enable_crc(struct tegra_dc *dc)
{
	u32 val;

	if (dc->crc_ref_cnt.legacy) {
		pr_err("CRC is already enabled.\n");
		return;
	}

	dc->crc_ref_cnt.legacy = true;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	val = CRC_ALWAYS_ENABLE | CRC_INPUT_DATA_ACTIVE_DATA |
		CRC_ENABLE_ENABLE;
	tegra_dc_writel(dc, val, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Register a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, true);
}

void tegra_dc_sysfs_disable_crc(struct tegra_dc *dc)
{
	if (!dc->crc_ref_cnt.legacy) {
		pr_err("CRC is already disabled.\n");
		return;
	}

	dc->crc_ref_cnt.legacy = false;

	/* Unregister a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, false);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, 0x0, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

u32 tegra_dc_sysfs_read_checksum_latched(struct tegra_dc *dc)
{
	int crc = 0;

	if (!dc) {
		pr_err("Failed to get dc: NULL parameter.\n");
		goto crc_error;
	}

	if (!dc->crc_ref_cnt.legacy) {
		pr_err("CRC is not enabled.\n");
		goto crc_error;
	}

	/* If gated quitely return */
	if (!tegra_dc_is_powered(dc))
		return 0;

	reinit_completion(&dc->crc_complete);
	if (dc->crc_pending &&
	    wait_for_completion_interruptible(&dc->crc_complete)) {
		pr_err("CRC read interrupted.\n");
		goto crc_error;
	}

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	crc = tegra_dc_readl(dc, DC_COM_CRC_CHECKSUM_LATCHED);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
crc_error:
	return crc;
}
EXPORT_SYMBOL(tegra_dc_sysfs_read_checksum_latched);

bool tegra_dc_windows_are_dirty(struct tegra_dc *dc, u32 win_act_req_mask)
{
	u32 val;

	if (tegra_platform_is_vdk())
		return false;

	val = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
	if (val & (win_act_req_mask))
		return true;

	return false;
}

static inline void __maybe_unused
enable_dc_irq(const struct tegra_dc *dc)
{
	if (tegra_platform_is_fpga())
		/* Always disable DC interrupts on FPGA. */
		disable_irq(dc->irq);
	else
		enable_irq(dc->irq);
}

/* assumes dc->lock is already taken. */
static void _tegra_dc_vsync_enable(struct tegra_dc *dc)
{
	int vsync_irq;

	if (test_bit(V_BLANK_USER, &dc->vblank_ref_count))
		return; /* already set, nothing needs to be done */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		vsync_irq = MSF_INT;
	else
		vsync_irq = V_BLANK_INT;
	tegra_dc_hold_dc_out(dc);
	set_bit(V_BLANK_USER, &dc->vblank_ref_count);
	tegra_dc_unmask_interrupt(dc, vsync_irq);
}

int tegra_dc_vsync_enable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	if (dc->enabled) {
		_tegra_dc_vsync_enable(dc);
		mutex_unlock(&dc->lock);
		return 0;
	}
	mutex_unlock(&dc->lock);
	return 1;
}

/* assumes dc->lock is already taken. */
static void _tegra_dc_vsync_disable(struct tegra_dc *dc)
{
	int vsync_irq;

	if (!test_bit(V_BLANK_USER, &dc->vblank_ref_count))
		return; /* already clear, nothing needs to be done */
	if (dc->out->type == TEGRA_DC_OUT_DSI)
		vsync_irq = MSF_INT;
	else
		vsync_irq = V_BLANK_INT;
	clear_bit(V_BLANK_USER, &dc->vblank_ref_count);
	if (!dc->vblank_ref_count)
		tegra_dc_mask_interrupt(dc, vsync_irq);
	tegra_dc_release_dc_out(dc);
}

void tegra_dc_vsync_disable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	_tegra_dc_vsync_disable(dc);
	mutex_unlock(&dc->lock);
}

bool tegra_dc_has_vsync(struct tegra_dc *dc)
{
	return true;
}

/* assumes dc->lock is already taken. */
static void _tegra_dc_user_vsync_enable(struct tegra_dc *dc, bool enable)
{
	if (enable) {
		dc->out->user_needs_vblank++;
		init_completion(&dc->out->user_vblank_comp);
		_tegra_dc_vsync_enable(dc);
	} else {
		_tegra_dc_vsync_disable(dc);
		if (dc->out->user_needs_vblank > 0)
			dc->out->user_needs_vblank--;
	}
}

int tegra_dc_wait_for_vsync(struct tegra_dc *dc)
{
	unsigned long timeout_ms;
	unsigned long refresh; /* in 1000th Hz */
	int ret;

	mutex_lock(&dc->lp_lock);
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		ret = -ENOTTY;
		goto out;
	}
	refresh = tegra_dc_calc_refresh(&dc->mode);
	if (refresh == 0) {
		dev_err(&dc->ndev->dev, "dc:refresh is %lu\n", refresh);
		ret = -ERANGE;
		goto out;
	}
	/* time out if waiting took more than 2 frames */
	timeout_ms = DIV_ROUND_UP(2 * 1000000, refresh);
	_tegra_dc_user_vsync_enable(dc, true);
	mutex_unlock(&dc->lock);
	ret = wait_for_completion_interruptible_timeout(
		&dc->out->user_vblank_comp, msecs_to_jiffies(timeout_ms));
	mutex_lock(&dc->lock);
	_tegra_dc_user_vsync_enable(dc, false);
out:
	mutex_unlock(&dc->lock);
	mutex_unlock(&dc->lp_lock);
	return ret;
}

int _tegra_dc_wait_for_frame_end(struct tegra_dc *dc,
	u32 timeout_ms)
{
	int ret;

	reinit_completion(&dc->frame_end_complete);

	tegra_dc_get(dc);

	tegra_dc_flush_interrupt(dc, FRAME_END_INT);
	/* unmask frame end interrupt */
	_tegra_dc_config_frame_end_intr(dc, true);

	ret = wait_for_completion_interruptible_timeout(
			&dc->frame_end_complete,
			msecs_to_jiffies(timeout_ms));

	_tegra_dc_config_frame_end_intr(dc, false);

	tegra_dc_put(dc);

	return ret;
}

void tegra_dc_set_act_vfp(struct tegra_dc *dc, int vfp)
{
	WARN_ON(!mutex_is_locked(&dc->lock));
	WARN_ON(!tegra_dc_is_nvdisplay() && vfp < dc->mode.v_ref_to_sync + 1);
	/* It's very unlikely that active vfp will need to
	 * be changed outside of vrr context */
	WARN_ON(!dc->out->vrr || !dc->out->vrr->capability);

	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, dc->mode.h_front_porch |
			(vfp << 16), DC_DISP_FRONT_PORCH);
	tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
			DC_CMD_STATE_ACCESS);
}

static void tegra_dc_vrr_extend_vfp(struct tegra_dc *dc)
{
	struct tegra_vrr *vrr  = dc->out->vrr;

	if (!vrr || !vrr->capability)
		return;

	if (!vrr->enable)
		return;

	tegra_dc_set_act_vfp(dc, MAX_VRR_V_FRONT_PORCH);
}

int tegra_dc_get_v_count(struct tegra_dc *dc)
{
	u32     value;

	value = tegra_dc_readl(dc, DC_DISP_DISPLAY_DBG_TIMING);
	return (value & DBG_V_COUNT_MASK) >> DBG_V_COUNT_SHIFT;
}

static void tegra_dc_vrr_get_ts(struct tegra_dc *dc)
{
	struct timespec time_now;
	struct tegra_vrr *vrr  = dc->out->vrr;

	if (!vrr || !vrr->capability ||
		(!vrr->enable && !vrr->lastenable))
		return;

	getnstimeofday(&time_now);
	vrr->fe_time_us = (s64)time_now.tv_sec * 1000000 +
				time_now.tv_nsec / 1000;
	vrr->v_count = tegra_dc_get_v_count(dc);
}

static void tegra_dc_vrr_sec(struct tegra_dc *dc)
{
	struct tegra_vrr *vrr  = dc->out->vrr;

	if (!vrr || !vrr->capability)
		return;

	if (!vrr->enable && !vrr->fe_intr_req)
		return;

	if (tegra_dc_is_nvdisplay())
		cancel_delayed_work_sync(&dc->vrr_work);

	/* Decrement frame end interrupt refcount previously
	   requested by secure library */
	if (vrr->fe_intr_req) {
		_tegra_dc_config_frame_end_intr(dc, false);
		vrr->fe_intr_req = 0;
	}

#if defined(CONFIG_TEGRA_HDMIVRR) && (defined(CONFIG_TRUSTED_LITTLE_KERNEL) || defined(CONFIG_OTE_TRUSTY))
	if (te_is_secos_dev_enabled())
		tegra_hdmivrr_te_vrr_sec(vrr);
#endif

	/* Increment frame end interrupt refcount requested
	   by secure library */
	if (vrr->fe_intr_req)
		_tegra_dc_config_frame_end_intr(dc, true);

	if (tegra_dc_is_nvdisplay()) {
		if (vrr->insert_frame)
			schedule_delayed_work(&dc->vrr_work,
				msecs_to_jiffies(vrr->insert_frame/1000));
	}
}

static void tegra_dc_vblank(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vblank_work);

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	/* Clear the V_BLANK_FLIP bit of vblank ref-count if update is clean. */
	if (!tegra_dc_windows_are_dirty(dc, WIN_ALL_ACT_REQ))
		clear_bit(V_BLANK_FLIP, &dc->vblank_ref_count);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

#define CSC_UPDATE_IF_CHANGED(entry, ENTRY) do { \
		if (cmu_active->csc.entry != cmu_shadow->csc.entry || \
			dc->cmu_shadow_force_update) { \
			cmu_active->csc.entry = cmu_shadow->csc.entry; \
			tegra_dc_writel(dc, \
				cmu_active->csc.entry, \
				DC_COM_CMU_CSC_##ENTRY); \
		} \
	} while (0)

static void _tegra_dc_handle_hdr(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	if (dc->out_ops->set_hdr)
		dc->out_ops->set_hdr(dc);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return;
}

static void tegra_dc_frame_end(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work,
		struct tegra_dc, frame_end_work);
	u32 val;
	u32 i;

	if (tegra_dc_is_t21x()) {
		mutex_lock(&dc->lock);

		if (!dc->enabled) {
			mutex_unlock(&dc->lock);
			return;
		}

		tegra_dc_get(dc);

		if (dc->cmu_shadow_dirty) {
			struct tegra_dc_cmu *cmu_active = &dc->cmu;
			struct tegra_dc_cmu *cmu_shadow = &dc->cmu_shadow;

			for (i = 0; i < 256; i++) {
				if (cmu_active->lut1[i] != cmu_shadow->lut1[i]
						||
						dc->cmu_shadow_force_update) {
					cmu_active->lut1[i] =
							cmu_shadow->lut1[i];
					val = LUT1_ADDR(i) |
						LUT1_DATA(cmu_shadow->lut1[i]);
					tegra_dc_writel(dc, val,
							DC_COM_CMU_LUT1);
				}
			}

			CSC_UPDATE_IF_CHANGED(krr, KRR);
			CSC_UPDATE_IF_CHANGED(kgr, KGR);
			CSC_UPDATE_IF_CHANGED(kbr, KBR);
			CSC_UPDATE_IF_CHANGED(krg, KRG);
			CSC_UPDATE_IF_CHANGED(kgg, KGG);
			CSC_UPDATE_IF_CHANGED(kbg, KBG);
			CSC_UPDATE_IF_CHANGED(krb, KRB);
			CSC_UPDATE_IF_CHANGED(kgb, KGB);
			CSC_UPDATE_IF_CHANGED(kbb, KBB);

			for (i = 0; i < 960; i++)
				if (cmu_active->lut2[i] != cmu_shadow->lut2[i]
						||
						dc->cmu_shadow_force_update) {
					cmu_active->lut2[i] =
							cmu_shadow->lut2[i];
					val = LUT2_ADDR(i) |
						LUT2_DATA(cmu_active->lut2[i]);
					tegra_dc_writel(dc, val,
							DC_COM_CMU_LUT2);
				}

			dc->cmu_shadow_dirty = false;
			dc->cmu_shadow_force_update = false;
			_tegra_dc_config_frame_end_intr(dc, false);
		}

		tegra_dc_put(dc);
		mutex_unlock(&dc->lock);
	}
	if (dc->hdr_cache_dirty) {
		_tegra_dc_handle_hdr(dc);
		_tegra_dc_config_frame_end_intr(dc, false);
		dc->hdr_cache_dirty = false;
	}

	return;
}

static void tegra_dc_one_shot_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, one_shot_work);
	mutex_lock(&dc->lock);

	/* memory client has gone idle */
	tegra_dc_clear_bandwidth(dc);

	if (dc->out_ops && dc->out_ops->idle) {
		tegra_dc_io_start(dc);
		dc->out_ops->idle(dc);
		tegra_dc_io_end(dc);
	}

	mutex_unlock(&dc->lock);
}

/* return an arbitrarily large number if count overflow occurs.
 * make it a nice base-10 number to show up in stats output */
static u64 tegra_dc_underflow_count(struct tegra_dc *dc, unsigned reg)
{
	unsigned count = tegra_dc_readl(dc, reg);

	tegra_dc_writel(dc, 0, reg);
	return ((count & 0x80000000) == 0) ? count : 10000000000ll;
}

static void tegra_dc_underflow_handler(struct tegra_dc *dc)
{
	if (tegra_dc_is_t21x()) {
		const u32 masks[] = {
			WIN_A_UF_INT,
			WIN_B_UF_INT,
			WIN_C_UF_INT,
			WIN_D_UF_INT,
			HC_UF_INT,
			WIN_T_UF_INT,
		};
		int i;

		dc->stats.underflows++;
		if (dc->underflow_mask & WIN_A_UF_INT)
			dc->stats.underflows_a += tegra_dc_underflow_count(dc,
					DC_WINBUF_AD_UFLOW_STATUS);
		if (dc->underflow_mask & WIN_B_UF_INT)
			dc->stats.underflows_b += tegra_dc_underflow_count(dc,
					DC_WINBUF_BD_UFLOW_STATUS);
		if (dc->underflow_mask & WIN_C_UF_INT)
			dc->stats.underflows_c += tegra_dc_underflow_count(dc,
					DC_WINBUF_CD_UFLOW_STATUS);
		if (dc->underflow_mask & HC_UF_INT)
			dc->stats.underflows_h += tegra_dc_underflow_count(dc,
					DC_WINBUF_HD_UFLOW_STATUS);
		if (dc->underflow_mask & WIN_D_UF_INT)
			dc->stats.underflows_d += tegra_dc_underflow_count(dc,
					DC_WINBUF_DD_UFLOW_STATUS);
		if (dc->underflow_mask & WIN_T_UF_INT)
			dc->stats.underflows_t += tegra_dc_underflow_count(dc,
					DC_WINBUF_TD_UFLOW_STATUS);

		/* Check for any underflow reset conditions */
		for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
			struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

			if (WARN_ONCE(i >= ARRAY_SIZE(masks),
						"underflow stats unsupported"))
				/* bail if the table above is missing entries */
				break;
			if (!masks[i])
				continue; /* skip empty entries */

			if (dc->underflow_mask & masks[i])
				win->underflows++;
			else
				win->underflows = 0;
		}
	} else {
		tegra_nvdisp_underflow_handler(dc);
	}

	/* Clear the underflow mask now that we've checked it. */
	tegra_dc_writel(dc, dc->underflow_mask, DC_CMD_INT_STATUS);
	dc->underflow_mask = 0;
	tegra_dc_unmask_interrupt(dc, ALL_UF_INT());
	trace_underflow(dc);
}

static void tegra_dc_vpulse2(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vpulse2_work);

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	/* Clear the V_PULSE2_FLIP if no update */
	if (!tegra_dc_windows_are_dirty(dc, WIN_ALL_ACT_REQ))
		clear_bit(V_PULSE2_FLIP, &dc->vpulse2_ref_count);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

static void tegra_dc_process_vblank(struct tegra_dc *dc)
{
	/* pending user vblank, so wakeup */
	if (dc->out->user_needs_vblank) {
		dc->out->user_needs_vblank = false;
		complete(&dc->out->user_vblank_comp);
	}
	if (test_bit(V_BLANK_USER, &dc->vblank_ref_count)) {
		u64 timestamp = tegra_dc_get_scanline_timestamp(dc,
					dc->mode_metadata.vblank_lines);
		if (unlikely(!timestamp))
			dev_err(&dc->ndev->dev, "Invalid Timestamp Value\n");

		tegra_dc_ext_process_vblank(dc->ndev->id, timestamp);
	}
}

int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable)
{
	int ret;

	mutex_lock(&dc->lock);
	ret = _tegra_dc_config_frame_end_intr(dc, enable);
	mutex_unlock(&dc->lock);

	return ret;
}

static void tegra_dc_one_shot_irq(struct tegra_dc *dc, unsigned long status)
{
	if (status & MSF_INT)
		tegra_dc_process_vblank(dc);

	if (status & V_BLANK_INT) {
		/* Sync up windows. */
		tegra_dc_trigger_windows(dc);

		/* Check COMMON_ACT_REQ. */
		if (tegra_dc_handle_common_channel_promotion(dc))
			clear_bit(V_BLANK_IMP, &dc->vblank_ref_count);

		/* Schedule any additional bottom-half vblank actvities. */
		queue_work(system_freezable_wq, &dc->vblank_work);
	}

	if (status & FRAME_END_INT) {
		if (atomic_read(&dc->crc_ref_cnt.global))
			tegra_dc_crc_process(dc);

		/* Mark the frame_end as complete. */
		dc->crc_pending = false;
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);
		if (!completion_done(&dc->crc_complete))
			complete(&dc->crc_complete);

		if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
			tegra_dc_put(dc);

		queue_work(system_freezable_wq, &dc->frame_end_work);
	}

	if (status & V_PULSE2_INT) {
		if (test_bit(V_PULSE2_LATENCY_MSRMNT, &dc->vpulse2_ref_count))
			tegra_dc_collect_latency_data(dc);
		queue_work(system_freezable_wq, &dc->vpulse2_work);
	}
}

static void tegra_dc_continuous_irq(struct tegra_dc *dc, unsigned long status)
{
	/* Schedule any additional bottom-half vblank actvities. */
	if (status & V_BLANK_INT)
		queue_work(system_freezable_wq, &dc->vblank_work);

	if (status & (V_BLANK_INT | MSF_INT)) {
		if (dc->out->user_needs_vblank) {
			dc->out->user_needs_vblank = false;
			complete(&dc->out->user_vblank_comp);
		}
		tegra_dc_process_vblank(dc);
	}

	if (status & FRAME_END_INT) {
		struct timespec tm;
		ktime_get_ts(&tm);
		dc->frame_end_timestamp = timespec_to_ns(&tm);
		wake_up(&dc->timestamp_wq);

		if (!tegra_dc_windows_are_dirty(dc, WIN_ALL_ACT_REQ)) {
			if (dc->out->type == TEGRA_DC_OUT_DSI) {
				tegra_dc_vrr_get_ts(dc);
				tegra_dc_vrr_sec(dc);
			} else
				tegra_dc_vrr_extend_vfp(dc);
		}

		if (atomic_read(&dc->crc_ref_cnt.global))
			tegra_dc_crc_process(dc);

		/* Mark the frame_end as complete. */
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);
		if (!completion_done(&dc->crc_complete))
			complete(&dc->crc_complete);

		if (dc->frm_lck_info.frame_lock_enable &&
			((dc->out->type == TEGRA_DC_OUT_HDMI) ||
			(dc->out->type == TEGRA_DC_OUT_DP) ||
			(dc->out->type == TEGRA_DC_OUT_FAKE_DP))) {
			mutex_unlock(&dc->lock);
			tegra_dc_common_handle_flip_lock_error(dc);
			mutex_lock(&dc->lock);
		} else {
			tegra_dc_trigger_windows(dc);
		}

		/* Check COMMON_ACT_REQ. */
		if (tegra_dc_handle_common_channel_promotion(dc))
			_tegra_dc_config_frame_end_intr(dc, false);

		queue_work(system_freezable_wq, &dc->frame_end_work);
	}

	if (status & V_PULSE2_INT) {
		if (test_bit(V_PULSE2_LATENCY_MSRMNT, &dc->vpulse2_ref_count))
			tegra_dc_collect_latency_data(dc);
		queue_work(system_freezable_wq, &dc->vpulse2_work);
	}
}

/* XXX: Not sure if we limit look ahead to 1 frame */
bool tegra_dc_is_within_n_vsync(struct tegra_dc *dc, s64 ts)
{
	BUG_ON(!dc->frametime_ns);
	return ((ts - dc->frame_end_timestamp) < dc->frametime_ns);
}

bool tegra_dc_does_vsync_separate(struct tegra_dc *dc, s64 new_ts, s64 old_ts)
{
	BUG_ON(!dc->frametime_ns);
	return (((new_ts - old_ts) > dc->frametime_ns)
		|| (div_s64((new_ts - dc->frame_end_timestamp), dc->frametime_ns)
			!= div_s64((old_ts - dc->frame_end_timestamp),
				dc->frametime_ns)));
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
inline u64 tegra_dc_get_tsc_time(void)
{
	struct timecounter *tc;
	cycle_t value;
	u64 frac = 0;

	tc = arch_timer_get_timecounter();
	if (tc) {
		const struct cyclecounter *cc = tc->cc;

		value = cc->read(cc);
		return cyclecounter_cyc2ns(cc, value, 0, &frac);
	}
	return 0;
}
#else
inline u64 tegra_dc_get_tsc_time(void)
{
	u64 frac = 0;
	const struct cyclecounter *cc;
	struct arch_timer_kvm_info *info;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	u64 value;
#else
	cycle_t value;
#endif

	info = arch_timer_get_kvm_info();
	cc = info->timecounter.cc;

	value = cc->read(cc);
	return cyclecounter_cyc2ns(cc, value, 0, &frac);
}
#endif

static inline void tegra_dc_scanline_trace(struct tegra_dc *dc)
{
	int current_scanline, vp3_scanline;
	int current_frame, diff_frames;
	int vblank_diff_lines, vp3_diff_lines;
	u64 vblank_delta_time, vp3_delta_time;
	u64 curr_timestamp, vblank_timestamp, vp3_timestamp;

	/* TBD: Currently, if this function is delayed by greater than
	 * one frame duration, then prints for those frames would be skipped.
	 */

	/* Get current TSC time & raster position */
	curr_timestamp = tegra_dc_get_tsc_time();
	current_scanline = tegra_dc_get_v_count(dc);
	current_frame = tegra_dc_readl(dc, DC_COM_RG_DPCA) >> 16;

	/* TBD: Extend for more than one programmable scanline. */
	/* Is there a programmed scanline ? */
	vp3_scanline = tegra_dc_ext_get_scanline(dc->ext);

	/* Compare current scanline to actual event occurrence */
	if (current_scanline >= dc->mode_metadata.vblank_lines  &&
	    current_scanline <= dc->mode_metadata.vtotal_lines) {
		/* If control still in old frame; where vblank occurred */
		vblank_diff_lines = current_scanline -
				dc->mode_metadata.vblank_lines;
		if (vp3_scanline >= 0)
			vp3_diff_lines = current_scanline - vp3_scanline;
		diff_frames = 0;
	} else {
		/* If in next frame */
		vblank_diff_lines = current_scanline + dc->mode.v_front_porch;
		if (vp3_scanline >= 0)
			vp3_diff_lines = (dc->mode_metadata.vtotal_lines
					- vp3_scanline) + current_scanline;
		diff_frames = -1;
	}

	/* Calculate time delta & accurate timestamps */
	vblank_delta_time = (u64)vblank_diff_lines *
				dc->mode_metadata.line_in_nsec;
	vblank_timestamp = curr_timestamp - vblank_delta_time;
	if (vp3_scanline >= 0) {
		vp3_delta_time = (u64)vp3_diff_lines *
				dc->mode_metadata.line_in_nsec;
		vp3_timestamp = curr_timestamp - vp3_delta_time;
	}

	/* Write to ftrace buffer */
	if (vp3_scanline >= 0) {
		trace_display_scanline(dc->ctrl_num, vp3_scanline,
				current_frame+diff_frames, vp3_timestamp);
	}
	/* Write -1 for line number to indicate its vblank line # */
	trace_display_scanline(dc->ctrl_num, -1,
				current_frame+diff_frames, vblank_timestamp);
}

static irqreturn_t tegra_dc_irq(int irq, void *ptr)
{
	struct tegra_dc *dc = ptr;
	unsigned long status;
	unsigned long underflow_mask;
	u32 val;
	int need_disable = 0;

	if (tegra_dc_is_t21x() && tegra_platform_is_fpga())
		return IRQ_NONE;

	mutex_lock(&dc->lock);
	if (!tegra_dc_is_powered(dc)) {
		mutex_unlock(&dc->lock);
		return IRQ_HANDLED;
	}

	tegra_dc_get(dc);

	if (!dc->enabled || !nvhost_module_powered_ext(dc->ndev)) {
		dev_dbg(&dc->ndev->dev, "IRQ when DC not powered!\n");
		status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
		tegra_dc_writel(dc, status, DC_CMD_INT_STATUS);
		tegra_dc_put(dc);
		mutex_unlock(&dc->lock);
		return IRQ_HANDLED;
	}

	/* clear all status flags except underflow, save those for the worker */
	status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
	tegra_dc_writel(dc, status & ~ALL_UF_INT(), DC_CMD_INT_STATUS);
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val & ~ALL_UF_INT(), DC_CMD_INT_MASK);

	/*
	 * Overlays can get thier internal state corrupted during and underflow
	 * condition.  The only way to fix this state is to reset the DC.
	 * if we get 4 consecutive frames with underflows, assume we're
	 * hosed and reset.
	 */
	underflow_mask = status & ALL_UF_INT();

	/* Check underflow */
	if (underflow_mask) {
		dc->underflow_mask |= underflow_mask;
		schedule_delayed_work(&dc->underflow_work,
			msecs_to_jiffies(1));
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		tegra_dc_one_shot_irq(dc, status);
	else
		tegra_dc_continuous_irq(dc, status);

	/* Trace scanlines before mode change takes effect below */
	if (status & V_BLANK_INT) {
		/* Do any extra work only if tracepoint enabled */
		if (trace_display_scanline_enabled())
			tegra_dc_scanline_trace(dc);
	}

	/* update video mode if it has changed since the last frame */
	if (status & (FRAME_END_INT | V_BLANK_INT))
		if (tegra_dc_update_mode(dc))
			need_disable = 1; /* force display off on error */

	if (status & FRAME_END_INT) {
		dc->dbg_fe_count++;
		if (dc->disp_active_dirty) {
			tegra_dc_writel(dc, dc->mode.h_active |
				(dc->mode.v_active << 16), DC_DISP_DISP_ACTIVE);
			tegra_dc_writel(dc,
				GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

			dc->disp_active_dirty = false;
		}
	}

	if (status & V_BLANK_INT)
		trace_display_vblank(dc->ctrl_num,
			tegra_dc_readl(dc, DC_COM_RG_DPCA) >> 16);

	tegra_dc_put(dc);

#ifdef TEGRA_DC_USR_SHARED_IRQ
	/* user shared display ISR call-back */
	if (dc->isr_usr_cb)
		dc->isr_usr_cb(dc->ctrl_num, status, dc->isr_usr_pdt);
#endif /* TEGRA_DC_USR_SHARED_IRQ */

	mutex_unlock(&dc->lock);

	if (need_disable)
		tegra_dc_disable_irq_ops(dc, true);

	return IRQ_HANDLED;
}

void tegra_dc_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {
	case 3:
		color_control = BASE_COLOR_SIZE111;
		break;

	case 6:
		color_control = BASE_COLOR_SIZE222;
		break;

	case 8:
		color_control = BASE_COLOR_SIZE332;
		break;

	case 9:
		color_control = BASE_COLOR_SIZE333;
		break;

	case 12:
		color_control = BASE_COLOR_SIZE444;
		break;

	case 15:
		color_control = BASE_COLOR_SIZE555;
		break;

	case 16:
		color_control = BASE_COLOR_SIZE565;
		break;

	case 18:
		color_control = BASE_COLOR_SIZE666;
		break;

	default:
		color_control = BASE_COLOR_SIZE888;
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_UNDEFINED_DITHER:
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= DITHER_CONTROL_DISABLE;
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= DITHER_CONTROL_ORDERED;
		break;
#ifdef CONFIG_TEGRA_DC_TEMPORAL_DITHER
	case TEGRA_DC_TEMPORAL_DITHER:
		color_control |= DITHER_CONTROL_TEMPORAL;
		break;
#else
	case TEGRA_DC_ERRDIFF_DITHER:
		/* The line buffer for error-diffusion dither is limited
		 * to 1280 pixels per line. This limits the maximum
		 * horizontal active area size to 1280 pixels when error
		 * diffusion is enabled.
		 */
		BUG_ON(dc->mode.h_active > 1280);
		color_control |= DITHER_CONTROL_ERRDIFF;
		break;
#endif
	default:
		dev_err(&dc->ndev->dev, "Error: Unsupported dithering mode\n");
	}

	if (tegra_dc_is_t21x()) {
		if (dc->cmu_enabled)
			color_control |= CMU_ENABLE;
	}

	tegra_dc_writel(dc, color_control, DC_DISP_DISP_COLOR_CONTROL);
}


/*
 * Due to the limitations in DSC architecture, program DSC block with predefined
 * values.
*/
void tegra_dc_dsc_init(struct tegra_dc *dc)
{
	struct tegra_dc_mode *mode = &dc->mode;
	u32 val;
	u32 slice_width, slice_height, chunk_size, hblank;
	u32 min_rate_buf_size, num_xtra_mux_bits, hrdelay;
	u32 initial_offset, final_offset;
	u32 initial_xmit_delay, initial_dec_delay;
	u32 initial_scale_value, final_scale;
	u32 scale_dec_interval, scale_inc_interval;
	u32 groups_per_line, total_groups, first_line_bpg_offset;
	u32 nfl_bpg_offset, slice_bpg_offset;
	u32 rc_model_size = DSC_DEF_RC_MODEL_SIZE;
	u32 delay_in_slice, output_delay, wrap_output_delay;
	u8 i, j;
	u8 bpp;
	u32 check_flatness;

	/* Link compression is only supported for DSI panels */
	if ((dc->out->type != TEGRA_DC_OUT_DSI) || !dc->out->dsc_en) {
		dev_dbg(&dc->ndev->dev,
			"Link compression not supported by the panel\n");
		return;
	}

	dev_info(&dc->ndev->dev, "Configuring DSC\n");
	/*
	 * Slice height and width are in pixel. When the whole picture is one
	 * slice, slice height and width should be equal to picture height or
	 * width.
	*/
	bpp = dc->out->dsc_bpp;
	slice_height = dc->out->slice_height;
	slice_width = (mode->h_active / dc->out->num_of_slices);
	val = DSC_VALID_SLICE_HEIGHT(slice_height) |
		DSC_VALID_SLICE_WIDTH(slice_width);
	tegra_dc_writel(dc, val, DSC_COM_DSC_SLICE_INFO);

	if (tegra_dc_is_t21x()) {
		/*
		 * Use RC overflow solution 2.
		 * Program overflow threshold values and
		 * enable flatness checking.
		 */
		check_flatness = ((3 - (slice_width % 3)) != 2);
		val = DC_DISP_SPARE0_VALID_OVERFLOW_THRES(
						DC_DISP_DEF_OVERFLOW_THRES) |
			DC_DISP_SPARE0_RC_SOLUTION_MODE(
						DC_SPARE0_RC_SOLUTION_2) |
			(check_flatness << 1) | 0x1;
		tegra_dc_writel(dc, val, DC_DISP_DISPLAY_SPARE0);
	}
	/*
	 * Calculate chunk size based on slice width. Enable block prediction
	 * and set compressed bpp rate.
	 */
	chunk_size = DIV_ROUND_UP((slice_width * bpp), 8);
	val = DSC_VALID_BITS_PER_PIXEL(bpp << 4) |
		DSC_VALID_CHUNK_SIZE(chunk_size);
	if (dc->out->en_block_pred)
		val |= DSC_BLOCK_PRED_ENABLE;
	tegra_dc_writel(dc, val, DSC_COM_DSC_COMMON_CTRL);

	/* Set output delay */
	initial_xmit_delay = (4096 / bpp);
	if (slice_height == mode->v_active)
		initial_xmit_delay = 475;
	delay_in_slice = DIV_ROUND_UP(DSC_ENC_FIFO_SIZE * 8 * 3, bpp) +
		slice_width + initial_xmit_delay + DSC_START_PIXEL_POS;
	hblank = mode->h_sync_width + mode->h_front_porch + mode->h_back_porch;
	output_delay = ((delay_in_slice / slice_width) *
		(mode->h_active + hblank)) + (delay_in_slice % slice_width);
	if (dc->out->dual_dsc_en) {
		wrap_output_delay = output_delay * 2 + mode->h_active / 2 + 28;
		if (wrap_output_delay % 2)
			wrap_output_delay += 1;
	} else {
		wrap_output_delay = output_delay + 20;
	}
	val = DSC_VALID_OUTPUT_DELAY(output_delay);
	val |= DSC_VALID_WRAP_OUTPUT_DELAY(wrap_output_delay);
	tegra_dc_writel(dc, val, DC_COM_DSC_DELAY);

	/* Set RC flatness info and bpg offset for first line of slice */
	first_line_bpg_offset = (bpp == 8) ? DSC_DEF_8BPP_FIRST_LINE_BPG_OFFS :
		DSC_DEF_12BPP_FIRST_LINE_BPG_OFFS;
	val = DSC_VALID_FLATNESS_MAX_QP(12) | DSC_VALID_FLATNESS_MIN_QP(3) |
		DSC_VALID_FIRST_LINE_BPG_OFFS(first_line_bpg_offset);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_FLATNESS_INFO);


	/* Set RC model offset values to be used at slice start and end */
	initial_offset = (bpp == 8) ? DSC_DEF_8BPP_INITIAL_OFFSET :
		DSC_DEF_12BPP_INITIAL_OFFSET;
	num_xtra_mux_bits = 198 + ((chunk_size * slice_height * 8 - 246) % 48);
	final_offset = rc_model_size - (initial_xmit_delay * bpp) +
		num_xtra_mux_bits;
	val = DSC_VALID_INITIAL_OFFSET(initial_offset) |
		DSC_VALID_FINAL_OFFSET(final_offset);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_OFFSET_INFO);

	/*
	 * DSC_SLICE_BPG_OFFSET:Bpg offset used to enforce slice bit constraint
	 * DSC_NFL_BPG_OFFSET:Non-first line bpg offset to use
	 */
	nfl_bpg_offset = DIV_ROUND_UP((first_line_bpg_offset << 11),
		(slice_height - 1));
	slice_bpg_offset = (rc_model_size - initial_offset +
		num_xtra_mux_bits) * (1 << 11);
	groups_per_line = slice_width / 3;
	total_groups = slice_height * groups_per_line;
	slice_bpg_offset = DIV_ROUND_UP(slice_bpg_offset, total_groups);
	val = DSC_VALID_SLICE_BPG_OFFSET(slice_bpg_offset) |
		DSC_VALID_NFL_BPG_OFFSET(nfl_bpg_offset);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_BPGOFF_INFO);

	/*
	 * INITIAL_DEC_DELAY:Num of pixels to delay the VLD on the decoder
	 * INITIAL_XMIT_DELAY:Num of pixels to delay the initial transmission
	 */
	min_rate_buf_size = rc_model_size - initial_offset +
		(initial_xmit_delay * bpp) +
		(groups_per_line * first_line_bpg_offset);
	hrdelay = DIV_ROUND_UP(min_rate_buf_size, bpp);
	initial_dec_delay = hrdelay - initial_xmit_delay;
	val = DSC_VALID_INITIAL_XMIT_DELAY(initial_xmit_delay) |
		DSC_VALID_INITIAL_DEC_DELAY(initial_dec_delay);
	tegra_dc_writel(dc, val, DSC_COM_DSC_RC_RELAY_INFO);

	/*
	 * SCALE_DECR_INTERVAL:Decrement scale factor every scale_decr_interval
	 * groups.
	 * INITIAL_SCALE_VALUE:Initial value for scale factor
	 * SCALE_INCR_INTERVAL:Increment scale factor every scale_incr_interval
	 * groups.
	 */
	initial_scale_value = (8 * rc_model_size) / (rc_model_size -
		initial_offset);
	scale_dec_interval = groups_per_line / (initial_scale_value - 8);
	val = DSC_VALID_SCALE_DECR_INTERVAL(scale_dec_interval) |
		DSC_VALID_INITIAL_SCALE_VALUE(initial_scale_value);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_SCALE_INFO);

	final_scale = (8 * rc_model_size) / (rc_model_size - final_offset);
	scale_inc_interval = (2048 * final_offset) /
		((final_scale - 9) * (slice_bpg_offset + nfl_bpg_offset));
	val = DSC_VALID_SCALE_INCR_INTERVAL(scale_inc_interval);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_SCALE_INFO_2);

	/* Set the RC parameters */
	val = DSC_VALID_RC_TGT_OFFSET_LO(3) | DSC_VALID_RC_TGT_OFFSET_HI(3) |
		DSC_VALID_RC_EDGE_FACTOR(6) |
		DSC_VALID_RC_QUANT_INCR_LIMIT1(11) |
		DSC_VALID_RC_QUANT_INCR_LIMIT0(11);
	tegra_dc_writel(dc, val, DC_COM_DSC_RC_PARAM_SET);

	for (i = 0, j = 0; j < DSC_MAX_RC_BUF_THRESH_REGS; j++) {
		val = DSC_VALID_RC_BUF_THRESH_0(dsc_rc_buf_thresh[i++]);
		val |= DSC_VALID_RC_BUF_THRESH_1(dsc_rc_buf_thresh[i++]);
		val |= DSC_VALID_RC_BUF_THRESH_2(dsc_rc_buf_thresh[i++]);
		val |= DSC_VALID_RC_BUF_THRESH_3(dsc_rc_buf_thresh[i++]);

		if (dsc_rc_buf_thresh_regs[j] == DC_COM_DSC_RC_BUF_THRESH_0)
			val |= DSC_VALID_RC_MODEL_SIZE(rc_model_size);
		tegra_dc_writel(dc, val, dsc_rc_buf_thresh_regs[j]);
	}

	for (i = 0, j = 0; j < DSC_MAX_RC_RANGE_CFG_REGS; j++) {
		val = DSC_VALID_RC_RANGE_PARAM_LO(
			SET_RC_RANGE_MIN_QP(dsc_rc_ranges_8bpp_8bpc[i][0]) |
			SET_RC_RANGE_MAX_QP(dsc_rc_ranges_8bpp_8bpc[i][1]) |
			SET_RC_RANGE_BPG_OFFSET(dsc_rc_ranges_8bpp_8bpc[i][2]));
		i++;
		val |= DSC_VALID_RC_RANGE_PARAM_HI(
			SET_RC_RANGE_MIN_QP(dsc_rc_ranges_8bpp_8bpc[i][0]) |
			SET_RC_RANGE_MAX_QP(dsc_rc_ranges_8bpp_8bpc[i][1]) |
			SET_RC_RANGE_BPG_OFFSET(dsc_rc_ranges_8bpp_8bpc[i][2]));
		i++;
		tegra_dc_writel(dc, val, dsc_rc_range_config[j]);
	}

	val = tegra_dc_readl(dc, DC_COM_DSC_UNIT_SET);
	val &= ~DSC_LINEBUF_DEPTH_8_BIT;
	/* If dual dsc is enabled then the number slices are distributed
	 * between each link i.e 2 links with 4 lanes each.
	 */
	val |= DSC_VALID_SLICE_NUM_MINUS1_IN_LINE((dc->out->dual_dsc_en ?
			dc->out->num_of_slices / 2 : dc->out->num_of_slices)
			- 1);
	val |= DSC_CHECK_FLATNESS2;
	val |= DSC_FLATNESS_FIX_EN;
	tegra_dc_writel(dc, val, DC_COM_DSC_UNIT_SET);

	dev_info(&dc->ndev->dev, "DSC configured\n");
}

void tegra_dc_en_dis_dsc(struct tegra_dc *dc, bool enable)
{
	u32 val, set_bits = 0x0;
	bool is_enabled = false, set_reg = false;

	if ((dc->out->type != TEGRA_DC_OUT_DSI) || !dc->out->dsc_en)
		return;

	val = tegra_dc_readl(dc, DC_COM_DSC_TOP_CTL);

	if (dc->out->dual_dsc_en) {
		set_bits = DSC_ENABLE | DSC_DUAL_ENABLE;
		if ((val & DSC_ENABLE) && (val & DSC_DUAL_ENABLE))
			is_enabled = true;
	} else {
		set_bits = DSC_ENABLE;
		if (val & DSC_ENABLE)
			is_enabled = true;
	}
	if (enable && !is_enabled) {
		val |= set_bits;
		set_reg = true;
	} else if (!enable && is_enabled) {
		val &= ~set_bits;
		set_reg = true;
	}

	if (set_reg) {
		dev_info(&dc->ndev->dev, "Link compression %s\n",
			enable ? "enabled" : "disabled");
		val &= ~DSC_AUTO_RESET;
		tegra_dc_writel(dc, val, DC_COM_DSC_TOP_CTL);
	}
}

/* Used only on T21x */
static void tegra_dc_init_vpulse2_int(struct tegra_dc *dc)
{
	u32 start, end;
	unsigned long val;

	val = V_PULSE2_H_POSITION(0) | V_PULSE2_LAST(0x1);
	tegra_dc_writel(dc, val, DC_DISP_V_PULSE2_CONTROL);

	start = dc->mode.v_ref_to_sync + dc->mode.v_sync_width +
		dc->mode.v_back_porch +	dc->mode.v_active;
	end = start + 1;
	val = V_PULSE2_START_A(start) + V_PULSE2_END_A(end);
	tegra_dc_writel(dc, val, DC_DISP_V_PULSE2_POSITION_A);

	val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
	val |= V_PULSE2_INT;
	tegra_dc_writel(dc, val , DC_CMD_INT_ENABLE);

	tegra_dc_mask_interrupt(dc, V_PULSE2_INT);
	val = tegra_dc_readl(dc, DC_DISP_DISP_SIGNAL_OPTIONS0);
	val |= V_PULSE_2_ENABLE;
	tegra_dc_writel(dc, val, DC_DISP_DISP_SIGNAL_OPTIONS0);
}

/* Used only on T21x */
static int tegra_dc_init(struct tegra_dc *dc)
{
	int i;
	int int_enable;
	u32 val;

	tegra_dc_io_start(dc);
	tegra_dc_writel(dc, 0x00000100, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	tegra_dc_writel(dc, 0x00000100 | dc->vblank_syncpt,
			DC_CMD_CONT_SYNCPT_VSYNC);

	tegra_dc_writel(dc, 0x00004700, DC_CMD_INT_TYPE);
	tegra_dc_writel(dc, WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT |
		WIN_T_UF_INT | WIN_D_UF_INT | HC_UF_INT |
		WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT,
		DC_CMD_INT_POLARITY);
	tegra_dc_writel(dc, 0x00202020, DC_DISP_MEM_HIGH_PRIORITY);
	tegra_dc_writel(dc, 0x00010101, DC_DISP_MEM_HIGH_PRIORITY_TIMER);
	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = (FRAME_END_INT | V_BLANK_INT | ALL_UF_INT());
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;

	tegra_dc_writel(dc, int_enable, DC_CMD_INT_ENABLE);
	tegra_dc_writel(dc, ALL_UF_INT(), DC_CMD_INT_MASK);
	tegra_dc_init_vpulse2_int(dc);

	tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
		DC_CMD_STATE_ACCESS);

	tegra_dc_writel(dc, 0x00000000, DC_DISP_BLEND_BACKGROUND_COLOR);

	if (tegra_dc_is_t21x()) {
		if (dc->is_cmu_set_bl)
			_tegra_dc_update_cmu_aligned(dc, &dc->cmu, true);
		else
			_tegra_dc_update_cmu(dc, &dc->cmu);
		dc->is_cmu_set_bl = false;
	}

	tegra_dc_set_color_control(dc);
	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		if (tegra_dc_is_t21x()) {
			tegra_dc_set_win_csc(dc, &win->win_csc);
			tegra_dc_set_lut(dc, win);
		}
		if (tegra_dc_is_nvdisplay()) {
			tegra_dc_set_nvdisp_win_csc(dc, &win->nvdisp_win_csc);
			tegra_dc_set_nvdisp_lut(dc, win);
		}
		tegra_dc_set_scaling_filter(dc);
	}

	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		BUG_ON(!win);

		/* refuse to operate on invalid syncpts */
		if (WARN_ON(win->syncpt.id == NVSYNCPT_INVALID))
			continue;

		if (!nvhost_syncpt_read_ext_check(dc->ndev, win->syncpt.id, &val))
			win->syncpt.min = win->syncpt.max = val;
	}

	dc->crc_pending = false;

	trace_display_mode(dc, &dc->mode);

	if (dc->mode.pclk) {
		if (!dc->initialized) {
			if (tegra_dc_program_mode(dc, &dc->mode)) {
				tegra_dc_io_end(dc);
				dev_warn(&dc->ndev->dev,
					"%s: tegra_dc_program_mode failed\n",
					__func__);
				return -EINVAL;
			}
		} else {
			dev_info(&dc->ndev->dev, "DC initialized, "
					"skipping tegra_dc_program_mode.\n");
		}
	}

	tegra_dc_io_end(dc);

	return 0;
}

/* Used only on T21x */
static bool _tegra_dc_controller_enable(struct tegra_dc *dc)
{
	int failed_init = 0;
	int i;

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	tegra_dc_unpowergate_locked(dc);

	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	tegra_dc_setup_clk(dc, dc->clk);

	/* dc clk always on for continuous mode */
	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
		tegra_dc_clk_enable(dc);
	else
#ifdef CONFIG_TEGRA_CORE_DVFS
		tegra_dvfs_set_rate(dc->clk, dc->mode.pclk);
#else
		;
#endif

	tegra_dc_get(dc);

	tegra_dc_power_on(dc);

	/* do not accept interrupts during initialization */
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	enable_dc_irq(dc);

	failed_init = tegra_dc_init(dc);
	if (failed_init) {
		tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
		disable_irq_nosync(dc->irq);
		tegra_dc_clear_bandwidth(dc);
		if (dc->out && dc->out->disable)
			dc->out->disable(&dc->ndev->dev);
		tegra_dc_put(dc);
		if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
			tegra_dc_clk_disable(dc);
		else
#ifdef CONFIG_TEGRA_CORE_DVFS
			tegra_dvfs_set_rate(dc->clk, 0);
#else
			;
#endif
		dev_warn(&dc->ndev->dev,
			"%s: tegra_dc_init failed\n", __func__);
		return false;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++)
		dc->blend.z[i] = -1;

	tegra_dc_ext_enable(dc->ext);

	/* initialize cursor to defaults, as driver depends on HW state */
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_NS);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_HI);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_HI_NS);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_POSITION);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_POSITION_NS);
	tegra_dc_writel(dc, 0xffffff, DC_DISP_CURSOR_FOREGROUND); /* white */
	tegra_dc_writel(dc, 0x000000, DC_DISP_CURSOR_BACKGROUND); /* black */
	tegra_dc_writel(dc, 0, DC_DISP_BLEND_CURSOR_CONTROL);

	trace_display_enable(dc);

	tegra_dc_writel(dc, CURSOR_UPDATE, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, CURSOR_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_dsc_init(dc);

	if (dc->out->postpoweron)
		dc->out->postpoweron(&dc->ndev->dev);

	if (dc->out_ops && dc->out_ops->postpoweron)
		dc->out_ops->postpoweron(dc);

	tegra_log_resume_time();

	tegra_dc_put(dc);

	return true;
}

static int _tegra_dc_set_default_videomode(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0) {
		switch (dc->out->type) {
		case TEGRA_DC_OUT_HDMI:
			/* No fallback mode. If no mode info available
			 * from bootloader or device tree,
			 * mode will be set by userspace during unblank.
			 */
			break;
		case TEGRA_DC_OUT_DP:
			if (tegra_dc_is_nvdisplay())
				break;
		case TEGRA_DC_OUT_FAKE_DP:
		case TEGRA_DC_OUT_NULL:
			return tegra_dc_set_fb_mode(dc, &tegra_dc_vga_mode, 0);

		/* Do nothing for other outputs for now */
		case TEGRA_DC_OUT_RGB:

		case TEGRA_DC_OUT_DSI:

		default:
			return false;
		}
	}

	return false;
}

int tegra_dc_set_default_videomode(struct tegra_dc *dc)
{
	return _tegra_dc_set_default_videomode(dc);
}

/* Preset sync point maxval for maximum range. */
static u32 tegra_dc_syncpt_preset_maxval(struct platform_device *dev, u32 id)
{
	u32 old;

	old = nvhost_syncpt_read_minval(dev, id);
	nvhost_syncpt_set_maxval(dev, id, old - 2);
	return old;
}

/* Advance sync point to flush all waiters, return value before advancing. */
static u32 tegra_dc_syncpt_flush(struct platform_device *dev, u32 id)
{
	u32 old;

	old = nvhost_syncpt_read_minval(dev, id);
	nvhost_syncpt_set_min_eq_max_ext(dev, id);
	return old;
}

static bool _tegra_dc_enable(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0)
		return false;

	if (!dc->out)
		return false;

	if (dc->enabled)
		return true;

	dc->shutdown = false;

	if ((dc->out->type == TEGRA_DC_OUT_HDMI ||
		dc->out->type == TEGRA_DC_OUT_DP ||
		dc->out->type == TEGRA_DC_OUT_DSI) &&
		!tegra_dc_hpd(dc))
		return false;

	pm_runtime_get_sync(&dc->ndev->dev);

	if (tegra_dc_is_nvdisplay()) {
		if (tegra_nvdisp_head_enable(dc)) {
			pm_runtime_put_sync(&dc->ndev->dev);
			return false;
		}
	} else {
		if (!_tegra_dc_controller_enable(dc)) {
			pm_runtime_put_sync(&dc->ndev->dev);
			return false;
		}
	}

	if (tegra_dc_is_nvdisplay())
		tegra_dc_crc_reset(dc);

	tegra_dc_client_handle_event(dc, NOTIFY_DC_ENABLED_EVENT);

	return true;
}

void tegra_dc_enable(struct tegra_dc *dc)
{
	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		if (tegra_dc_reserve_common_channel(dc)) {
			dev_err(&dc->ndev->dev,
				"%s: DC %d enable failed due to timeout\n",
				__func__, dc->ctrl_num);
			mutex_unlock(&dc->lock);

			return;
		}
		tegra_dc_syncpt_preset_maxval(dc->ndev, dc->vblank_syncpt);
		dc->enabled = _tegra_dc_enable(dc);
		tegra_dc_release_common_channel(dc);
	}

	mutex_unlock(&dc->lock);
	trace_display_mode(dc, &dc->mode);
}

static void tegra_dc_flush_syncpts_window(struct tegra_dc *dc, unsigned win)
{
	struct tegra_dc_win *w = tegra_dc_get_window(dc, win);
	u32 max;

	/* refuse to operate on invalid syncpts */
	if (WARN_ON(w->syncpt.id == NVSYNCPT_INVALID))
		return;

	/* flush any pending syncpt waits */
	max = tegra_dc_incr_syncpt_max_locked(dc, win);
	while (w->syncpt.min < w->syncpt.max) {
		trace_display_syncpt_flush(dc, w->syncpt.id,
			w->syncpt.min, w->syncpt.max);
		w->syncpt.min++;
		nvhost_syncpt_cpu_incr_ext(dc->ndev, w->syncpt.id);
	}
}

void tegra_dc_disable_window(struct tegra_dc *dc, unsigned win)
{
	struct tegra_dc_win *w = tegra_dc_get_window(dc, win);

	/* reset window bandwidth */
	w->bandwidth = 0;
	w->new_bandwidth = 0;

	/* disable windows */
	w->flags &= ~TEGRA_WIN_FLAG_ENABLED;

	/* flush pending syncpts */
	tegra_dc_flush_syncpts_window(dc, win);
}

static void _tegra_dc_controller_disable(struct tegra_dc *dc)
{
	unsigned i;

	tegra_dc_get(dc);

	if (atomic_read(&dc->holding)) {
		/* Force release all refs but the last one */
		atomic_set(&dc->holding, 1);
		tegra_dc_release_dc_out(dc);
	}

	if (dc->out && dc->out->prepoweroff)
		dc->out->prepoweroff();

	if (dc->out_ops && dc->out_ops->vrr_enable &&
		dc->out && dc->out->vrr && dc->out->vrr->capability) {
		dc->out_ops->vrr_enable(dc, 0);
		/* TODO: Fix properly. Bug 1644102. */
		tegra_dc_set_act_vfp(dc, dc->mode.v_front_porch);
	}

	if (dc->out_ops && dc->out_ops->disable)
		dc->out_ops->disable(dc);

	if (tegra_dc_is_powered(dc))
		tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	disable_irq_nosync(dc->irq);

	tegra_dc_clear_bandwidth(dc);

	if (dc->out && dc->out->disable)
		dc->out->disable(&dc->ndev->dev);

	for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
		tegra_dc_disable_window(dc, i);
	}
	trace_display_disable(dc);

	if (dc->out_ops && dc->out_ops->postpoweroff)
		dc->out_ops->postpoweroff(dc);

	if (tegra_dc_is_nvdisplay())
		/* clear the windows ownership from head*/
		tegra_nvdisp_head_disable(dc);

	/* clean up tegra_dc_vsync_enable() */
	while (dc->out->user_needs_vblank > 0)
		_tegra_dc_user_vsync_enable(dc, false);

	if (test_bit(V_BLANK_USER, &dc->vblank_ref_count)) {
		tegra_dc_release_dc_out(dc);
		clear_bit(V_BLANK_USER, &dc->vblank_ref_count);
	}

	tegra_dc_put(dc);

	if (tegra_dc_is_t21x()) {
		/* disable always on dc clk in continuous mode */
		if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
			tegra_dc_clk_disable(dc);
		else
#ifdef CONFIG_TEGRA_CORE_DVFS
			tegra_dvfs_set_rate(dc->clk, 0);
#else
		;
#endif
	}

}

void tegra_dc_stats_enable(struct tegra_dc *dc, bool enable)
{
	return;
}

bool tegra_dc_stats_get(struct tegra_dc *dc)
{
	return true;
}

/* blank selected windows by disabling them */
void tegra_dc_blank_wins(struct tegra_dc *dc, unsigned windows)
{
	int nwins = tegra_dc_get_numof_dispwindows();
	struct tegra_dc_win *dcwins[nwins];
	struct tegra_dc_win blank_win;
	unsigned i;
	unsigned long int blank_windows;
	int nr_win = 0;

	/* YUV420 10bpc variables */
	bool yuv_420_10b_path = false;
	int fb_win_idx = -1;
	int fb_win_pos = -1;

	if (dc->yuv_bypass && tegra_dc_is_yuv420_10bpc(&dc->mode))
		yuv_420_10b_path = true;

	if (tegra_dc_is_nvdisplay() && dc->shutdown)
		yuv_420_10b_path = false;

	if (yuv_420_10b_path) {
		u32 active_width = dc->mode.h_active;
		u32 active_height = dc->mode.v_active;

		blank_win = *tegra_fb_get_blank_win(dc->fb);

		/*
		 * 420 10bpc blank frame statically
		 * created for this pixel format
		 */
		blank_win.h.full = dfixed_const(1);
		blank_win.w.full = dfixed_const(active_width);
		blank_win.fmt = TEGRA_DC_EXT_FMT_T_A8R8G8B8;
		blank_win.out_w = active_width;
		blank_win.out_h = active_height;

		dcwins[0] = &blank_win;
		fb_win_idx = dcwins[0]->idx;
		nr_win++;
	}

	blank_windows = windows & dc->valid_windows;

	if (!blank_windows)
		return;

	for_each_set_bit(i, &blank_windows, tegra_dc_get_numof_dispwindows()) {
		dcwins[nr_win] = tegra_dc_get_window(dc, i);
		if (!dcwins[nr_win])
			continue;
		/*
		 * Prevent disabling the YUV410 10bpc window in case
		 * it is also in blank_windows, additionally, prevent
		 * adding it to the list twice.
		 */
		if (fb_win_idx == dcwins[nr_win]->idx) {
			fb_win_pos = i;
			continue;
		}
		dcwins[nr_win++]->flags &= ~TEGRA_WIN_FLAG_ENABLED;
	}

	tegra_dc_update_windows(dcwins, nr_win, NULL, true, false);
	tegra_dc_sync_windows(dcwins, nr_win);

	tegra_dc_program_bandwidth(dc, true);

	/*
	 * Disable, reset bandwidth and advance pending syncpoints
	 * of all windows. In case the statically created 420 10bpc
	 * is also present in blank_windows, only advance syncpoints.
	 */
	for_each_set_bit(i, &blank_windows, tegra_dc_get_numof_dispwindows()) {
		if (fb_win_pos == i) {
			tegra_dc_flush_syncpts_window(dc, i);
			continue;
		}
		tegra_dc_disable_window(dc, i);
	}
}

int tegra_dc_restore(struct tegra_dc *dc)
{
	return tegra_dc_ext_restore(dc->ext);
}

static void _tegra_dc_disable(struct tegra_dc *dc)
{
	/* power down resets the registers, setting to true
	 * causes CMU to be restored in tegra_dc_init(). */
	if (tegra_dc_is_t21x())
		dc->cmu_dirty = true;

	tegra_dc_crc_deinit(dc);

	tegra_dc_get(dc);
	_tegra_dc_controller_disable(dc);
	tegra_dc_put(dc);

	tegra_dc_powergate_locked(dc);

	pm_runtime_put(&dc->ndev->dev);

	tegra_log_suspend_entry_time();

	tegra_dc_client_handle_event(dc, NOTIFY_DC_DISABLED_EVENT);
}

void tegra_dc_disable(struct tegra_dc *dc)
{
	dc->shutdown = true;
	tegra_dc_disable_irq_ops(dc, false);
	tegra_dc_syncpt_flush(dc->ndev, dc->vblank_syncpt);
}

static inline void tegra_dc_disable_all_wins(struct tegra_dc *dc)
{
	int blank_windows = tegra_dc_ext_disable(dc->ext);

	/*
	 * The tegra_dc_ext_disable() call above will disable the windows on
	 * this head that are owned by the given dc_ext owner. Any active
	 * windows on this head that have no dc_ext owner will be left
	 * untouched. The following tegra_dc_blank_wins() call will ensure that
	 * any remaining windows are actually disabled.
	 */
	tegra_dc_blank_wins(dc, ~blank_windows);
}

static void tegra_dc_disable_irq_ops(struct tegra_dc *dc, bool from_irq)
{
	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

	if (dc->shutdown) {
		if ((dc->out->type == TEGRA_DC_OUT_HDMI) ||
		    (dc->out->type == TEGRA_DC_OUT_DP))
			if (dc->out_ops && dc->out_ops->shutdown_interface)
				dc->out_ops->shutdown_interface(dc);
	}

	if (tegra_dc_reserve_common_channel(dc)) {
		dev_err(&dc->ndev->dev,
			"%s: DC %d disable failed due to timeout\n",
			__func__, dc->ctrl_num);

		return;
	}

	tegra_dc_disable_all_wins(dc);

	if (dc->cursor.enabled)
		tegra_dc_cursor_suspend(dc);

	/* it's important that new underflow work isn't scheduled before the
	 * lock is acquired. */
	cancel_delayed_work_sync(&dc->underflow_work);


	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}

	mutex_lock(&dc->lp_lock);
	mutex_lock(&dc->lock);

	if (dc->enabled) {
		dc->enabled = false;
		dc->blanked = false;

		if (!dc->suspended)
			_tegra_dc_disable(dc);
	}

	tegra_dc_release_common_channel(dc);

#ifdef CONFIG_SWITCH
	if (dc->switchdev_registered)
		switch_set_state(&dc->modeset_switch, 0);
#endif
	mutex_unlock(&dc->lock);
	mutex_unlock(&dc->lp_lock);
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);
	if (!from_irq)
		synchronize_irq(dc->irq);
	trace_display_mode(dc, &dc->mode);

	/* disable pending clks due to uncompleted frames */
	while (tegra_platform_is_silicon() && atomic_read(&dc->enable_count))
		tegra_dc_put(dc);
}

static void tegra_dc_underflow_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, underflow_work);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	if (dc->enabled)
		tegra_dc_underflow_handler(dc);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

static void (*flip_callback)(void);
static spinlock_t flip_callback_lock;
static bool init_tegra_dc_flip_callback_called;

static int __init init_tegra_dc_flip_callback(void)
{
	spin_lock_init(&flip_callback_lock);
	init_tegra_dc_flip_callback_called = true;
	return 0;
}

pure_initcall(init_tegra_dc_flip_callback);

int tegra_dc_set_flip_callback(void (*callback)(void))
{
	WARN_ON(!init_tegra_dc_flip_callback_called);

	spin_lock(&flip_callback_lock);
	flip_callback = callback;
	spin_unlock(&flip_callback_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_set_flip_callback);

int tegra_dc_unset_flip_callback(void)
{
	spin_lock(&flip_callback_lock);
	flip_callback = NULL;
	spin_unlock(&flip_callback_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_unset_flip_callback);

void tegra_dc_call_flip_callback(void)
{
	spin_lock(&flip_callback_lock);
	if (flip_callback)
		flip_callback();
	spin_unlock(&flip_callback_lock);
}
EXPORT_SYMBOL(tegra_dc_call_flip_callback);

#ifdef CONFIG_SWITCH
static ssize_t switch_modeset_print_mode(struct switch_dev *sdev, char *buf)
{
	struct tegra_dc *dc =
		container_of(sdev, struct tegra_dc, modeset_switch);

	if (!sdev->state)
		return sprintf(buf, "offline\n");

	return sprintf(buf, "%dx%d\n", dc->mode.h_active, dc->mode.v_active);
}
#endif

/* enables pads and clocks to perform DDC/I2C */
int tegra_dc_ddc_enable(struct tegra_dc *dc, bool enabled)
{
	int ret = -ENOSYS;
	if (dc->out_ops) {
		if (enabled && dc->out_ops->ddc_enable)
			ret = dc->out_ops->ddc_enable(dc);
		else if (!enabled && dc->out_ops->ddc_disable)
			ret = dc->out_ops->ddc_disable(dc);
	}
	return ret;
}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
int tegra_dc_slgc_disp0(struct notifier_block *nb,
	unsigned long unused0, void *unused1)
{

	struct tegra_dc *dc = NULL;
	u32 val;

	if (tegra_dc_is_nvdisplay())
		return NOTIFY_OK;

	dc = container_of(nb, struct tegra_dc, slgc_notifier);
	tegra_dc_get(dc);

	val = tegra_dc_readl(dc, DC_COM_DSC_TOP_CTL);
	val |= DSC_SLCG_OVERRIDE;
	tegra_dc_writel(dc, val, DC_COM_DSC_TOP_CTL); /* set */
	/* flush the previous write */
	(void)tegra_dc_readl(dc, DC_CMD_DISPLAY_COMMAND);
	val &= ~DSC_SLCG_OVERRIDE;
	tegra_dc_writel(dc, val, DC_COM_DSC_TOP_CTL); /* restore */

	tegra_dc_put(dc);

	return NOTIFY_OK;
}
#endif

int tegra_dc_update_winmask(struct tegra_dc *dc, unsigned long winmask)
{
	struct tegra_dc *dc_other;
	struct tegra_dc_win *win;
	int i, j, ret = 0;
	int win_idx = -1;

	if (tegra_dc_is_t21x())
		return -EINVAL;

	/* check that dc is not NULL and do range check */
	if (!dc || (winmask >= (1 << tegra_dc_get_numof_dispwindows())))
		return -EINVAL;

	mutex_lock(&dc->lock);
	if ((!dc->ndev) || (dc->enabled)) {
		ret = -EINVAL;
		goto exit;
	}

	/* check requested=enabled windows NOT owned by other dcs */
	for_each_set_bit(i, &winmask, tegra_dc_get_numof_dispwindows()) {
		j = dc->ndev->id;
		win = tegra_dc_get_window(dc, i);
		/* is window already owned by this dc? */
		if (win && win->dc && (win->dc == dc)) {
			/* get first valid window index for fb win index */
			if (win_idx == -1)
				win_idx = i;
			continue;
		}
		/* is window already owned by other dc? */
		for (j = 0; j < tegra_dc_get_numof_dispheads(); j++) {
			dc_other = tegra_dc_get_dc(j);
			if (!dc_other)
				continue;
			if (!dc_other->pdata) {
				ret = -EINVAL;
				goto exit;
			}
			/* found valid dc, does it own window=i? */
			if ((dc_other->pdata->win_mask >> i) & 0x1) {
				dev_err(&dc->ndev->dev,
					"win[%d] already on fb%d\n", i, j);
				ret = -EINVAL;
				goto exit;
			}
		}

		/* get first valid window index for fb win index */
		if (win_idx == -1)
			win_idx = i;
	}

	/* attach window happens on device enable call and
	 * detach window happens on device disable call
	 */

	dc->pdata->win_mask = winmask;
	dc->valid_windows = winmask;

	tegra_fb_set_win_index(dc, winmask);
	dc->pdata->fb->win = win_idx;

exit:
	mutex_unlock(&dc->lock);
	return ret;
}

struct clk *tegra_disp_of_clk_get_by_name(struct device_node *np,
						const char *name)
{
	return of_clk_get_by_name(np, name);
}

struct clk *tegra_disp_clk_get(struct device *dev, const char *id)
{
	struct clk *disp_clk;

	if (tegra_dc_is_t21x())
		return devm_clk_get(dev, id);

	disp_clk = devm_clk_get(dev, id);
	if (IS_ERR_OR_NULL(disp_clk))
		pr_err("Failed to get %s clk\n", id);
	return disp_clk;

}

void tegra_disp_clk_put(struct device *dev, struct clk *clk)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			devm_clk_put(dev, clk);
	} else {
		return clk_put(clk);
	}
}

bool tegra_is_bl_display_initialized(int instance)
{
	struct tegra_dc *dc = find_dc_by_ctrl_num(instance);

	if (!dc) {
		pr_err("could not find dc with ctrl number %d\n", instance);
		return false;
	}
	if (!dc->fb_mem) {
		pr_debug("dc->fb_mem not initialized\n");
		return false;
	}
	return (dc->fb_mem->start != 0);
}
EXPORT_SYMBOL(tegra_is_bl_display_initialized);

#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
void tegra_get_fb_resource(struct resource *fb_res, int instance)
{
}
#endif

static int tegra_dc_probe(struct platform_device *ndev)
{
	struct tegra_dc *dc;
	struct tegra_dc_mode *mode;
	struct tegra_dc_platform_data *dt_pdata = NULL;
	struct clk *clk;
#ifndef CONFIG_TEGRA_ISOMGR
	struct clk *emc_clk;
#else
	int isomgr_client_id = -1;
#endif
	struct tegra_bwmgr_client *emc_la_handle;
	struct device_node *np = ndev->dev.of_node;
	struct resource *fb_mem = NULL;
	char clk_name[16];
	int ret = 0;
	void __iomem *base;
	int irq;
	int i;
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	int partition_id_disa, partition_id_disb;
#endif
	struct resource of_fb_res;
	struct resource of_lut_res;
	int hotplug_init_status = -1;

	if (tegra_dc_is_nvdisplay() && !tegra_dc_common_probe_status())
		return -EPROBE_DEFER;

	/* Serialize dc device registration */
	mutex_lock(&tegra_dc_registration_lock);

	/* Specify parameters for the maximum physical segment size. */
	ndev->dev.dma_parms = &tegra_dc_dma_parameters;

	dc = kzalloc(sizeof(struct tegra_dc), GFP_KERNEL);
	if (!dc) {
		dev_err(&ndev->dev, "can't allocate memory for tegra_dc\n");
		mutex_unlock(&tegra_dc_registration_lock);
		return -ENOMEM;
	}

	irq = of_irq_to_resource(np, 0, NULL);
	if (!irq)
		goto err_free;

	ndev->id = tegra_dc_set(dc);
	if (ndev->id < 0) {
		dev_err(&ndev->dev, "can't add dc\n");
		goto err_free;
	}

	dt_pdata = of_dc_parse_platform_data(ndev, dt_pdata);
	if (IS_ERR_OR_NULL(dt_pdata)) {
		ret = dt_pdata ? PTR_ERR(dt_pdata) : -EINVAL;
		goto err_free;
	}

	dc->ctrl_num = dt_pdata->ctrl_num;

	base = of_iomap(np, 0);
	if (!base) {
		dev_err(&ndev->dev, "registers can't be mapped\n");
		ret = -EBUSY;
		goto err_release_resource_reg;
	}

	dev_info(&ndev->dev, "Display dc.%p registered with id=%d\n",
			base, ndev->id);

	if (tegra_dc_is_t21x()) {
		int i;
		char syncpt_name[25];
		const char win_name[] = "abcd";

		for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++)
			dc->windows[i].syncpt.id = NVSYNCPT_INVALID;

		dc->valid_windows = dt_pdata->win_mask;

		for_each_set_bit(i, &dc->valid_windows,
			tegra_dc_get_numof_dispwindows()) {
			/* Get syncpt_name like disp0_a */
			snprintf(syncpt_name, sizeof(syncpt_name),
				"disp%d_%c", dc->ctrl_num, win_name[i]);
			dc->windows[i].syncpt.id =
				nvhost_get_syncpt_client_managed(ndev,
								syncpt_name);
			/* Use first valid window as fb window */
			if (dt_pdata->fb->win == TEGRA_FB_WIN_INVALID)
				dt_pdata->fb->win = i;
		}

		if (dc->ctrl_num == 0) {
			dc->vblank_syncpt = NVSYNCPT_VBLANK0;
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
			partition_id_disa = tegra_pd_get_powergate_id(
								tegra_disa_pd);
			if (partition_id_disa < 0) {
				ret = -EINVAL;
				goto err_iounmap_reg;
			}
			dc->powergate_id = partition_id_disa;
#ifdef CONFIG_TEGRA_ISOMGR
			isomgr_client_id = TEGRA_ISO_CLIENT_DISP_0;
#endif
			dc->slgc_notifier.notifier_call = tegra_dc_slgc_disp0;
			slcg_register_notifier(dc->powergate_id,
					&dc->slgc_notifier);
#endif
		} else if (dc->ctrl_num == 1) {
			dc->vblank_syncpt = NVSYNCPT_VBLANK1;
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
			partition_id_disb = tegra_pd_get_powergate_id(
								tegra_disb_pd);
			if (partition_id_disb < 0) {
				ret = -EINVAL;
				goto err_iounmap_reg;
			}
			dc->powergate_id = partition_id_disb;
#endif
#ifdef CONFIG_TEGRA_ISOMGR
			isomgr_client_id = TEGRA_ISO_CLIENT_DISP_1;
#endif
		} else {
			dev_err(&ndev->dev, "unknown dc number:%d\n",
				dc->ctrl_num);
		}
	}

	memset(&of_fb_res, 0, sizeof(struct resource));
	if (tegra_dc_is_nvdisplay()) {
		/* on T186+ k4.4, k4.9, k4.14, fb is passed through the DT */
		ret = of_tegra_get_fb_resource(np, &of_fb_res, "surface");
		if (!ret)
			ret = of_tegra_get_fb_resource(np, &of_lut_res, "lut");
	} else {
#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
		/* on T210 k4.14, use the DT -FIXME: debug T210 lut passing */
		(void) of_lut_res;
		ret = of_tegra_get_fb_resource(np, &of_fb_res, "surface");
#else
		/* FIXME: still use the command line for T210 k4.9 */
		tegra_get_fb_resource(&of_fb_res, dc->ctrl_num);
#endif
	}

	if (ret < 0)
		goto err_iounmap_reg;

	fb_mem = kzalloc(sizeof(struct resource), GFP_KERNEL);
	if (fb_mem == NULL) {
		ret = -ENOMEM;
		goto err_iounmap_reg;
	}
	fb_mem->name = "fbmem";
	fb_mem->flags = IORESOURCE_MEM;
	fb_mem->start = (resource_size_t)of_fb_res.start;
	fb_mem->end = (resource_size_t)of_fb_res.end;

	if (tegra_dc_is_nvdisplay())
		snprintf(clk_name, sizeof(clk_name), "nvdisplay_p%u",
			 dc->ctrl_num);
	else
		snprintf(clk_name, sizeof(clk_name), "disp%u",
			 dc->ctrl_num + 1);

	clk = tegra_disp_clk_get(&ndev->dev, clk_name);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&ndev->dev, "can't get clock: %s\n", clk_name);
		ret = -ENOENT;
		goto err_iounmap_reg;
	}

	dc->clk = clk;
	dc->shift_clk_div.mul = dc->shift_clk_div.div = 1;
	/* Initialize one shot work delay, it will be assigned by dsi
	 * according to refresh rate later. */
	dc->one_shot_delay_ms = 40;

	dc->base = base;
	dc->irq = irq;
	dc->ndev = ndev;
	dc->fb_mem = fb_mem;
	dc->pdata = dt_pdata;

	dc->bw_kbps = 0;

	if (tegra_dc_is_nvdisplay()) {
		/* dc variables need to initialized before nvdisp init */
		ret = tegra_nvdisp_init(dc);
		if (ret)
			goto err_iounmap_reg;
	}

	mutex_init(&dc->lock);
	mutex_init(&dc->one_shot_lock);
	mutex_init(&dc->lp_lock);
	mutex_init(&dc->msrmnt_info.lock);
	init_completion(&dc->frame_end_complete);
	init_completion(&dc->crc_complete);
	init_completion(&dc->hpd_complete);
	init_waitqueue_head(&dc->wq);
	init_waitqueue_head(&dc->timestamp_wq);
	INIT_WORK(&dc->vblank_work, tegra_dc_vblank);
	dc->vblank_ref_count = 0;
	INIT_WORK(&dc->frame_end_work, tegra_dc_frame_end);
	INIT_WORK(&dc->vpulse2_work, tegra_dc_vpulse2);
	dc->vpulse2_ref_count = 0;
	INIT_DELAYED_WORK(&dc->underflow_work, tegra_dc_underflow_worker);
	INIT_DELAYED_WORK(&dc->one_shot_work, tegra_dc_one_shot_worker);
	if (tegra_dc_is_nvdisplay())
		INIT_DELAYED_WORK(&dc->vrr_work, tegra_nvdisp_vrr_work);

	if (tegra_dc_is_t21x())
		tegra_dc_init_lut_defaults(&dc->fb_lut);
	if (tegra_dc_is_nvdisplay())
		tegra_dc_init_nvdisp_lut_defaults(&dc->fb_nvdisp_lut);

	dc->n_windows = tegra_dc_get_numof_dispwindows();

	if (tegra_dc_is_t21x()) {
		for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
			struct tegra_dc_win *tmp_win = &dc->tmp_wins[i];
			struct tegra_dc_win *win = &dc->windows[i];

			win->dc = dc;

			if (test_bit(i, &dc->valid_windows)) {
				win->idx = i;
				tmp_win->idx = i;
				tmp_win->dc = dc;

				tegra_dc_init_win_csc_defaults(&win->win_csc);
				tegra_dc_init_lut_defaults(&win->lut);
			}
		}
	}

	if (tegra_dc_is_nvdisplay()) {
		if (dc->pdata->nvdisp_cmu)
			dc->default_csc = dc->pdata->nvdisp_cmu->panel_csc;
		else
			tegra_nvdisp_init_win_csc_defaults(&dc->default_csc);
	}

	platform_set_drvdata(ndev, dc);

#ifdef CONFIG_SWITCH
	dc->modeset_switch.name = dev_name(&ndev->dev);
	dc->modeset_switch.state = 0;
	dc->modeset_switch.print_state = switch_modeset_print_mode;
	ret = switch_dev_register(&dc->modeset_switch);
	if (ret < 0) {
		dev_err(&ndev->dev,
			"failed to register switch driver ret(%d)\n", ret);
		dc->switchdev_registered = false;
	} else
		dc->switchdev_registered = true;
#endif

	tegra_dc_feature_register(dc);

	if (dc->pdata->default_out) {
		if (dc->pdata->default_out->hotplug_init)
			dc->pdata->default_out->hotplug_init(&dc->ndev->dev);
		ret = tegra_dc_set_out(dc, dc->pdata->default_out, false);
		if (ret < 0) {
			dev_err(&dc->ndev->dev, "failed to initialize DC out ops\n");
			goto err_put_clk;
		}
	} else {
		dev_err(&ndev->dev,
			"No default output specified.  Leaving output disabled.\n");
	}
	dc->mode_dirty = false; /* ignore changes tegra_dc_set_out has done */

	if (!dc->out) {
		ret = -EINVAL;
		goto err_put_clk;
	}

	if (tegra_dc_is_nvdisplay())
		nvdisp_register_backlight_notifier(dc);

	dc->boot_topology.disp_id = dc->ndev->id;
	dc->boot_topology.protocol = dc->out->type;

	if (dc->out_ops->get_connector_instance)
		dc->boot_topology.conn_inst =
			dc->out_ops->get_connector_instance(dc);
	else
		dc->boot_topology.conn_inst = TEGRA_DC_TOPOLOGY_INVALID;

	dc->boot_topology.valid = true;
	dc->current_topology = dc->boot_topology;

	if ((dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) &&
		dc->out->type == TEGRA_DC_OUT_LVDS) {
		struct fb_monspecs specs;
		struct tegra_dc_lvds_data *lvds = tegra_dc_get_outdata(dc);
		if (!tegra_edid_get_monspecs(lvds->edid, &specs))
			tegra_dc_set_fb_mode(dc, specs.modedb, false);
	}

#ifndef CONFIG_TEGRA_ISOMGR
	/*
	 * The emc is a shared clock, it will be set based on
	 * the requirements for each user on the bus.
	 */
	snprintf(clk_name, sizeof(clk_name), "disp%u_emc",
			dc->ctrl_num + 1);
	emc_clk = tegra_disp_clk_get(&ndev->dev, clk_name);
	if (IS_ERR_OR_NULL(emc_clk)) {
		dev_err(&ndev->dev, "can't get %s clock\n", clk_name);
		ret = -ENOENT;
		goto err_put_clk;
	}
	dc->emc_clk = emc_clk;
#endif

	/*
	 * The emc_la clock is being added to set the floor value
	 * for emc depending on the LA calculaions for each window
	 */
	if (!tegra_dc_is_nvdisplay()) {
		snprintf(clk_name, sizeof(clk_name), "disp%u_la_emc",
						dc->ctrl_num + 1);
		emc_la_handle = tegra_bwmgr_register(
					display_la_emc_client_id[dc->ctrl_num]);
		if (IS_ERR_OR_NULL(emc_la_handle)) {
			dev_err(&ndev->dev, "can't get handle %s\n", clk_name);
			ret = -ENOENT;
			goto err_disable_dc;
		}
		dc->emc_la_handle = emc_la_handle;
		ret = tegra_bwmgr_set_emc(dc->emc_la_handle, 0,
				TEGRA_BWMGR_SET_EMC_FLOOR);
		if (ret) {
			dev_err(&ndev->dev, "can't set emc clock: %d\n", ret);
			ret = -EINVAL;
			goto err_disable_dc;
		}
	}

	dc->ext = tegra_dc_ext_register(ndev, dc);
	if (IS_ERR_OR_NULL(dc->ext)) {
		dev_warn(&ndev->dev, "Failed to enable Tegra DC extensions.\n");
		dc->ext = NULL;
	}

	/* interrupt handler must be registered before tegra_fb_register() */
	if (request_threaded_irq(irq, NULL, tegra_dc_irq, IRQF_ONESHOT,
			dev_name(&ndev->dev), dc)) {
		dev_err(&ndev->dev, "request_irq %d failed\n", irq);
		ret = -EBUSY;
		goto err_disable_dc;
	}
	disable_dc_irq(dc);

	tegra_pd_add_device(&ndev->dev);
	pm_runtime_use_autosuspend(&ndev->dev);
	pm_runtime_set_autosuspend_delay(&ndev->dev, 100);
	pm_runtime_enable(&ndev->dev);
	/* Enable async suspend/resume to reduce LP0 latency */
	device_enable_async_suspend(&ndev->dev);

	/* if bootloader leaves this head enabled, then skip CMU programming. */
	dc->is_cmu_set_bl = (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) != 0;
	dc->cmu_enabled = dc->pdata->cmu_enable;

#if defined(CONFIG_TEGRA_ISOMGR)
	if (tegra_dc_is_t21x()) {
		if (isomgr_client_id == -1) {
			dc->isomgr_handle = NULL;
		} else {
			dc->isomgr_handle = tegra_isomgr_register(
					isomgr_client_id,
					tegra_dc_calc_min_bandwidth(dc),
					tegra_dc_bandwidth_renegotiate, dc);
			if (IS_ERR(dc->isomgr_handle)) {
				dev_err(&dc->ndev->dev,
						"could not register isomgr. err=%ld\n",
						PTR_ERR(dc->isomgr_handle));
				ret = -ENOENT;
				goto err_disable_dc;
			}
			dc->reserved_bw = tegra_dc_calc_min_bandwidth(dc);
			/*
			 * Use maximum value so we can try to reserve as much as
			 * needed until we are told by isomgr to backoff.
			 */
			dc->available_bw = UINT_MAX;
		}
	}
#endif

	/* Initialize the flip stats to 0. */
	atomic64_set(&dc->flip_stats.flips_queued, 0);
	atomic64_set(&dc->flip_stats.flips_skipped, 0);
	atomic64_set(&dc->flip_stats.flips_cmpltd, 0);

	tegra_dc_create_debugfs(dc);


	dc->frm_lck_info.frame_lock_enable = dc->pdata->frame_lock_enable;
	dc->frm_lck_info.job_pending = false;
	init_waitqueue_head(&(dc->frm_lck_info.win_upd_reqs));

	dev_info(&ndev->dev, "probed\n");

	if (dc->pdata->fb) {
		if (dc->enabled && dc->pdata->fb->bits_per_pixel == -1) {
			unsigned long fmt;
			tegra_dc_writel(dc,
					WINDOW_A_SELECT << dc->pdata->fb->win,
					DC_CMD_DISPLAY_WINDOW_HEADER);

			fmt = tegra_dc_readl(dc, DC_WIN_COLOR_DEPTH);
			dc->pdata->fb->bits_per_pixel =
				tegra_dc_fmt_bpp(fmt);
		}

		mode = tegra_dc_get_override_mode(dc);
		if (mode) {
			dc->pdata->fb->xres = mode->h_active;
			dc->pdata->fb->yres = mode->v_active;
		}

		/* if current mode is not set, add 640x480 to current mode
		 * because fb_register always adds the current mode to the
		 * modelist.
		 */
		if (tegra_dc_is_t21x() && (!dc->mode.pclk || !dc->mode.h_active
					   || !dc->mode.v_active))
			tegra_dc_set_fb_mode(dc, &tegra_dc_vga_mode, false);

		ret = tegra_dc_io_start(dc);
		dc->fb = tegra_fb_register(ndev, dc, dc->pdata->fb, fb_mem);
		if (!ret)
			tegra_dc_io_end(dc);
		if (IS_ERR_OR_NULL(dc->fb)) {
			dc->fb = NULL;
			dev_err(&ndev->dev, "failed to register fb\n");
			ret = -EINVAL;
			goto err_remove_debugfs;
		}
	}

	if (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) {
		/* WAR: BL is putting DC in bad state for EDP configuration */
		if (!tegra_platform_is_vdk() &&
			(dc->out->type == TEGRA_DC_OUT_DP)) {
			if (tegra_dc_is_t21x()) {
				dc->rst = of_reset_control_get(np, "dc_rst");
				if (IS_ERR_OR_NULL(dc->rst)) {
					dev_err(&dc->ndev->dev,
						"Unable to get dc_rst%u reset control\n",
						dc->ctrl_num);
					ret = dc->rst ? PTR_ERR(dc->rst) : -ENOENT;
					goto err_remove_debugfs;
				}

				if (dc->rst) {
					tegra_disp_clk_prepare_enable(dc->clk);
					reset_control_assert(dc->rst);
					udelay(10);
					reset_control_deassert(dc->rst);
					udelay(10);
					tegra_disp_clk_disable_unprepare(
								dc->clk);
				}
			}
		}
	}

	if (dc->out_ops && dc->out_ops->hotplug_init)
		hotplug_init_status = dc->out_ops->hotplug_init(dc);

	if (dc->out->type == TEGRA_DC_OUT_DP) {
		ret = tegra_dc_set_fbcon_boot_mode(dc, dc->edid);
		if (ret)
			dev_err(&dc->ndev->dev,
				"Failed to set fbcon mode for DC %d\n",
				dc->ctrl_num);
	}

	if (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) {
		_tegra_dc_set_default_videomode(dc);
		dc->enabled = _tegra_dc_enable(dc);

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
		/* BL or PG init will keep DISA unpowergated after booting.
		 * Adding an extra powergate to balance the refcount
		 * since _tegra_dc_enable() increases the refcount.
		 */
		if (tegra_dc_is_t21x()) {
			if (!tegra_platform_is_fpga())
				if (dc->powergate_id == TEGRA_POWERGATE_DISA)
					tegra_dc_powergate_locked(dc);
		}
#endif
	}

	if (dc->out_ops) {
		if (dc->out_ops->detect && hotplug_init_status >= 0)
			dc->connected = dc->out_ops->detect(dc);
		else
			dc->connected = true;
	} else
		dc->connected = false;

	/* Powergate display module when it's unconnected. */
	/* detect() function, if presetns, responsible for the powergate */
	if (!tegra_dc_get_connected(dc) &&
			!(dc->out_ops && dc->out_ops->detect))
		tegra_dc_powergate_locked(dc);

	tegra_dc_create_sysfs(&dc->ndev->dev);

	/*
	 * Overriding the display mode only applies for modes set up during
	 * boot. It should not apply for e.g. HDMI hotplug.
	 */
	dc->initialized = false;

	/*
	 * Initialize vedid state. This is placed here
	 * to allow persistence across sw HDMI hotplugs.
	 */
	dc->vedid = false;
	dc->vedid_data = NULL;

	mutex_unlock(&tegra_dc_registration_lock);
	return 0;

err_remove_debugfs:
	tegra_dc_remove_debugfs(dc);
	free_irq(irq, dc);
err_disable_dc:
	if (dc->ext) {
		tegra_dc_ext_disable(dc->ext);
		tegra_dc_ext_unregister(dc->ext);
	}
	mutex_lock(&dc->lock);
	if (dc->enabled)
		_tegra_dc_disable(dc);
	dc->enabled = false;
	mutex_unlock(&dc->lock);
#if defined(CONFIG_TEGRA_ISOMGR)
	if (tegra_dc_is_t21x())
		tegra_isomgr_unregister(dc->isomgr_handle);
#elif !defined(CONFIG_TEGRA_ISOMGR)
	tegra_disp_clk_put(&ndev->dev, emc_clk);
#endif
	if (!tegra_dc_is_nvdisplay())
		tegra_bwmgr_unregister(dc->emc_la_handle);
err_put_clk:
#ifdef CONFIG_SWITCH
	if (dc->switchdev_registered)
		switch_dev_unregister(&dc->modeset_switch);
#endif
	tegra_disp_clk_put(&ndev->dev, clk);
err_iounmap_reg:
	iounmap(base);
	kfree(fb_mem);
err_release_resource_reg:
err_free:
	tegra_dc_clear(dc);
	kfree(dc);
	dc = NULL;
	mutex_unlock(&tegra_dc_registration_lock);

	return ret;
}

static int tegra_dc_remove(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	int i;

	if (!dc)
		return 0;

	if (dc->out->type == TEGRA_DC_OUT_HDMI)
		for (i = 0; i < ARRAY_SIZE(hdmi_extcon_cable_id); i++)
			if (extcon_hdmi_dc_map[i] == dc->ctrl_num) {
				extcon_hdmi_dc_map[i] = -1;
				break;
			}

	tegra_dc_remove_sysfs(&dc->ndev->dev);
	tegra_dc_remove_debugfs(dc);

	if (dc->fb) {
		tegra_fb_unregister(dc->fb);
		kfree(dc->fb_mem);
	}

	if (dc->ext) {
		tegra_dc_ext_disable(dc->ext);
		tegra_dc_ext_unregister(dc->ext);
	}

	kfree(dc->flip_buf.data);
	kfree(dc->crc_buf.data);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}
	mutex_lock(&dc->lock);
	if (dc->enabled)
		_tegra_dc_disable(dc);
	dc->enabled = false;
	mutex_unlock(&dc->lock);
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);
	synchronize_irq(dc->irq); /* wait for IRQ handlers to finish */

#ifdef CONFIG_SWITCH
	if (dc->switchdev_registered)
		switch_dev_unregister(&dc->modeset_switch);
#endif
	free_irq(dc->irq, dc);

#if defined(CONFIG_TEGRA_ISOMGR)
	if (tegra_dc_is_nvdisplay()) {
		if (!tegra_platform_is_vdk())
			tegra_nvdisp_bandwidth_unregister();
	} else {
		if (dc->isomgr_handle) {
			tegra_isomgr_unregister(dc->isomgr_handle);
			dc->isomgr_handle = NULL;
		}
	}
#else
	tegra_disp_clk_put(&ndev->dev, dc->emc_clk);
#endif

	if (!tegra_dc_is_nvdisplay())
		tegra_bwmgr_unregister(dc->emc_la_handle);

	tegra_disp_clk_put(&ndev->dev, dc->clk);
	iounmap(dc->base);
	tegra_dc_clear(dc);
	kfree(dc);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_dc_suspend(struct platform_device *ndev, pm_message_t state)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	int ret = 0;

	if (!dc)
		return ret;

	trace_display_suspend(dc);
	dev_info(&ndev->dev, "suspend\n");

	ret = tegra_dc_reserve_common_channel(dc);
	if (ret) {
		dev_err(&dc->ndev->dev,
			"%s: DC %d suspend failed due to timeout\n",
			__func__, dc->ctrl_num);

		return ret;
	}

	tegra_dc_disable_all_wins(dc);

	tegra_dc_cursor_suspend(dc);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}
	mutex_lock(&dc->lock);
	ret = tegra_dc_io_start(dc);

	if (dc->enabled) {
		_tegra_dc_disable(dc);
		dc->enabled = false;
		dc->reenable_on_resume = true;
	}

	dc->suspended = true;

	if (dc->out_ops && dc->out_ops->suspend)
		dc->out_ops->suspend(dc);

	tegra_dc_release_common_channel(dc);

	if (dc->out && dc->out->postsuspend) {
		dc->out->postsuspend();
		/* avoid resume event due to voltage falling on interfaces that
		 * support hotplug wake. And only do this if a panel is
		 * connected, if we are already disconnected, then no phantom
		 * hotplug can occur by disabling the voltage.
		 */
		if ((dc->out->flags & TEGRA_DC_OUT_HOTPLUG_WAKE_LP0)
			&& tegra_dc_get_connected(dc))
			msleep(100);
	}

	if (!ret)
		tegra_dc_io_end(dc);

	tegra_dc_syncpt_flush(dc->ndev, dc->vblank_syncpt);
	mutex_unlock(&dc->lock);
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);
	synchronize_irq(dc->irq); /* wait for IRQ handlers to finish */

	return 0;
}

static int tegra_dc_resume(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	if (!dc)
		return 0;

	trace_display_resume(dc);
	dev_info(&ndev->dev, "resume\n");

	tegra_dc_syncpt_preset_maxval(dc->ndev, dc->vblank_syncpt);

	/* To pan the fb on resume */
	tegra_fb_pan_display_reset(dc->fb);

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init(&ndev->dev);

	if (dc->out_ops && dc->out_ops->resume)
		dc->out_ops->resume(dc);

	mutex_lock(&dc->lock);
	if (!dc->enabled && dc->reenable_on_resume)
		dc->enabled = _tegra_dc_enable(dc);
	dc->reenable_on_resume = false;
	dc->suspended = false;

	mutex_unlock(&dc->lock);
	tegra_dc_cursor_resume(dc);

	return 0;
}

#endif /* CONFIG_PM */

void tegra_dc_shutdown(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

	if (!dc->enabled)
		return;

	kfree(dc->vedid_data);
	dc->vedid_data = NULL;
	dc->vedid = false;


	/* Let dc clients know about shutdown event before calling disable */
	if (dc->out_ops && dc->out_ops->shutdown)
		dc->out_ops->shutdown(dc);

	tegra_dc_disable(dc);
}

static int suspend_set(const char *val, struct kernel_param *kp)
{
	if (!strcmp(val, "dump"))
		dump_regs(tegra_dcs[0]);
#ifdef CONFIG_PM
	else if (!strcmp(val, "suspend"))
		tegra_dc_suspend(tegra_dcs[0]->ndev, PMSG_SUSPEND);
	else if (!strcmp(val, "resume"))
		tegra_dc_resume(tegra_dcs[0]->ndev);
#endif

	return 0;
}

static int suspend_get(char *buffer, struct kernel_param *kp)
{
	return 0;
}

static int suspend;
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 135)) && (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)))
module_param_call(suspend, (void *)suspend_set, (void *)suspend_get, &suspend, 0644);
#else
module_param_call(suspend, suspend_set, suspend_get, &suspend, 0644);
#endif

#ifndef MODULE
static int __init parse_disp_params(char *options, struct tegra_dc_mode *mode)
{
	int i, params[11];
	char *p;

	memset(params, 0, ARRAY_SIZE(params) * sizeof(int));
	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if ((p = strsep(&options, ",")) != NULL) {
			if (*p)
				params[i] = simple_strtoul(p, &p, 10);
		} else
			return -EINVAL;
	}

	if ((mode->pclk = params[0]) == 0)
		return -EINVAL;

	mode->h_active      = params[1];
	mode->v_active      = params[2];
	mode->h_ref_to_sync = params[3];
	mode->v_ref_to_sync = params[4];
	mode->h_sync_width  = params[5];
	mode->v_sync_width  = params[6];
	mode->h_back_porch  = params[7];
	mode->v_back_porch  = params[8];
	mode->h_front_porch = params[9];
	mode->v_front_porch = params[10];

	return 0;
}

static int __init tegra_dc_mode_override(char *str)
{
	char *p = str, *options;

	if (!p || !*p)
		return -EINVAL;

	p = strstr(str, "hdmi:");
	if (p) {
		p += 5;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_HDMI]))
			return -EINVAL;
	}

	p = strstr(str, "rgb:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_RGB]))
			return -EINVAL;
	}

	p = strstr(str, "dsi:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_DSI]))
			return -EINVAL;
	}

	p = strstr(str, "null:");
	if (p) {
		p += 5;
		options = strsep(&p, ";");
		if (parse_disp_params(options,
				&override_disp_mode[TEGRA_DC_OUT_NULL]))
			return -EINVAL;
	}

	return 0;
}

__setup("disp_params=", tegra_dc_mode_override);
#endif


#ifdef TEGRA_DC_USR_SHARED_IRQ

static struct tegra_dc  *tegra_dc_hwidx2dc(int dcid)
{
	struct tegra_dc  *dc;
	int              i;

	for (i = 0; i < tegra_dc_get_numof_dispheads(); i++) {
		dc = tegra_dc_get_dc(i);
		if (dc && (dcid == dc->ctrl_num))
			return dc;
	}

	return NULL;
}

/*
 * get Tegra display head status
 * o inputs:
 *  - dcid: display head HW index (0 to max_dc_heads-1)
 *  - pSts: pointer to the head status structure to be returned
 * o outputs:
 *  - return: error number
 *   . 0: registration successful without an error
 *   . !0: registration failed with an error
 *  - *pSts: head status
 * o notes:
 */
int  tegra_dc_get_disphead_sts(int dcid, struct tegra_dc_head_status *pSts)
{
	struct tegra_dc  *dc = tegra_dc_hwidx2dc(dcid);

	if (dc) {
		pSts->magic = TEGRA_DC_HEAD_STATUS_MAGIC1;
		pSts->irqnum = dc->irq;
		pSts->init = dc->initialized ? 1 : 0;
		pSts->connected = dc->connected ? 1 : 0;
		pSts->active = dc->enabled ? 1 : 0;
		return 0;
	} else {
		return -ENODEV;
	}
}
EXPORT_SYMBOL(tegra_dc_get_disphead_sts);

/*
 * to register the Tegra display ISR user call-back routine
 * o inputs:
 *  - dcid: display head HW index (0 to max_dc_heads-1)
 *  - usr_isr_cb: function pointer to the user call-back routine
 *  - usr_isr_pdt: user call-back private data
 * o outputs:
 *  - return: error code
 *   . 0: registration successful without an error
 *   . !0: registration failed with an error
 * o notes: will overwrite the old CB always
 */
int  tegra_dc_register_isr_usr_cb(int dcid,
	int (*usr_isr_cb)(int dcid, unsigned long irq_sts, void *usr_isr_pdt),
	void *usr_isr_pdt)
{
	struct tegra_dc  *dc = tegra_dc_hwidx2dc(dcid);

	/* register usr ISR */
	if (dc && usr_isr_cb) {
		if (dc->isr_usr_cb) {
			dev_warn(&dc->ndev->dev,
				"%s DC%d: overwriting ISR USR CB:%p PDT:%p\n",
				 __func__, dcid,
				 dc->isr_usr_cb, dc->isr_usr_pdt);
		}
		mutex_lock(&dc->lock);
		/* always replace the old ISR */
		dc->isr_usr_cb  = usr_isr_cb;
		dc->isr_usr_pdt = usr_isr_pdt;
		mutex_unlock(&dc->lock);
		dev_info(&dc->ndev->dev,
			"DC%d: ISR USR CB:%p PDT:%p registered\n",
			dcid, usr_isr_cb, usr_isr_pdt);
		return 0;
	} else {
		return dc ? -EINVAL : -ENODEV;
	}
}
EXPORT_SYMBOL(tegra_dc_register_isr_usr_cb);

/*
 * to unregister the Tegra display ISR user call-back routine
 * o inputs:
 *  - dcid: display head HW index (0 to max_dc_heads-1)
 *  - usr_isr_cb: registered user call-back. ignored.
 *  - usr_isr_pdt: registered user call-back private data. ignored.
 * o outputs:
 *  - return: error code
 *   . 0: unregistration successful
 *   . !0: unregistration failed with an error
 * o notes: will unregister the current CB always
 */
int  tegra_dc_unregister_isr_usr_cb(int dcid,
	int (*usr_isr_cb)(int dcid, unsigned long irq_sts, void *usr_isr_pdt),
	void *usr_isr_pdt)
{
	struct tegra_dc  *dc = tegra_dc_hwidx2dc(dcid);

	/* unregister USR ISR CB */
	if (dc) {
		mutex_lock(&dc->lock);
		dc->isr_usr_cb = NULL;
		dc->isr_usr_pdt = NULL;
		mutex_unlock(&dc->lock);
		dev_info(&dc->ndev->dev,
			"DC%d: USR ISR CB unregistered\n", dcid);
		return 0;
	} else {
		return -ENODEV;
	}
}
EXPORT_SYMBOL(tegra_dc_unregister_isr_usr_cb);

#endif /* TEGRA_DC_USR_SHARED_IRQ */

/**
 * tegra_dc_enable_disable_frame_lock - enables/disables frame_lock in dc.
 * @dc: Pointer to tegra_dc struct.
 * @enable: Boolean value for enabling or disabling.
 *
 * The only call(entry point) to this function should be from dc_common.
 * dc_common makes sure that dc!=NULL before calling this API.
 *
 * Return: void
 */
void tegra_dc_enable_disable_frame_lock(struct tegra_dc *dc, bool enable)
{
	mutex_lock(&dc->lock);
	dc->frm_lck_info.frame_lock_enable = enable;
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_enable_disable_frame_lock);

/**
 * tegra_dc_request_trigger_wins - enables/disables frame_lock in dc.
 * @dc: Pointer to tegra_dc struct.
 * @enable: Boolean value for status.
 *
 * Return: void
 */
void tegra_dc_request_trigger_wins(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	tegra_dc_trigger_windows(dc);
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_request_trigger_wins);

int  tegra_dc_get_numof_dispheads(void)
{
	if (!hw_data || !hw_data->valid)
		return -ENODEV;

	return hw_data->nheads;
}
EXPORT_SYMBOL(tegra_dc_get_numof_dispheads);

int tegra_dc_get_numof_dispwindows(void)
{
	if (!hw_data || !hw_data->valid)
		return DC_N_WINDOWS;

	return hw_data->nwins;
}
EXPORT_SYMBOL(tegra_dc_get_numof_dispwindows);

int tegra_dc_get_numof_dispsors(void)
{
	if (!hw_data || !hw_data->valid)
		return -ENODEV;

	return hw_data->nsors;
}
EXPORT_SYMBOL(tegra_dc_get_numof_dispsors);

struct tegra_dc_sor_info *tegra_dc_get_sor_cap(void)
{
	if (!hw_data || !hw_data->valid)
		return NULL;

	return hw_data->sor_info;
}

/* tegra_dc_get_max_lines() - gets v_total for current mode
 * @disp_id : the display id of the concerned head.
 *
 * Return : v_total if successful else error value.
 */
int tegra_dc_get_max_lines(int disp_id)
{
	int max_lines;
	struct tegra_dc *dc;
	struct tegra_dc_mode *m;

	dc = tegra_dc_get_dc(disp_id);
	if (!dc)
		return -ENODEV;

	m = &dc->mode;

	max_lines = m->v_back_porch + m->v_active +
			m->v_front_porch + m->v_sync_width;

	return max_lines;
}
EXPORT_SYMBOL(tegra_dc_get_max_lines);

/* tegra_dc_get_addr_info() - gets the base address for a head
 * @disp_id : the display id of the concerned head.
 * @res : ptr to the resource from the caller.
 *
 * Return : 0 if successful else error value.
 */
int tegra_dc_get_addr_info(int disp_id, struct resource *res)
{
	int ret;
	struct tegra_dc *dc;

	dc = tegra_dc_get_dc(disp_id);
	if (!dc || !res)
		return -ENODEV;

	ret = of_address_to_resource(dc->ndev->dev.of_node, 0, res);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(tegra_dc_get_addr_info);
/*
 * tegra_dc_get_vsync_timestamp() - Reads the vsync timestamp.
 * @dc : pointer to struct tegra_dc for the current head.
 *
 * Currently supported for one chip version. TODO for others.
 *
 * Return : The 64 bit timestamp value.
 */
uint64_t tegra_dc_get_vsync_timestamp(struct tegra_dc *dc)
{
	if (!dc->enabled)
		return 0;

	if (tegra_dc_is_t21x() || tegra_dc_is_t18x())
		return 0;
	else if (tegra_dc_is_t19x())
		return tegra_dc_get_vsync_timestamp_t19x(dc);

	pr_warn("%s: Couldn't find the right chip version\n", __func__);

	return 0;
}

/*
 * tegra_dc_line2ns() - gets the time required to scan a given number of lines.
 * @dc : points to struct tegra_dc for the current head.
 *
 * Return : time in nanosecs
 */
static inline u64 tegra_dc_line2ns(struct tegra_dc *dc, int nr_lines)
{
	return nr_lines * dc->mode_metadata.line_in_nsec;
}

/*
 * tegra_dc_get_scanline_timestamp() - gets the timestamp of a scanline
 * @dc : pointer to struct tegra_dc of the cuurent head.
 * @scanline : the scanline for which timestamp is needed.
 *
 * This helper function gives the timestamp of the @scanline when it occurred
 * last.
 *
 * Return : the timestamp value.
 */
static u64 tegra_dc_get_scanline_timestamp(struct tegra_dc *dc,
						const u32 scanline)
{
	ktime_t ts;
	u64 timestamp;
	int curr_scanline;

	curr_scanline = tegra_dc_get_v_count(dc);
	ts = ktime_get();

	timestamp = ktime_to_ns(ts);
	if (scanline < curr_scanline)
		timestamp -= tegra_dc_line2ns(dc, curr_scanline - scanline);
	else
		timestamp -= tegra_dc_line2ns(dc, curr_scanline +
				(dc->mode_metadata.vtotal_lines - scanline));
	return timestamp;
}

/*
 * tegra_dc_collect_latency_data() - stores relevant info needed
 *					for latency instrumentation.
 * @dc : pointer to struct tegra_dc of the current head.
 *
 * Currently supports nvdisplay only. Using the already stored
 * dma_buff_handle and the relevant offset, reads the first 2 pixels of
 * framebuffer. Returns if not nvdisplay, or the latency_measuring
 * functionality is disabled or the handle is NULL.
 *
 * Return : void
 */
static void tegra_dc_collect_latency_data(struct tegra_dc *dc)
{
	int ret;
	struct dma_buf *handle;
	int page_num = 0;
	void *ptr;
	u64 value;

	if (tegra_dc_is_t21x() || !dc->enabled)
		return;

	mutex_lock(&dc->msrmnt_info.lock);

	handle = dc->msrmnt_info.buf_handle;

	if (!handle || !dc->msrmnt_info.enabled) {
		dev_dbg(&dc->ndev->dev,
			"dma_buff is NULL or latency collection is disabled\n");
		mutex_unlock(&dc->msrmnt_info.lock);
		return;
	}

	ret = dma_buf_begin_cpu_access(handle, 0,
				handle->size, DMA_BIDIRECTIONAL);

	if (ret) {
		dev_err(&dc->ndev->dev, "dma_buf_begin_cpu_access failed\n");
		mutex_unlock(&dc->msrmnt_info.lock);
		return;
	}

	ptr = dma_buf_kmap(handle, page_num);
	if (!ptr) {
		dev_err(&dc->ndev->dev, "dma_buf_kmap failed\n");
		dma_buf_end_cpu_access(handle, 0,
				handle->size, DMA_BIDIRECTIONAL);
		mutex_unlock(&dc->msrmnt_info.lock);
		return;
	}
	value = *((u64 *)(ptr + dc->msrmnt_info.offset));
	trace_display_embedded_latency(dc->ctrl_num,
			dc->msrmnt_info.line_num, be64_to_cpup(&value));

	dma_buf_kunmap(handle, page_num, ptr);
	dma_buf_end_cpu_access(handle, 0, handle->size, DMA_BIDIRECTIONAL);
	mutex_unlock(&dc->msrmnt_info.lock);
}

/*
 * tegra_dc_en_dis_latency_msrmnt_mode() - enables/disables latency
 *					measurement mode in dc.
 * @dc : pointer to struct tegra_dc of the current head.
 * @enable : tells if the latency_measuring functionality has to be enabled
 * or disabled.
 *
 * This wrapper functions calls into the pertinent enable/disable function based
 * on the chip type.
 *
 * Return : 0 if successful else relevant error number
 */
int tegra_dc_en_dis_latency_msrmnt_mode(struct tegra_dc *dc, int enable)
{
	if (!dc)
		return -ENODEV;

	if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_set_msrmnt_mode(dc, enable);

	return 0;
}

struct tegra_dc_pd_table *tegra_dc_get_disp_pd_table(void)
{
	if (!hw_data || !hw_data->valid)
		return ERR_PTR(-ENODEV);

	return hw_data->pd_table;
}

static int tegra_dc_assign_hw_data(void)
{
	const struct of_device_id *match = NULL;

	if (hw_data) {
		pr_warn("%s: hw data already assigned\n", __func__);
		return 0;
	}

	of_find_matching_node_and_match(NULL, tegra_display_of_match, &match);
	if (!match) {
		pr_err("%s: no matching compatible node\n", __func__);
		return -ENODEV;
	}

	hw_data = (struct tegra_dc_hw_data *)match->data;
	if (!hw_data->valid) {
		hw_data = NULL;
		pr_err("%s: hw_data is not valid for %s\n", __func__,
			match->compatible);
		return -EINVAL;
	}

	return 0;
}

inline bool tegra_dc_is_t21x(void)
{
	return hw_data && (hw_data->version == TEGRA_DC_HW_T210);
}

inline bool tegra_dc_is_t18x(void)
{
	return hw_data && (hw_data->version == TEGRA_DC_HW_T18x);
}

inline bool tegra_dc_is_t19x(void)
{
	return hw_data && (hw_data->version == TEGRA_DC_HW_T19x);
}

inline bool tegra_dc_is_nvdisplay(void)
{
	return (tegra_dc_is_t18x() || tegra_dc_is_t19x());
}
EXPORT_SYMBOL(tegra_dc_is_nvdisplay);

static struct tegra_dc_sor_info t21x_sor_info[] = {
	{ .hdcp_supported = false },  /* SOR0 */
	{ .hdcp_supported = true },   /* SOR1 */
};

static void tegra_dc_populate_t21x_hw_data(struct tegra_dc_hw_data *hw_data)
{
	if (!hw_data)
		return;

	hw_data->nheads = 2;
	hw_data->nwins = 5;
	hw_data->nsors = 2;
	hw_data->sor_info = t21x_sor_info;

	/* unused */
	hw_data->pd_table = NULL;

	hw_data->valid = true;
	hw_data->version = TEGRA_DC_HW_T210;
}

/**
 * tegra_dc_hw_init - Initializes hardware specific data for dc.
 *
 * The only call(entry point) to this function as of today should
 * be from dc_common. Since dc_common needs to be probed before
 * tegradc and the former also needs some hardware related info,
 * dc_common initializes hw data in its module_init(). The order
 * of calls to module_init() is kept as dc_common's followed by
 * tegra_dc's module_init().
 *
 * Return: 0 if success else corresponding error number from
 * @tegra_dc_assign_hw_data().
 */
int tegra_dc_hw_init(void)
{
	int ret;

	tegra_dc_populate_t21x_hw_data(&t21x_hw_data);
	if (tegra_dc_populate_t18x_hw_data)
		tegra_dc_populate_t18x_hw_data(&t18x_hw_data);
	if (tegra_dc_populate_t19x_hw_data)
		tegra_dc_populate_t19x_hw_data(&t19x_hw_data);

	ret = tegra_dc_assign_hw_data();

	return ret;
}

static struct platform_driver tegra_dc_driver = {
	.driver = {
		.name = "tegradc",
		.owner = THIS_MODULE,
		.of_match_table =
			of_match_ptr(tegra_display_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = tegra_dc_probe,
	.remove = tegra_dc_remove,
#ifdef CONFIG_PM
	.suspend = tegra_dc_suspend,
	.resume = tegra_dc_resume,
#endif
	.shutdown = tegra_dc_shutdown,
};

static int __init tegra_dc_module_init(void)
{
	int ret;
	int max_heads;
#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) && defined(CONFIG_DEBUG_FS)
	int i;
#endif
	max_heads = tegra_dc_get_numof_dispheads();
	if (max_heads < 0) {
		printk(KERN_ERR "tegradc module_init failed\n");
		return -ENOENT;
	}

	tegra_dcs = kzalloc(max_heads *	sizeof(struct tegra_dc *), GFP_KERNEL);
	if (!tegra_dcs)
		return -ENOMEM;

#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) && defined(CONFIG_DEBUG_FS)
	boot_out_type = kzalloc(max_heads * sizeof(int), GFP_KERNEL);
	if (!boot_out_type) {
		kfree(tegra_dcs);
		return -ENOMEM;
	}
	for (i = 0; i < max_heads; i++)
		boot_out_type[i] = -1;
#endif

	ret = tegra_dc_ext_module_init();
	if (ret) {
#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) && defined(CONFIG_DEBUG_FS)
		kfree(boot_out_type);
#endif
		kfree(tegra_dcs);
		return ret;
	}

	return platform_driver_register(&tegra_dc_driver);
}

static void __exit tegra_dc_module_exit(void)
{
#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) && defined(CONFIG_DEBUG_FS)
	kfree(boot_out_type);
#endif
	kfree(tegra_dcs);
	platform_driver_unregister(&tegra_dc_driver);
	tegra_dc_ext_module_exit();
}

module_exit(tegra_dc_module_exit);
module_init(tegra_dc_module_init);
