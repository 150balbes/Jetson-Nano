/*
 * Cryptographic API.
 * drivers/crypto/tegra-se.c
 *
 * Support for Tegra Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/version.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/ahb.h>
#include <crypto/scatterwalk.h>
#include <soc/tegra/pmc.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/akcipher.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/akcipher.h>
#include <crypto/sha.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <crypto/internal/kpp.h>
#include <crypto/kpp.h>
#include <crypto/dh.h>

#include "tegra-se.h"

#define DRIVER_NAME	"tegra-se"

#if defined(CONFIG_PM)
static struct device *save_se_device;
#endif

enum tegra_se_dev_algo {
	SE_DRBG,
	SE_AES,
	SE_CMAC,
	SE_RSA,
	SE_SHA,
	NUM_SE_ALGO,
};

static struct tegra_se_dev *se_devices[NUM_SE_ALGO];

/* Security Engine operation modes */
enum tegra_se_aes_op_mode {
	SE_AES_OP_MODE_CBC,	/* Cipher Block Chaining (CBC) mode */
	SE_AES_OP_MODE_ECB,	/* Electronic Codebook (ECB) mode */
	SE_AES_OP_MODE_CTR,	/* Counter (CTR) mode */
	SE_AES_OP_MODE_OFB,	/* Output feedback (CFB) mode */
	SE_AES_OP_MODE_RNG_X931,	/* Random number generator (RNG) mode */
	SE_AES_OP_MODE_RNG_DRBG,	/* Deterministic Random Bit Generator */
	SE_AES_OP_MODE_CMAC,	/* Cipher-based MAC (CMAC) mode */
	SE_AES_OP_MODE_SHA1,	/* Secure Hash Algorithm-1 (SHA1) mode */
	SE_AES_OP_MODE_SHA224,	/* Secure Hash Algorithm-224  (SHA224) mode */
	SE_AES_OP_MODE_SHA256,	/* Secure Hash Algorithm-256  (SHA256) mode */
	SE_AES_OP_MODE_SHA384,	/* Secure Hash Algorithm-384  (SHA384) mode */
	SE_AES_OP_MODE_SHA512	/* Secure Hash Algorithm-512  (SHA512) mode */
};

/* Security Engine key table type */
enum tegra_se_key_table_type {
	SE_KEY_TABLE_TYPE_KEY,	/* Key */
	SE_KEY_TABLE_TYPE_ORGIV,	/* Original IV */
	SE_KEY_TABLE_TYPE_UPDTDIV	/* Updated IV */
};

/* Security Engine request context */
struct tegra_se_req_context {
	enum tegra_se_aes_op_mode op_mode; /* Security Engine operation mode */
	bool encrypt;	/* Operation type */
};

struct tegra_se_chipdata {
	bool cprng_supported;
	bool drbg_supported;
	bool rsa_supported;
	bool drbg_src_entropy_clk_enable;
	bool const_freq;
	unsigned long aes_freq;
	unsigned long rng_freq;
	unsigned long sha1_freq;
	unsigned long sha224_freq;
	unsigned long sha256_freq;
	unsigned long sha384_freq;
	unsigned long sha512_freq;
	unsigned long rsa_freq;
	bool mccif_supported;
	bool rsa_key_rw_op;
	u32 aes_keydata_reg_sz;
	bool ahb_ack;
	bool handle_sc7;
};

struct tegra_se_dev {
	struct device *dev;
	void __iomem *io_reg;	/* se device memory/io */
	int irq;	/* irq allocated */
	spinlock_t lock;	/* spin lock */
	struct clk *pclk;	/* Security Engine clock */
	struct clk *enclk;	/* Entropy clock */
	struct crypto_queue queue; /* Security Engine crypto queue */
	struct tegra_se_slot *slot_list;	/* pointer to key slots */
	struct tegra_se_rsa_slot *rsa_slot_list; /* rsa key slot pointer */
	u64 ctr;
	u32 *src_ll_buf;	/* pointer to source linked list buffer */
	dma_addr_t src_ll_buf_adr; /* Source linked list buffer dma address */
	u32 src_ll_size;	/* Size of source linked list buffer */
	u32 *dst_ll_buf;	/* pointer to destination linked list buffer */
	dma_addr_t dst_ll_buf_adr; /* Destination linked list dma address */
	u32 dst_ll_size;	/* Size of destination linked list buffer */
	u32 *ctx_save_buf;	/* LP context buffer pointer*/
	dma_addr_t ctx_save_buf_adr;	/* LP context buffer dma address*/
	u32 *sg_in_buf;
	dma_addr_t sg_in_buf_adr;
	u32 *sg_out_buf;
	dma_addr_t sg_out_buf_adr;
	u32 *dh_buf1, *dh_buf2;
	struct completion complete;	/* Tells the task completion */
	bool work_q_busy;	/* Work queue busy status */
	bool polling;
	struct tegra_se_chipdata *chipdata; /* chip specific data */
	u32 ahb_id;
};

static struct tegra_se_dev *sg_tegra_se_dev;

/* Security Engine AES context */
struct tegra_se_aes_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u32 op_mode;	/* AES operation mode */
};

/* Security Engine random number generator context */
struct tegra_se_rng_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 *dt_buf;	/* Destination buffer pointer */
	dma_addr_t dt_buf_adr;	/* Destination buffer dma address */
	u32 *rng_buf;	/* RNG buffer pointer */
	dma_addr_t rng_buf_adr;	/* RNG buffer dma address */
};

struct tegra_se_sha_zero_length_vector {
	unsigned int size;
	char *digest;
};

/* Security Engine SHA context */
struct tegra_se_sha_context {
	struct tegra_se_dev	*se_dev;	/* Security Engine device */
	u32 op_mode;	/* SHA operation mode */
	u32 total_count; /* Total bytes in all the requests */
};

/* Security Engine AES CMAC context */
struct tegra_se_aes_cmac_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u8 K1[TEGRA_SE_KEY_128_SIZE];	/* Key1 */
	u8 K2[TEGRA_SE_KEY_128_SIZE];	/* Key2 */
	dma_addr_t dma_addr;	/* DMA address of local buffer */
	u32 buflen;	/* local buffer length */
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

static struct tegra_se_slot srk_slot = {
	.slot_num = 0,
	.available = false,
};

/* Security Engine Linked List */
struct tegra_se_ll {
	u32 addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

static LIST_HEAD(key_slot);
static LIST_HEAD(rsa_key_slot);
static DEFINE_SPINLOCK(rsa_key_slot_lock);

#define RSA_MIN_SIZE	64
#define RSA_MAX_SIZE	256
#define DISK_ENCR_BUF_SZ	512
#define RNG_RESEED_INTERVAL	0x00773594
#define TEGRA_SE_RSA_CONTEXT_SAVE_KEYSLOT_COUNT	2

#define MIN_DH_SZ_BITS  1536

static DEFINE_SPINLOCK(key_slot_lock);
static DEFINE_MUTEX(se_hw_lock);

/* create a work for handling the async transfers */
static void tegra_se_work_handler(struct work_struct *work);
static DECLARE_WORK(se_work, tegra_se_work_handler);
static struct workqueue_struct *se_work_q;

#define PMC_SCRATCH43_REG_OFFSET 0x22c
#define GET_MSB(x)  ((x) >> (8 * sizeof(x) - 1))
static int force_reseed_count;
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

static inline void se_writel(struct tegra_se_dev *se_dev,
			     unsigned int val, unsigned int reg_offset)
{
	writel(val, se_dev->io_reg + reg_offset);
}

static inline unsigned int se_readl(struct tegra_se_dev *se_dev,
				    unsigned int reg_offset)
{
	unsigned int val;

	val = readl(se_dev->io_reg + reg_offset);

	return val;
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
		if (slot->available) {
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
		 * Slot 0 and 15 are reserved and will not be added to the
		 * free slots pool. Slot 0 is used for SRK generation and
		 * Slot 15 is used for SSK operation
		 */
		if ((i == srk_slot.slot_num) || (i == ssk_slot.slot_num))
			continue;
		se_dev->slot_list[i].available = true;
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}
	spin_unlock(&key_slot_lock);

	return 0;
}

static void tegra_se_key_read_disable(u8 slot_num)
{
	struct tegra_se_dev *se_dev = sg_tegra_se_dev;
	u32 val;

	val = se_readl(se_dev,
		       (SE_KEY_TABLE_ACCESS_REG_OFFSET + (slot_num * 4)));
	val &= ~(1 << SE_KEY_READ_DISABLE_SHIFT);
	se_writel(se_dev,
		  val, (SE_KEY_TABLE_ACCESS_REG_OFFSET + (slot_num * 4)));
}

static void tegra_se_key_read_disable_all(void)
{
	struct tegra_se_dev *se_dev = sg_tegra_se_dev;
	u8 slot_num;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	for (slot_num = 0; slot_num < TEGRA_SE_KEYSLOT_COUNT; slot_num++)
		tegra_se_key_read_disable(slot_num);

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);
}

static void tegra_se_config_algo(struct tegra_se_dev *se_dev,
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
	case SE_AES_OP_MODE_RNG_X931:
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
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA1) |
			SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA224:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA224) |
			SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA256:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA256) |
			SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA384:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA384) |
			SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA512:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA512) |
			SE_CONFIG_DST(DST_HASHREG);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}

	se_writel(se_dev, val, SE_CONFIG_REG_OFFSET);
}

static void tegra_se_write_seed(struct tegra_se_dev *se_dev, u32 *pdata)
{
	u32 i;

	for (i = 0; i < SE_CRYPTO_CTR_REG_COUNT; i++)
		se_writel(se_dev, pdata[i], SE_CRYPTO_CTR_REG_OFFSET + (i * 4));
}

static void tegra_se_write_key_table(u8 *pdata, u32 data_len, u8 slot_num,
				     enum tegra_se_key_table_type type)
{
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	u32 data_size;
	u32 *pdata_buf = (u32 *)pdata;
	u8 pkt = 0, quad = 0;
	u32 val = 0, i;

	if (!pdata_buf)
		return;

	if ((type == SE_KEY_TABLE_TYPE_KEY) &&
	    (slot_num == ssk_slot.slot_num))
		return;

	if (type == SE_KEY_TABLE_TYPE_ORGIV)
		quad = QUAD_ORG_IV;
	else if (type == SE_KEY_TABLE_TYPE_UPDTDIV)
		quad = QUAD_UPDTD_IV;
	else
		quad = QUAD_KEYS_128;

	/* write data to the key table */

	if (se_dev->chipdata->aes_keydata_reg_sz == 128) {
		data_size = SE_KEYTABLE_REG_MAX_DATA;
		do {
			for (i = 0; i < data_size; i += 4, data_len -= 4)
				se_writel(se_dev, *pdata_buf++,
					  SE_KEYTABLE_DATA0_REG_OFFSET + i);

			pkt = SE_KEYTABLE_SLOT(slot_num) |
				SE_KEYTABLE_QUAD(quad);
			val = SE_KEYTABLE_OP_TYPE(OP_WRITE) |
				SE_KEYTABLE_TABLE_SEL(TABLE_KEYIV) |
				SE_KEYTABLE_PKT(pkt);

			se_writel(se_dev, val, SE_KEYTABLE_REG_OFFSET);

			data_size = data_len;
			quad = QUAD_KEYS_256;
		} while (data_len);
	} else {
		data_size = SE_KEYTABLE_QUAD_SIZE_BYTES;
		do {
			pkt = SE_KEYTABLE_SLOT(slot_num) |
				SE_KEYTABLE_QUAD(quad);

			for (i = 0; i < data_size; i += 4, data_len -= 4) {
				val = SE_KEYTABLE_PKT(pkt) |
					SE_KEYTABLE_IV_WORD(i / 4);
				se_writel(se_dev, val, SE_KEYTABLE_REG_OFFSET);
				se_writel(se_dev, *pdata_buf++,
					  SE_KEYTABLE_DATA_REG_OFFSET);
			}

			data_size = data_len;
			quad = QUAD_KEYS_256;
		} while (data_len);
	}
}

static void tegra_se_config_crypto(struct tegra_se_dev *se_dev,
				   enum tegra_se_aes_op_mode mode, bool encrypt,
				   u8 slot_num, bool org_iv)
{
	u32 val = 0;
	unsigned long freq = 0;
	int err = 0;

	switch (mode) {
	case SE_AES_OP_MODE_CMAC:
	case SE_AES_OP_MODE_CBC:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_AHB) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_AESOUT) |
				SE_CRYPTO_XOR_POS(XOR_TOP) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_AHB) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_PREVAHB) |
				SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_RNG_X931:
		val = SE_CRYPTO_INPUT_SEL(INPUT_AHB) |
			SE_CRYPTO_XOR_POS(XOR_BYPASS) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->rng_freq;
		break;
	case SE_AES_OP_MODE_RNG_DRBG:
		val = SE_CRYPTO_INPUT_SEL(INPUT_RANDOM) |
			SE_CRYPTO_XOR_POS(XOR_BYPASS) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		if (tegra_get_chip_id() == TEGRA114)
			val |= SE_CRYPTO_KEY_INDEX(slot_num);
		freq = se_dev->chipdata->rng_freq;
		break;
	case SE_AES_OP_MODE_ECB:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_AHB) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_AHB) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_CTR:
		val = SE_CRYPTO_INPUT_SEL(INPUT_LNR_CTR) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_AHB) |
			SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_OFB:
		val = SE_CRYPTO_INPUT_SEL(INPUT_AESOUT) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_AHB) |
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

	if (se_dev->pclk && !se_dev->chipdata->const_freq) {
		err = clk_set_rate(se_dev->pclk, freq);
		if (err) {
			dev_err(se_dev->dev, "clock set_rate failed.\n");
			return;
		}
	}

	/* enable hash for CMAC */
	if (mode == SE_AES_OP_MODE_CMAC)
		val |= SE_CRYPTO_HASH(HASH_ENABLE);

	if (se_dev->chipdata->mccif_supported)
		val |= SE_CRYPTO_MEMIF(MEMIF_MCCIF);
	else
		val |= SE_CRYPTO_MEMIF(MEMIF_AHB);

	se_writel(se_dev, val, SE_CRYPTO_REG_OFFSET);

	if (mode == SE_AES_OP_MODE_RNG_DRBG) {
		if (force_reseed_count <= 0) {
			val = SE_RNG_CONFIG_MODE(DRBG_MODE_FORCE_RESEED) |
				SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY);
			se_writel(se_dev, val, SE_RNG_CONFIG_REG_OFFSET);
		force_reseed_count = RNG_RESEED_INTERVAL;
		} else {
			val = SE_RNG_CONFIG_MODE(DRBG_MODE_NORMAL) |
				SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY);
			se_writel(se_dev, val, SE_RNG_CONFIG_REG_OFFSET);
		}
		--force_reseed_count;

		se_writel(se_dev, RNG_RESEED_INTERVAL,
			  SE_RNG_RESEED_INTERVAL_REG_OFFSET);
	}

	if (mode == SE_AES_OP_MODE_CTR)
		se_writel(se_dev, 1, SE_SPARE_0_REG_OFFSET);

	if (mode == SE_AES_OP_MODE_OFB)
		se_writel(se_dev, 1, SE_SPARE_0_REG_OFFSET);
}

static void tegra_se_config_sha(struct tegra_se_dev *se_dev, u32 count,
				unsigned long freq)
{
	int i;
	int err = 0;

	se_writel(se_dev, (count * 8), SE_SHA_MSG_LENGTH_REG_OFFSET);
	se_writel(se_dev, (count * 8), SE_SHA_MSG_LEFT_REG_OFFSET);
	for (i = 1; i < 4; i++) {
		se_writel(se_dev, 0, SE_SHA_MSG_LENGTH_REG_OFFSET + (4 * i));
		se_writel(se_dev, 0, SE_SHA_MSG_LEFT_REG_OFFSET + (4 * i));
	}

	if (se_dev->pclk && !se_dev->chipdata->const_freq) {
		err = clk_set_rate(se_dev->pclk, freq);
		if (err) {
			dev_err(se_dev->dev, "clock set_rate failed.\n");
			return;
		}
	}
	se_writel(se_dev, SHA_ENABLE, SE_SHA_CONFIG_REG_OFFSET);
}

static int tegra_se_start_operation(struct tegra_se_dev *se_dev, u32 nbytes,
				    bool context_save, bool diff_dst)
{
	u32 nblocks = nbytes / TEGRA_SE_AES_BLOCK_SIZE;
	int ret = 0;
	u32 val = 0, timeout = TEGRA_SE_TIMEOUT_1S;

	if ((tegra_get_chip_id() == TEGRA114) &&
	    (nblocks > SE_MAX_LAST_BLOCK_SIZE)) {
		dev_err(se_dev->dev, "nblocks out of range\n");
		return -EDOM;
	}

	/* clear any pending interrupts */
	val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
	se_writel(se_dev, val, SE_INT_STATUS_REG_OFFSET);

	se_writel(se_dev, se_dev->src_ll_buf_adr, SE_IN_LL_ADDR_REG_OFFSET);

	if (diff_dst)
		se_writel(se_dev, se_dev->dst_ll_buf_adr,
			  SE_OUT_LL_ADDR_REG_OFFSET);
	else
		se_writel(se_dev, se_dev->src_ll_buf_adr,
			  SE_OUT_LL_ADDR_REG_OFFSET);

	if (nblocks)
		se_writel(se_dev, nblocks - 1, SE_BLOCK_COUNT_REG_OFFSET);

	if (!se_dev->polling) {
		/* enable interupts */
		val = SE_INT_ERROR(INT_ENABLE) | SE_INT_OP_DONE(INT_ENABLE);
		se_writel(se_dev, val, SE_INT_ENABLE_REG_OFFSET);

		reinit_completion(&se_dev->complete);
	}

	if (context_save)
		se_writel(se_dev, SE_OPERATION(OP_CTX_SAVE),
			  SE_OPERATION_REG_OFFSET);
	else
		se_writel(se_dev, SE_OPERATION(OP_START),
			  SE_OPERATION_REG_OFFSET);

	if (se_dev->polling) {
		/* polling */
		val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
		while (!SE_OP_DONE(val, OP_DONE))
			val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
	} else {
		ret = wait_for_completion_timeout(&se_dev->complete,
						  msecs_to_jiffies(1000));
		if (ret == 0) {
			dev_err(se_dev->dev, "operation timed out no interrupt\n");
			return -ETIMEDOUT;
		}
	}

	if (se_dev->chipdata->ahb_ack) {
		/* Ensure data is out from SE using MEM_INTERFACE signal */
		val = se_readl(se_dev, SE_STATUS_REG_OFFSET);
		while (val & SE_STATUS_MEM_INTERFACE(MEM_INTERFACE_BUSY)) {
			if (!timeout) {
				dev_err(se_dev->dev, "mem operation timeout\n");
				return -ETIMEDOUT;
			}
			udelay(1);
			timeout--;
			val = se_readl(se_dev, SE_STATUS_REG_OFFSET);
		}

		timeout = TEGRA_SE_TIMEOUT_1S;
		while (tegra_ahb_is_mem_wrque_busy(se_dev->ahb_id)) {
			if (!timeout) {
				dev_err(se_dev->dev, "mem operation timeout\n");
				return -ETIMEDOUT;
			}
			udelay(1);
			timeout--;
		}
	}

	return 0;
}

static void tegra_se_read_hash_result(struct tegra_se_dev *se_dev,
				      u8 *pdata, u32 nbytes, bool swap32)
{
	u32 *result = (u32 *)pdata;
	u32 i;

	for (i = 0; i < nbytes / 4; i++) {
		result[i] = se_readl(se_dev, SE_HASH_RESULT_REG_OFFSET +
				(i * sizeof(u32)));
		if (swap32)
			result[i] = be32_to_cpu(result[i]);
	}
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
		se_ll->addr = (u32)sg_dma_address(sg);
		se_ll->data_len = min(sg->length, (size_t)total_loop);
		total_loop -= min(sg->length, (size_t)total_loop);
		sg = sg_next(sg);
		se_ll++;
	}

	return ret;
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
		nbytes -= min(sl->length, (size_t)nbytes);
		if (!nbytes)
			break;
		sl = sg_next(sl);
	}

	return sg_nents;
}

static int tegra_se_alloc_ll_buf(struct tegra_se_dev *se_dev,
				 u32 num_src_sgs, u32 num_dst_sgs)
{
	if (se_dev->src_ll_buf || se_dev->dst_ll_buf) {
		dev_err(se_dev->dev, "trying to allocate memory to allocated memory\n");
		return -EBUSY;
	}

	if (num_src_sgs) {
		se_dev->src_ll_size =
			(sizeof(struct tegra_se_ll) * num_src_sgs) +
			sizeof(u32);
		se_dev->src_ll_buf = dma_alloc_coherent(se_dev->dev,
							se_dev->src_ll_size,
							&se_dev->src_ll_buf_adr,
							GFP_KERNEL);
		if (!se_dev->src_ll_buf) {
			dev_err(se_dev->dev, "can not allocate src lldma buffer\n");
			return -ENOMEM;
		}
	}
	if (num_dst_sgs) {
		se_dev->dst_ll_size =
			(sizeof(struct tegra_se_ll) * num_dst_sgs) +
			sizeof(u32);
		se_dev->dst_ll_buf = dma_alloc_coherent(se_dev->dev,
							se_dev->dst_ll_size,
							&se_dev->dst_ll_buf_adr,
							GFP_KERNEL);
		if (!se_dev->dst_ll_buf) {
			dev_err(se_dev->dev, "can not allocate dst ll dma buffer\n");
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

static void tegra_se_get_sg_dma_buf(struct scatterlist *sg, u32 num_sgs,
				    u32 nbytes, u32 *sg_buf)
{
	struct sg_mapping_iter miter;
	unsigned int sg_flags = SG_MITER_ATOMIC | SG_MITER_FROM_SG;
	unsigned long flags;
	u32 *temp_buffer = sg_buf;
	unsigned int total = 0;

	sg_miter_start(&miter, sg, num_sgs, sg_flags);

	local_irq_save(flags);
	while (sg_miter_next(&miter) && total < nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(nbytes - total));
		memcpy(temp_buffer, miter.addr + total, len);
		temp_buffer += len;
		total += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);
}

static void tegra_se_get_dst_sg(struct scatterlist *sg, u32 num_sgs,
				u32 nbytes, u32 *sg_buf)
{
	struct sg_mapping_iter miter;
	unsigned int sg_flags = SG_MITER_ATOMIC | SG_MITER_FROM_SG;
	unsigned long flags;
	u32 *temp_buffer = sg_buf;
	unsigned int total = 0;

	sg_miter_start(&miter, sg, num_sgs, sg_flags);

	local_irq_save(flags);
	total = 0;
	while (sg_miter_next(&miter) && total < nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(nbytes - total));
		memcpy(miter.addr + total, temp_buffer, len);
		temp_buffer += len;
		total += len;
	}

	sg_miter_stop(&miter);
	local_irq_restore(flags);
}

static int tegra_se_setup_ablk_req(struct tegra_se_dev *se_dev,
				   struct ablkcipher_request *req)
{
	struct scatterlist *src_sg, *dst_sg;
	struct tegra_se_ll *src_ll, *dst_ll;
	u32 total, num_src_sgs, num_dst_sgs;
	int ret1 = 0, ret2 = 0;

	num_src_sgs = tegra_se_count_sgs(req->src, req->nbytes);
	num_dst_sgs = tegra_se_count_sgs(req->dst, req->nbytes);

	if ((num_src_sgs > SE_MAX_SRC_SG_COUNT) ||
	    (num_dst_sgs > SE_MAX_DST_SG_COUNT)) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EDOM;
	}

	*se_dev->src_ll_buf = num_src_sgs - 1;
	*se_dev->dst_ll_buf = num_dst_sgs - 1;

	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	src_sg = req->src;
	dst_sg = req->dst;
	total = req->nbytes;

	if (total) {
		if (req->nbytes == DISK_ENCR_BUF_SZ) {
			tegra_se_get_sg_dma_buf(src_sg, num_src_sgs, total,
						se_dev->sg_in_buf);
			src_ll->addr = se_dev->sg_in_buf_adr;
			src_ll->data_len = req->nbytes;

			dst_ll->addr = se_dev->sg_out_buf_adr;
			dst_ll->data_len = req->nbytes;
		} else {
			if (src_sg == dst_sg) {
				ret1 = tegra_map_sg(se_dev->dev, src_sg, 1,
						    DMA_BIDIRECTIONAL, src_ll,
						    total);
				if (!ret1)
					return -EINVAL;
			} else {
				ret1 = tegra_map_sg(se_dev->dev, src_sg, 1,
						    DMA_TO_DEVICE, src_ll,
						    total);
				ret2 = tegra_map_sg(se_dev->dev, dst_sg, 1,
						    DMA_FROM_DEVICE, dst_ll,
						    total);
				if (!ret1 || !ret2)
					return -EINVAL;
			}
		}

		WARN_ON(src_sg->length != dst_sg->length);
	}

	return 0;
}

static void tegra_se_dequeue_complete_req(struct tegra_se_dev *se_dev,
					  struct ablkcipher_request *req)
{
	struct scatterlist *src_sg, *dst_sg;
	u32 total;

	if (req) {
		src_sg = req->src;
		dst_sg = req->dst;
		total = req->nbytes;
		if (src_sg == dst_sg)
			tegra_unmap_sg(se_dev->dev, dst_sg, DMA_BIDIRECTIONAL,
				       total);
		else {
			tegra_unmap_sg(se_dev->dev, dst_sg, DMA_FROM_DEVICE,
				       total);
			tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE,
				       total);
		}
	}
}

static void tegra_se_process_new_req(struct crypto_async_request *async_req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	struct ablkcipher_request *req = ablkcipher_request_cast(async_req);
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);
	struct tegra_se_aes_context *aes_ctx =
		crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	int ret = 0;

	/* take access to the hw */
	mutex_lock(&se_hw_lock);

	/* write IV */
	if (req->info) {
		if (req_ctx->op_mode == SE_AES_OP_MODE_CTR) {
			tegra_se_write_seed(se_dev, (u32 *)req->info);
		} else {
			tegra_se_write_key_table(req->info,
						 TEGRA_SE_AES_IV_SIZE,
						 aes_ctx->slot->slot_num,
						 SE_KEY_TABLE_TYPE_UPDTDIV);
		}
	}

	ret = tegra_se_setup_ablk_req(se_dev, req);
	if (ret)
		goto out;

	tegra_se_config_algo(se_dev, req_ctx->op_mode,
			req_ctx->encrypt, aes_ctx->keylen);
	tegra_se_config_crypto(se_dev, req_ctx->op_mode,
		req_ctx->encrypt, aes_ctx->slot->slot_num,
		false);

	ret = tegra_se_start_operation(se_dev, req->nbytes, false,
				       ((req->src == req->dst) ? false : true));
	if (req->nbytes == DISK_ENCR_BUF_SZ)
		tegra_se_get_dst_sg(req->dst, 1, req->nbytes,
				    se_dev->sg_out_buf);
	else
		tegra_se_dequeue_complete_req(se_dev, req);
out:
	mutex_unlock(&se_hw_lock);
	req->base.complete(&req->base, ret);
}

static irqreturn_t tegra_se_irq(int irq, void *dev)
{
	struct tegra_se_dev *se_dev = dev;
	u32 val, err_stat;

	val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
	se_writel(se_dev, val, SE_INT_STATUS_REG_OFFSET);

	if (val & SE_INT_ERROR(INT_SET)) {
		err_stat = se_readl(se_dev, SE_ERR_STATUS_0);
		dev_err(se_dev->dev, "tegra_se_irq::error status is %x\n",
			err_stat);
	}

	if (val & SE_INT_OP_DONE(INT_SET))
		complete(&se_dev->complete);

	return IRQ_HANDLED;
}

static void tegra_se_work_handler(struct work_struct *work)
{
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;

	pm_runtime_get_sync(se_dev->dev);

	do {
		spin_lock_irq(&se_dev->lock);
		backlog = crypto_get_backlog(&se_dev->queue);
		async_req = crypto_dequeue_request(&se_dev->queue);
		if (!async_req)
			se_dev->work_q_busy = false;

		spin_unlock_irq(&se_dev->lock);

		if (backlog) {
			backlog->complete(backlog, -EINPROGRESS);
			backlog = NULL;
		}

		if (async_req) {
			tegra_se_process_new_req(async_req);
			async_req = NULL;
		}
	} while (se_dev->work_q_busy);
	pm_runtime_put(se_dev->dev);
}

static int tegra_se_aes_queue_req(struct ablkcipher_request *req)
{
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	unsigned long flags;
	bool idle = true;
	int err = 0;

	if (!tegra_se_count_sgs(req->src, req->nbytes)) {
		dev_err(se_dev->dev, "invalid SG count");
		return -EINVAL;
	}

	spin_lock_irqsave(&se_dev->lock, flags);
	err = ablkcipher_enqueue_request(&se_dev->queue, req);
	if (se_dev->work_q_busy)
		idle = false;
	spin_unlock_irqrestore(&se_dev->lock, flags);

	if (idle) {
		spin_lock_irq(&se_dev->lock);
		se_dev->work_q_busy = true;
		spin_unlock_irq(&se_dev->lock);
		queue_work(se_work_q, &se_work);
	}

	return err;
}

static int tegra_se_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;

	return tegra_se_aes_queue_req(req);
}

static int tegra_se_aes_setkey(struct crypto_ablkcipher *tfm,
			       const u8 *key, u32 keylen)
{
	struct tegra_se_aes_context *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_se_dev *se_dev = NULL;
	struct tegra_se_slot *pslot;
	u8 *pdata = (u8 *)key;

	if (!ctx || !ctx->se_dev) {
		pr_err("invalid context or dev");
		return -EINVAL;
	}

	se_dev = ctx->se_dev;

	if ((keylen != TEGRA_SE_KEY_128_SIZE) &&
	    (keylen != TEGRA_SE_KEY_192_SIZE) &&
	    (keylen != TEGRA_SE_KEY_256_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		return -EINVAL;
	}

	if (key) {
		if (!ctx->slot ||
		    (ctx->slot && ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				return -ENOMEM;
			}
			ctx->slot = pslot;
		}
		ctx->keylen = keylen;
	} else {
		tegra_se_free_key_slot(ctx->slot);
		ctx->slot = &ssk_slot;
		ctx->keylen = AES_KEYSIZE_128;
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	/* load the key */
	tegra_se_write_key_table(pdata, keylen, ctx->slot->slot_num,
				 SE_KEY_TABLE_TYPE_KEY);

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static int tegra_se_aes_cra_init(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_context *ctx = crypto_tfm_ctx(tfm);

	ctx->se_dev = sg_tegra_se_dev;
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
	struct tegra_se_dev *se_dev = se_devices[SE_DRBG];

	rng_ctx->se_dev = se_dev;
	rng_ctx->dt_buf = dma_alloc_coherent(se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
					     &rng_ctx->dt_buf_adr, GFP_KERNEL);
	if (!rng_ctx->dt_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		return -ENOMEM;
	}

	rng_ctx->rng_buf = dma_alloc_coherent(rng_ctx->se_dev->dev,
					      TEGRA_SE_RNG_DT_SIZE,
					      &rng_ctx->rng_buf_adr,
					      GFP_KERNEL);
	if (!rng_ctx->rng_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->dt_buf, rng_ctx->dt_buf_adr);
		return -ENOMEM;
	}

	return 0;
}

static int tegra_se_rng_drbg_get_random(struct crypto_rng *tfm,
					const u8 *src, unsigned int slen,
					u8 *rdata, unsigned int dlen)
{
	struct tegra_se_rng_context *rng_ctx = crypto_rng_ctx(tfm);
	struct tegra_se_dev *se_dev = rng_ctx->se_dev;
	struct tegra_se_ll *src_ll, *dst_ll;
	u8 *rdata_addr;
	int ret = 0, j;
	unsigned int num_blocks, data_len = 0;

	num_blocks = (dlen / TEGRA_SE_RNG_DT_SIZE);

	data_len = (dlen % TEGRA_SE_RNG_DT_SIZE);
	if (data_len == 0)
		num_blocks = num_blocks - 1;

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	src_ll->addr = rng_ctx->dt_buf_adr;
	src_ll->data_len = TEGRA_SE_RNG_DT_SIZE;
	dst_ll->addr = rng_ctx->rng_buf_adr;
	dst_ll->data_len = TEGRA_SE_RNG_DT_SIZE;

	tegra_se_config_algo(se_dev, SE_AES_OP_MODE_RNG_DRBG, true,
			     TEGRA_SE_KEY_128_SIZE);
	tegra_se_config_crypto(se_dev, SE_AES_OP_MODE_RNG_DRBG, true, 0, true);

	for (j = 0; j <= num_blocks; j++) {
		ret = tegra_se_start_operation(se_dev, TEGRA_SE_RNG_DT_SIZE,
					       false, true);
		if (ret)
			break;

		rdata_addr = (rdata + (j * TEGRA_SE_RNG_DT_SIZE));

		if (data_len && num_blocks == j) {
			memcpy(rdata_addr, rng_ctx->rng_buf, data_len);
		} else {
			memcpy(rdata_addr, rng_ctx->rng_buf,
			       TEGRA_SE_RNG_DT_SIZE);
		}
	}

	if (!ret)
		ret = dlen;

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

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

	if (rng_ctx->dt_buf) {
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->dt_buf, rng_ctx->dt_buf_adr);
	}

	if (rng_ctx->rng_buf) {
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
				  rng_ctx->rng_buf, rng_ctx->rng_buf_adr);
	}
	rng_ctx->se_dev = NULL;
}

static int tegra_se_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx;

	if (!tfm)
		return -EINVAL;
	sha_ctx = crypto_ahash_ctx(tfm);

	if (!sha_ctx)
		return -EINVAL;
	/* Initialize total bytes with zero */
	sha_ctx->total_count = 0;
	return 0;
}

static int tegra_se_shash_init(struct shash_desc *desc)
{
	return 0;
}

static int tegra_se_sha_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	struct scatterlist *src_sg;
	struct tegra_se_ll *src_ll;
	u32 total, num_sgs;
	unsigned long freq = 0;
	int err = 0;

	if (!req->nbytes)
		return 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case SHA1_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA1;
		freq = se_dev->chipdata->sha1_freq;
		break;

	case SHA224_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA224;
		freq = se_dev->chipdata->sha224_freq;
		break;

	case SHA256_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA256;
		freq = se_dev->chipdata->sha256_freq;
		break;

	case SHA384_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA384;
		freq = se_dev->chipdata->sha384_freq;
		break;

	case SHA512_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA512;
		freq = se_dev->chipdata->sha512_freq;
		break;

	default:
		dev_err(se_dev->dev, "Invalid SHA digest size\n");
		return -EINVAL;
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
	if (num_sgs > SE_MAX_SRC_SG_COUNT) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		err = -EDOM;
		goto fail;
	}

	*se_dev->src_ll_buf = num_sgs - 1;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	src_sg = req->src;
	total = req->nbytes;

	err = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
			   src_ll, total);
	if (!err) {
		err = -EINVAL;
		goto fail;
	}

	tegra_se_config_algo(se_dev, sha_ctx->op_mode, false, 0);
	tegra_se_config_sha(se_dev, req->nbytes, freq);
	err = tegra_se_start_operation(se_dev, 0, false, true);

	src_sg = req->src;
	total = req->nbytes;

	tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE, total);
	/* Increase total count with current req number of bytes */
	sha_ctx->total_count += req->nbytes;

fail:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return err;
}

static int tegra_se_sha_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_sha_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	int err = 0;
	u32 mode;
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

	if (req->nbytes) {
		err = tegra_se_sha_update(req);
		if (err)
			return err;
	} else
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

	/* If total length is zero return zero hash result */
	if (!sha_ctx->total_count) {
		/*
		 *  SW WAR for zero length SHA operation since
		 *  SE HW can't accept zero length SHA operation.
		 */
		mode = sha_ctx->op_mode - SE_AES_OP_MODE_SHA1;
		memcpy(req->result, zero_vec[mode].digest, zero_vec[mode].size);
		return 0;
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	/* read hash result */
	tegra_se_read_hash_result(se_dev, req->result,
				  crypto_ahash_digestsize(tfm), true);

	if ((sha_ctx->op_mode == SE_AES_OP_MODE_SHA384) ||
	    (sha_ctx->op_mode == SE_AES_OP_MODE_SHA512)) {
		u32 *result = (u32 *)req->result;
		u32 temp, i;

		for (i = 0; i < crypto_ahash_digestsize(tfm) / 4; i += 2) {
			temp = result[i];
			result[i] = result[i + 1];
			result[i + 1] = temp;
		}
	}
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static int tegra_se_shash_update(struct shash_desc *desc, const u8 *data,
				 unsigned int len)
{
	struct tegra_se_sha_context *sha_ctx = crypto_shash_ctx(desc->tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];
	struct tegra_se_ll *src_ll;
	u32 *temp_virt = NULL;
	unsigned long freq = 0;
	int err = 0;
	dma_addr_t tmp_buf_adr;	/* Destination buffer dma address */

	switch (crypto_shash_digestsize(desc->tfm)) {
	case SHA1_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA1;
		freq = se_dev->chipdata->sha1_freq;
		break;

	case SHA224_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA224;
		freq = se_dev->chipdata->sha224_freq;
		break;

	case SHA256_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA256;
		freq = se_dev->chipdata->sha256_freq;
		break;

	case SHA384_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA384;
		freq = se_dev->chipdata->sha384_freq;
		break;

	case SHA512_DIGEST_SIZE:
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA512;
		freq = se_dev->chipdata->sha512_freq;
		break;

	default:
		dev_err(se_dev->dev, "Invalid SHA digest size\n");
		return -EINVAL;
	}

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	temp_virt = dma_alloc_coherent(se_dev->dev, len,
				       &tmp_buf_adr, GFP_KERNEL);
	if (!temp_virt) {
		pr_err("\ndma_alloc_coherent failed\n");
		err = -EINVAL;
		goto exit;
	}

	dma_sync_single_for_device(se_dev->dev, tmp_buf_adr, len,
				   DMA_TO_DEVICE);

	/* copy client buffer to local dmaable buffer*/
	memcpy(temp_virt, data, len);

	/* Prepare linked list for HW */
	*se_dev->src_ll_buf =  0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	src_ll->addr = tmp_buf_adr;
	src_ll->data_len = len;

	tegra_se_config_algo(se_dev, sha_ctx->op_mode, false, 0);
	tegra_se_config_sha(se_dev, len, freq);
	err = tegra_se_start_operation(se_dev, 0, false, true);

	dma_free_coherent(se_dev->dev, len, temp_virt, tmp_buf_adr);
exit:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return err;
}

static int tegra_se_shash_final(struct shash_desc *desc, u8 *out)
{
	struct tegra_se_sha_context *sha_ctx = crypto_shash_ctx(desc->tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_SHA];

	switch (crypto_shash_digestsize(desc->tfm)) {
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

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	/* read hash result */
	tegra_se_read_hash_result(se_dev, out,
				  crypto_shash_digestsize(desc->tfm), true);

	if ((sha_ctx->op_mode == SE_AES_OP_MODE_SHA384) ||
	    (sha_ctx->op_mode == SE_AES_OP_MODE_SHA512)) {
		u32 *result = (u32 *)out;
		u32 temp, i;

		for (i = 0; i < crypto_shash_digestsize(desc->tfm) / 4;
		     i += 2) {
			temp = result[i];
			result[i] = result[i + 1];
			result[i + 1] = temp;
		}
	}
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static int tegra_se_sha_digest(struct ahash_request *req)
{
	return tegra_se_sha_init(req) ?: tegra_se_sha_final(req);
}

static int tegra_se_shash_digest(struct shash_desc *desc, const u8 *data,
				 unsigned int len, u8 *out)
{
	return tegra_se_shash_final(desc, out);
}

static int tegra_se_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_se_sha_context));
	return 0;
}

static void tegra_se_sha_cra_exit(struct crypto_tfm *tfm)
{
	/* do nothing */
}

static int tegra_se_shash_cra_init(struct crypto_tfm *tfm)
{
	unsigned int *hash = crypto_tfm_ctx(tfm);
	*hash = 0;
	return 0;
}

static void tegra_se_shash_cra_exit(struct crypto_tfm *tfm)
{
	/* do nothing */
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
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	struct scatterlist *src_sg;
	struct tegra_se_ll *src_ll;
	struct sg_mapping_iter miter;
	u32 num_sgs, blocks_to_process, last_block_bytes = 0, bytes_to_copy = 0;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	int ret = 0, i = 0;
	bool padding_needed = false;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC, total = 0;
	u8 *temp_buffer = NULL;
	bool use_orig_iv = true;

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	blocks_to_process = req->nbytes / TEGRA_SE_AES_BLOCK_SIZE;
	/* num of bytes less than block size */
	if ((req->nbytes % TEGRA_SE_AES_BLOCK_SIZE) || !blocks_to_process) {
		padding_needed = true;
		last_block_bytes = req->nbytes % TEGRA_SE_AES_BLOCK_SIZE;
	} else {
		/* decrement num of blocks */
		blocks_to_process--;
		if (blocks_to_process) {
			last_block_bytes = req->nbytes -
				(blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE);
		} else {
			last_block_bytes = req->nbytes;
		}
	}

	/* first process all blocks except last block */
	if (blocks_to_process) {
		num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
		if (num_sgs > SE_MAX_SRC_SG_COUNT) {
			dev_err(se_dev->dev, "num of SG buffers are more\n");
			ret = -EDOM;
			goto out;
		}
		*se_dev->src_ll_buf = num_sgs - 1;
		src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
		src_sg = req->src;
		total = blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE;

		ret = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
				   src_ll, total);
		if (!ret) {
			ret = -EINVAL;
			goto out;
		}
		tegra_se_config_algo(se_dev, SE_AES_OP_MODE_CMAC, true,
				     cmac_ctx->keylen);
		/* write zero IV */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);
		tegra_se_write_key_table(piv, TEGRA_SE_AES_IV_SIZE,
					 cmac_ctx->slot->slot_num,
					 SE_KEY_TABLE_TYPE_ORGIV);
		tegra_se_config_crypto(se_dev, SE_AES_OP_MODE_CMAC, true,
				       cmac_ctx->slot->slot_num, true);
		ret = tegra_se_start_operation(se_dev, blocks_to_process *
					       TEGRA_SE_AES_BLOCK_SIZE,
					       false, true);
		src_sg = req->src;
		tegra_unmap_sg(se_dev->dev, src_sg,  DMA_TO_DEVICE, total);

		if (ret)
			goto out;
		use_orig_iv = false;
	}

	/* get the last block bytes from the sg_dma buffer using miter */
	src_sg = req->src;
	num_sgs = tegra_se_count_sgs(req->src, req->nbytes);
	sg_flags |= SG_MITER_FROM_SG;
	cmac_ctx->buffer = dma_alloc_coherent(se_dev->dev,
					      TEGRA_SE_AES_BLOCK_SIZE,
					      &cmac_ctx->dma_addr, GFP_KERNEL);
	if (!cmac_ctx->buffer)
		goto out;

	local_irq_save(flags);
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	total = 0;
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
	*se_dev->src_ll_buf = 0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	src_ll->addr = cmac_ctx->dma_addr;
	src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	if (use_orig_iv) {
		/* use zero IV, this is when num of bytes is
		 * less <= block size
		 */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);
		tegra_se_write_key_table(piv, TEGRA_SE_AES_IV_SIZE,
					 cmac_ctx->slot->slot_num,
					 SE_KEY_TABLE_TYPE_ORGIV);
	}

	tegra_se_config_algo(se_dev, SE_AES_OP_MODE_CMAC, true,
			     cmac_ctx->keylen);
	tegra_se_config_crypto(se_dev, SE_AES_OP_MODE_CMAC, true,
			       cmac_ctx->slot->slot_num, use_orig_iv);
	ret = tegra_se_start_operation(se_dev, TEGRA_SE_AES_BLOCK_SIZE,
				       false, true);
	if (ret)
		goto out;

	tegra_se_read_hash_result(se_dev, req->result,
				  TEGRA_SE_AES_CMAC_DIGEST_SIZE, false);
out:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	if (cmac_ctx->buffer)
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
				  cmac_ctx->buffer, cmac_ctx->dma_addr);

	return ret;
}

static int tegra_se_aes_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
				    unsigned int keylen)
{
	struct tegra_se_aes_cmac_context *ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_AES];
	struct tegra_se_ll *src_ll, *dst_ll;
	struct tegra_se_slot *pslot;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	u32 *pbuf;
	dma_addr_t pbuf_adr;
	int ret = 0;
	u8 const rb = 0x87;
	u8 msb;

	if (!ctx) {
		dev_err(se_dev->dev, "invalid context");
		return -EINVAL;
	}

	if ((keylen != TEGRA_SE_KEY_128_SIZE) &&
	    (keylen != TEGRA_SE_KEY_192_SIZE) &&
	    (keylen != TEGRA_SE_KEY_256_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		return -EINVAL;
	}

	if (key) {
		if (!ctx->slot ||
		    (ctx->slot && ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				return -ENOMEM;
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
		tegra_se_free_key_slot(ctx->slot);
		return -ENOMEM;
	}
	memset(pbuf, 0, TEGRA_SE_AES_BLOCK_SIZE);

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	src_ll->addr = pbuf_adr;
	src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;
	dst_ll->addr = pbuf_adr;
	dst_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	/* load the key */
	tegra_se_write_key_table((u8 *)key, keylen, ctx->slot->slot_num,
				 SE_KEY_TABLE_TYPE_KEY);

	/* write zero IV */
	memset(piv, 0, TEGRA_SE_AES_IV_SIZE);

	/* load IV */
	tegra_se_write_key_table(piv, TEGRA_SE_AES_IV_SIZE, ctx->slot->slot_num,
				 SE_KEY_TABLE_TYPE_ORGIV);

	/* config crypto algo */
	tegra_se_config_algo(se_dev, SE_AES_OP_MODE_CBC, true, keylen);

	tegra_se_config_crypto(se_dev, SE_AES_OP_MODE_CBC, true,
			       ctx->slot->slot_num, true);

	ret = tegra_se_start_operation(se_dev, TEGRA_SE_AES_BLOCK_SIZE,
				       false, true);
	if (ret) {
		dev_err(se_dev->dev, "%s: start op failed\n", __func__);
		tegra_se_free_key_slot(ctx->slot);
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
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	if (pbuf)
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
				  pbuf, pbuf_adr);

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

	se_dev->rsa_slot_list = devm_kzalloc(se_dev->dev,
					     sizeof(struct tegra_se_rsa_slot) *
					     TEGRA_SE_RSA_KEYSLOT_COUNT,
					     GFP_KERNEL);
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
		pr_err("Invalid rsa context\n");
		return -EINVAL;
	}

	return ctx->mod_len;
}

static int tegra_se_rsa_setkey(struct crypto_akcipher *tfm, const void *key,
			       unsigned int keylen)
{
	struct tegra_se_aes_rsa_context *ctx = akcipher_tfm_ctx(tfm);
	struct tegra_se_dev *se_dev = se_devices[SE_RSA];
	u32 module_key_length, exponent_key_length;
	u32 pkt, val, key_size_words, key_word_size = 4;
	u32 *pkeydata = (u32 *)key;
	s32 i = 0;
	struct tegra_se_rsa_slot *pslot;
	unsigned long freq = 0;

	int err = 0;

	if (!ctx || !key) {
		dev_err(se_dev->dev, "Invalid context or key\n");
		return -EINVAL;
	}

	/* Allocate rsa key slot */
	if (!ctx->slot) {
		pslot = tegra_se_alloc_rsa_key_slot();
		if (!pslot) {
			dev_err(se_dev->dev, "no free key slot\n");
			return -ENOMEM;
		}
		ctx->slot = pslot;
	}

	module_key_length = (keylen >> 16);
	exponent_key_length = (keylen & (0xFFFF));

	if (!(((module_key_length / 64) >= 1) &&
	      ((module_key_length / 64) <= 4))) {
		tegra_se_rsa_free_key_slot(ctx->slot);
		dev_err(se_dev->dev, "Modulus length is not in range\n");
		return -EDOM;
	}

	ctx->mod_len = module_key_length;
	ctx->exp_len = exponent_key_length;

	freq = se_dev->chipdata->rsa_freq;

	if (se_dev->pclk && !se_dev->chipdata->const_freq) {
		err = clk_set_rate(se_dev->pclk, freq);
		if (err) {
			dev_err(se_dev->dev, "clock set_rate failed\n");
			tegra_se_rsa_free_key_slot(ctx->slot);
			return err;
		}
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if (exponent_key_length) {
		key_size_words = (exponent_key_length / key_word_size);

		/* Write exponent */
		if (se_dev->chipdata->rsa_key_rw_op) {
			for (i = (key_size_words - 1); i >= 0; i--) {
				se_writel(se_dev, *pkeydata++,
					  SE_RSA_KEYTABLE_DATA);
				pkt =
				RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
					RSA_KEY_PKT_WORD_ADDR(i);
				val = SE_RSA_KEY_OP(RSA_KEY_WRITE) |
					SE_RSA_KEYTABLE_PKT(pkt);
				se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
			}
		} else {
			for (i = (key_size_words - 1); i >= 0; i--) {
				pkt =
				RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
					RSA_KEY_PKT_WORD_ADDR(i);
				val = SE_RSA_KEYTABLE_PKT(pkt);
				se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
				se_writel(se_dev, *pkeydata++,
					  SE_RSA_KEYTABLE_DATA);
			}
		}
	}

	if (module_key_length) {
		key_size_words = (module_key_length / key_word_size);
		/* Write modulus */
		if (se_dev->chipdata->rsa_key_rw_op) {
			for (i = (key_size_words - 1); i >= 0; i--) {
				se_writel(se_dev, *pkeydata++,
					  SE_RSA_KEYTABLE_DATA);
				pkt =
				RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
					RSA_KEY_PKT_WORD_ADDR(i);
				val = SE_RSA_KEY_OP(RSA_KEY_WRITE) |
					SE_RSA_KEYTABLE_PKT(pkt);
				se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
			}
		} else {
			for (i = (key_size_words - 1); i >= 0; i--) {
				pkt =
				RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
					RSA_KEY_PKT_WORD_ADDR(i);
				val = SE_RSA_KEYTABLE_PKT(pkt);
				se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
				se_writel(se_dev, *pkeydata++,
					  SE_RSA_KEYTABLE_DATA);
			}
		}
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static int tegra_se_rsa_op(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = NULL;
	struct tegra_se_aes_rsa_context *rsa_ctx = NULL;
	struct tegra_se_dev *se_dev = se_devices[SE_RSA];
	struct tegra_se_ll *src_ll, *dst_ll;
	u32 num_src_sgs, num_dst_sgs;
	int ret1 = 0, ret2 = 0;
	u32 val = 0;

	if (!req) {
		dev_err(se_dev->dev, "Invalid rsa request\n");
		return -EINVAL;
	}

	tfm = crypto_akcipher_reqtfm(req);
	if (!tfm) {
		dev_err(se_dev->dev, "Invalid rsa transform\n");
		return -EINVAL;
	}

	rsa_ctx = akcipher_tfm_ctx(tfm);

	if (!rsa_ctx || !rsa_ctx->slot) {
		dev_err(se_dev->dev, "Invalid rsa context\n");
		return -EINVAL;
	}

	if ((req->src_len < TEGRA_SE_RSA512_INPUT_SIZE) ||
	    (req->src_len > TEGRA_SE_RSA2048_INPUT_SIZE)) {
		dev_err(se_dev->dev, "rsa request src length not in range\n");
		return -EDOM;
	}

	if ((req->dst_len < TEGRA_SE_RSA512_INPUT_SIZE) ||
	    (req->dst_len > TEGRA_SE_RSA2048_INPUT_SIZE)) {
		dev_err(se_dev->dev, "rsa request dst length not in range\n");
		return -EDOM;
	}

	if (req->src_len != rsa_ctx->mod_len) {
		dev_err(se_dev->dev, "Invalid rsa request src length\n");
		return -EINVAL;
	}

	num_src_sgs = tegra_se_count_sgs(req->src, req->src_len);
	num_dst_sgs = tegra_se_count_sgs(req->dst, req->dst_len);
	if ((num_src_sgs > SE_MAX_SRC_SG_COUNT) ||
	    (num_dst_sgs > SE_MAX_DST_SG_COUNT)) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EDOM;
	}

	mutex_lock(&se_hw_lock);
	*se_dev->src_ll_buf = num_src_sgs - 1;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);

	if (req->src == req->dst) {
		dst_ll = src_ll;
		ret1 = tegra_map_sg(se_dev->dev, req->src, 1, DMA_BIDIRECTIONAL,
				    src_ll, req->src_len);
		if (!ret1) {
			mutex_unlock(&se_hw_lock);
			return -EINVAL;
		}
	} else {
		*se_dev->dst_ll_buf = num_dst_sgs - 1;
		dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
		ret1 = tegra_map_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE,
				    src_ll, req->src_len);
		ret2 = tegra_map_sg(se_dev->dev, req->dst, 1, DMA_FROM_DEVICE,
				    dst_ll, req->dst_len);
		if (!ret1 || !ret2) {
			mutex_unlock(&se_hw_lock);
			return -EINVAL;
		}
	}

	pm_runtime_get_sync(se_dev->dev);
	/* Write key length */
	se_writel(se_dev, ((rsa_ctx->mod_len / 64) - 1),
		  SE_RSA_KEY_SIZE_REG_OFFSET);

	/* Write exponent size in 32 bytes */
	se_writel(se_dev, (rsa_ctx->exp_len / 4),
		  SE_RSA_EXP_SIZE_REG_OFFSET);

	val = SE_CONFIG_ENC_ALG(ALG_RSA) |
		SE_CONFIG_DEC_ALG(ALG_NOP) |
		SE_CONFIG_DST(DST_MEMORY);
	se_writel(se_dev, val, SE_CONFIG_REG_OFFSET);
	se_writel(se_dev, RSA_KEY_SLOT(rsa_ctx->slot->slot_num), SE_RSA_CONFIG);
	se_writel(se_dev, SE_CRYPTO_INPUT_SEL(INPUT_AHB), SE_CRYPTO_REG_OFFSET);

	ret1 = tegra_se_start_operation(se_dev, 256, false, true);
	if (ret1)
		dev_err(se_dev->dev, "%s start op failed\n", __func__);

	if (req->src == req->dst) {
		tegra_unmap_sg(se_dev->dev, req->src, DMA_BIDIRECTIONAL,
			       req->src_len);
	} else {
		tegra_unmap_sg(se_dev->dev, req->src, DMA_TO_DEVICE,
			       req->src_len);
		tegra_unmap_sg(se_dev->dev, req->dst, DMA_FROM_DEVICE,
			       req->dst_len);
	}

	pm_runtime_put_sync(se_dev->dev);
	mutex_unlock(&se_hw_lock);

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
	u32 *pkeydata;
	int i, err = 0;
	struct tegra_se_rsa_slot *pslot;
	unsigned long freq = 0;

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
		dev_err(se_dev->dev, "DH Modulus length not in rane\n");
		return -EDOM;
	}

	freq = se_dev->chipdata->rsa_freq;

	if (se_dev->pclk && !se_dev->chipdata->const_freq) {
		err = clk_set_rate(se_dev->pclk, freq);
		if (err) {
			dev_err(se_dev->dev, "clock set_rate failed.\n");
			tegra_se_rsa_free_key_slot(ctx->slot);
			return err;
		}
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if (exponent_key_length) {
		key_size_words = (exponent_key_length / key_word_size);
		/* Write exponent */
		for (i = (key_size_words - 1); i >= 0; i--) {
			pkt = RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
					RSA_KEY_PKT_WORD_ADDR(i);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
			se_writel(se_dev, be32_to_cpu(*pkeydata++),
				  SE_RSA_KEYTABLE_DATA);
		}
	}

	if (module_key_length) {
		pkeydata = (u32 *)ctx->p;
		key_size_words = (module_key_length / key_word_size);
		/* Write modulus */
		for (i = (key_size_words - 1); i >= 0; i--) {
			pkt = RSA_KEY_INPUT_MODE(RSA_KEY_INPUT_MODE_REG) |
					RSA_KEY_NUM(ctx->slot->slot_num) |
					RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
					RSA_KEY_PKT_WORD_ADDR(i);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			se_writel(se_dev, val, SE_RSA_KEYTABLE_ADDR);
			se_writel(se_dev, be32_to_cpu(*pkeydata++),
				  SE_RSA_KEYTABLE_DATA);
		}
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static void tegra_se_fix_endianness(struct tegra_se_dev *se_dev,
				    struct scatterlist *sg, u32 num_sgs,
				    u32 nbytes, bool be)
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
	int err, j;
	u32 val, total, zpad_sz;

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

	*se_dev->src_ll_buf = num_src_sgs - 1;
	*se_dev->dst_ll_buf = num_dst_sgs - 1;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	err = tegra_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE,
			   src_ll, total);
	if (!err) {
		err = -EINVAL;
		goto free;
	}
	err = tegra_map_sg(se_dev->dev, req->dst, 1, DMA_FROM_DEVICE,
			   dst_ll, req->dst_len);
	if (!err) {
		err = -EINVAL;
		goto unmap;
	}

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	/* Write key length */
	se_writel(se_dev, ((dh_ctx->p_size / 64) - 1),
		  SE_RSA_KEY_SIZE_REG_OFFSET);

	/* Write exponent size in 32 bytes */
	se_writel(se_dev, (dh_ctx->key_size / 4),
		  SE_RSA_EXP_SIZE_REG_OFFSET);

	val = SE_CONFIG_ENC_ALG(ALG_RSA) |
		SE_CONFIG_DEC_ALG(ALG_NOP) |
		SE_CONFIG_DST(DST_MEMORY);
	se_writel(se_dev, val, SE_CONFIG_REG_OFFSET);
	se_writel(se_dev, RSA_KEY_SLOT(dh_ctx->slot->slot_num), SE_RSA_CONFIG);
	se_writel(se_dev, SE_CRYPTO_INPUT_SEL(INPUT_AHB), SE_CRYPTO_REG_OFFSET);

	err = tegra_se_start_operation(se_dev, 256, false, true);
	if (err) {
		dev_err(se_dev->dev,
			"tegra_se_aes_rsa_digest:: start op failed\n");
		goto exit;
	}

	tegra_se_fix_endianness(se_dev, req->dst, num_dst_sgs,
				req->dst_len, false);
exit:
	pm_runtime_put_sync(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	tegra_unmap_sg(se_dev->dev, req->dst, DMA_FROM_DEVICE, req->dst_len);
unmap:
	tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE, total);
free:
	if (!req->src)
		devm_kfree(se_dev->dev, base_buff);

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static int tegra_se_dh_set_secret(struct crypto_kpp *tfm, const void *buf,
				  unsigned int len)
#else
static int tegra_se_dh_set_secret(struct crypto_kpp *tfm, void *buf,
				  unsigned int len)
#endif
{
	int ret = 0;

	struct tegra_se_dh_context *ctx = tegra_se_dh_get_ctx(tfm);
	struct dh params;

	ctx->se_dev = sg_tegra_se_dev;

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
		.cra_priority = 300,
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
	},
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

static struct shash_alg shash_algs[] = {
{
		.init = tegra_se_shash_init,
		.update = tegra_se_shash_update,
		.final = tegra_se_shash_final,
		.digest = tegra_se_shash_digest,
		.digestsize = SHA1_DIGEST_SIZE,
		.statesize = SHA1_STATE_SIZE,
		.base = {
			.cra_name = "sha1",
			.cra_driver_name = "tegra-se-sha1-shash",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_SHASH,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_shash_cra_init,
			.cra_exit = tegra_se_shash_cra_exit,
		}
	}, {
		.init = tegra_se_shash_init,
		.update = tegra_se_shash_update,
		.final = tegra_se_shash_final,
		.digest = tegra_se_shash_digest,
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize = SHA224_STATE_SIZE,
		.base = {
			.cra_name = "sha224",
			.cra_driver_name = "tegra-se-sha224-shash",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_SHASH,
			.cra_blocksize = SHA224_DIGEST_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_shash_cra_init,
			.cra_exit = tegra_se_shash_cra_exit,
		}
	}, {
		.init = tegra_se_shash_init,
		.update = tegra_se_shash_update,
		.final = tegra_se_shash_final,
		.digest = tegra_se_shash_digest,
		.digestsize = SHA256_DIGEST_SIZE,
		.statesize = SHA256_STATE_SIZE,
		.base = {
			.cra_name = "sha256",
			.cra_driver_name = "tegra-se-sha256-shash",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_SHASH,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_shash_cra_init,
			.cra_exit = tegra_se_shash_cra_exit,
		}
	}, {
		.init = tegra_se_shash_init,
		.update = tegra_se_shash_update,
		.final = tegra_se_shash_final,
		.digest = tegra_se_shash_digest,
		.digestsize = SHA384_DIGEST_SIZE,
		.statesize = SHA384_STATE_SIZE,
		.base = {
			.cra_name = "sha384",
			.cra_driver_name = "tegra-se-sha384-shash",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_SHASH,
			.cra_blocksize = SHA384_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_shash_cra_init,
			.cra_exit = tegra_se_shash_cra_exit,
		}
	}, {
		.init = tegra_se_shash_init,
		.update = tegra_se_shash_update,
		.final = tegra_se_shash_final,
		.digest = tegra_se_shash_digest,
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = SHA512_STATE_SIZE,
		.base = {
			.cra_name = "sha512",
			.cra_driver_name = "tegra-se-sha512-shash",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_SHASH,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_shash_cra_init,
			.cra_exit = tegra_se_shash_cra_exit,
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

static bool is_algo_supported(struct device_node *node, char *algo)
{
	if (of_property_match_string(node, "supported-algos", algo) >= 0)
		return true;
	else
		return false;
}

static bool is_algo_supported_in_hw(struct tegra_se_dev *se_dev,
	const char *algo)
{
	if (!strcmp(algo, "ansi_cprng")) {
		if (se_dev->chipdata->cprng_supported)
			return true;
		else
			return false;
	}

	if (!strcmp(algo, "drbg")) {
		if (se_dev->chipdata->drbg_supported)
			return true;
		else
			return false;
	}

	if (!strcmp(algo, "rsa-pka0") || !strcmp(algo, "rsa")) {
		if (se_dev->chipdata->rsa_supported) {
			if (tegra_chip_get_revision() == TEGRA210_REVISION_A01)
				return false;
			else
				return true;
		} else {
			return false;
		}
	}

	return true;
}

static struct tegra_se_chipdata tegra_se_chipdata = {
	.rsa_supported = false,
	.cprng_supported = true,
	.drbg_supported = false,
	.aes_freq = 300000000,
	.rng_freq = 300000000,
	.sha1_freq = 300000000,
	.sha224_freq = 300000000,
	.sha256_freq = 300000000,
	.sha384_freq = 300000000,
	.sha512_freq = 300000000,
	.mccif_supported = false,
	.rsa_key_rw_op = true,
	.aes_keydata_reg_sz = 128,
	.ahb_ack = false,
	.handle_sc7 = true,
};

static struct tegra_se_chipdata tegra11_se_chipdata = {
	.rsa_supported = true,
	.cprng_supported = false,
	.drbg_supported = true,
	.const_freq = false,
	.aes_freq = 150000000,
	.rng_freq = 150000000,
	.sha1_freq = 200000000,
	.sha224_freq = 250000000,
	.sha256_freq = 250000000,
	.sha384_freq = 150000000,
	.sha512_freq = 150000000,
	.rsa_freq = 350000000,
	.mccif_supported = false,
	.rsa_key_rw_op = true,
	.aes_keydata_reg_sz = 128,
	.ahb_ack = false,
	.handle_sc7 = true,
};

static struct tegra_se_chipdata tegra21_se_chipdata = {
	.rsa_supported = true,
	.cprng_supported = false,
	.drbg_supported = true,
	.const_freq = true,
	.aes_freq = 510000000,
	.rng_freq = 510000000,
	.sha1_freq = 510000000,
	.sha224_freq = 510000000,
	.sha256_freq = 510000000,
	.sha384_freq = 510000000,
	.sha512_freq = 510000000,
	.rsa_freq = 510000000,
	.mccif_supported = true,
	.rsa_key_rw_op = false,
	.aes_keydata_reg_sz = 32,
	.ahb_ack = false,
	.handle_sc7 = false,
};

static struct tegra_se_chipdata tegra210b01_se_chipdata = {
	.rsa_supported = true,
	.cprng_supported = false,
	.drbg_supported = true,
	.const_freq = true,
	.aes_freq = 510000000,
	.rng_freq = 510000000,
	.sha1_freq = 510000000,
	.sha224_freq = 510000000,
	.sha256_freq = 510000000,
	.sha384_freq = 510000000,
	.sha512_freq = 510000000,
	.rsa_freq = 510000000,
	.mccif_supported = true,
	.rsa_key_rw_op = false,
	.aes_keydata_reg_sz = 32,
	.ahb_ack = true,
	.handle_sc7 = false,
};

static const struct of_device_id tegra_se_of_match[] = {
	{
		.compatible = "nvidia,tegra124-se",
		.data = &tegra11_se_chipdata,
	},
	{
		.compatible = "nvidia,tegra210-se",
		.data = &tegra21_se_chipdata,
	},
	{
		.compatible = "nvidia,tegra210b01-se",
		.data = &tegra210b01_se_chipdata,
	}, {
	}
};
MODULE_DEVICE_TABLE(of, tegra_se_of_match);

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
	struct resource *res = NULL;
	const struct of_device_id *match;
	struct device_node *node;
	int err = 0, i = 0, j = 0, k = 0, val;
	const char *rsa_name;

	se_dev = devm_kzalloc(&pdev->dev, sizeof(*se_dev), GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		match = of_match_device(of_match_ptr(tegra_se_of_match),
					&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Error: No device match found\n");
			return -ENODEV;
		}
		se_dev->chipdata = (struct tegra_se_chipdata *)match->data;
	} else {
		se_dev->chipdata =
			(struct tegra_se_chipdata *)pdev->id_entry->driver_data;
	}

	if (se_dev->chipdata->ahb_ack) {
		val = tegra_ahb_get_master_id(&pdev->dev);
		if (val < 0) {
			dev_err(&pdev->dev, "Error: AHB master id not found\n");
			return -EINVAL;
		}
		se_dev->ahb_id = val;
	}

	spin_lock_init(&se_dev->lock);
	crypto_init_queue(&se_dev->queue, TEGRA_SE_CRYPTO_QUEUE_LENGTH);
	platform_set_drvdata(pdev, se_dev);
	se_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	se_dev->io_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(se_dev->io_reg)) {
		dev_err(se_dev->dev, "ioremap failed\n");
		return PTR_ERR(se_dev->io_reg);
	}

	se_dev->irq = platform_get_irq(pdev, 0);
	if (se_dev->irq < 0) {
		dev_err(se_dev->dev, "platform_get_irq failed\n");
		return -ENXIO;
	}

	/* Initialize the clock */
	se_dev->pclk = devm_clk_get(&pdev->dev, "se");
	if (IS_ERR(se_dev->pclk)) {
		dev_err(se_dev->dev, "clock intialization failed (%ld)\n",
			PTR_ERR(se_dev->pclk));
		return PTR_ERR(se_dev->pclk);
	}

	if (se_dev->chipdata->const_freq)
		err = clk_set_rate(se_dev->pclk, 510000000);
	else
		err = clk_set_rate(se_dev->pclk, ULONG_MAX);
	if (err) {
		dev_err(se_dev->dev, "clock set_rate failed.\n");
		return err;
	}

	node = of_node_get(se_dev->dev->of_node);
	if (is_algo_supported(node, "aes")) {
		err = tegra_init_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_key_slot failed\n");
			return err;
		}

		err = tegra_init_rsa_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_rsa_key_slot failed\n");
			return err;
		}

		se_work_q = alloc_workqueue("se_work_q",
					WQ_HIGHPRI | WQ_UNBOUND, 16);
		if (!se_work_q) {
			dev_err(se_dev->dev, "alloc_workqueue failed\n");
			return -ENOMEM;
		}
	}

	init_completion(&se_dev->complete);

	sg_tegra_se_dev = se_dev;
	tegra_pd_add_device(se_dev->dev);
	pm_runtime_enable(se_dev->dev);
	tegra_se_fill_se_dev_info(se_dev);
	tegra_se_key_read_disable_all();

	err = devm_request_irq(&pdev->dev, se_dev->irq, tegra_se_irq, 0,
			       DRIVER_NAME, se_dev);
	if (err) {
		dev_err(se_dev->dev, "request_irq failed - irq[%d] err[%d]\n",
			se_dev->irq, err);
		goto fail;
	}

	se_dev->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	err = tegra_se_alloc_ll_buf(se_dev, SE_MAX_SRC_SG_COUNT,
				    SE_MAX_DST_SG_COUNT);
	if (err) {
		dev_err(se_dev->dev, "can not allocate ll dma buffer\n");
		goto fail;
	}

	if (is_algo_supported(node, "drbg") &&
		(is_algo_supported_in_hw(se_dev, rng_algs[0].base.cra_name))) {
		INIT_LIST_HEAD(&rng_algs[0].base.cra_list);
		err = crypto_register_rng(&rng_algs[0]);
		if (err) {
			dev_err(se_dev->dev, "crypto_register_rng failed\n");
			goto fail_rng;
		}
	}

	if (is_algo_supported(node, "aes")) {
		for (i = 0; i < ARRAY_SIZE(aes_algs); i++) {
			INIT_LIST_HEAD(&aes_algs[i].cra_list);
			err = crypto_register_alg(&aes_algs[i]);
			if (err) {
				dev_err(se_dev->dev,
					"crypto_register_alg failed for %s\n",
					aes_algs[i].cra_name);
				goto fail_alg;
			}
		}
		err = crypto_register_ahash(&hash_algs[0]);
		if (err) {
			dev_err(se_dev->dev,
			"crypto_register_sha alg failed index[%d] err: %d\n",
			j, err);
			goto fail_alg1;
		}
	}

	if (is_algo_supported(node, "sha")) {
		for (j = 0; j < ARRAY_SIZE(shash_algs); j++) {
			err = crypto_register_shash(&shash_algs[j]);
			if (err) {
				dev_err(se_dev->dev,
					"crypto_register_shash failed for %s\n",
					shash_algs[j].base.cra_name);
				goto fail_shash;
			}
		}

		for (j = 1; j < ARRAY_SIZE(hash_algs); j++) {
			err = crypto_register_ahash(&hash_algs[j]);
			if (err) {
				dev_err(se_dev->dev,
					"crypto_register_ahash failed for %s\n",
					hash_algs[j].halg.base.cra_name);
				goto fail_ahash;
			}
		}
	}

	if (is_algo_supported(node, "rsa")) {

		err = of_property_read_u32(node, "pka0-rsa-priority", &val);
		if (!err)
			rsa_alg.base.cra_priority = val;

		err = of_property_read_string(node, "pka0-rsa-name", &rsa_name);
		if (!err)
			strcpy(rsa_alg.base.cra_name, rsa_name);

		if (is_algo_supported_in_hw(se_dev, rsa_alg.base.cra_name)) {
			err = crypto_register_akcipher(&rsa_alg);
			if (err) {
				dev_err(se_dev->dev,
				"crypto_register_akcipher failed %d\n", err);
				goto fail_akcipher;
			}
			err = crypto_register_kpp(&dh_algs[0]);
			if (err) {
				dev_err(se_dev->dev, "crypto_register_kpp failed %d\n",
					err);
				goto fail_kpp;
			}

			se_dev->dh_buf1 = (u32 *)devm_kzalloc(se_dev->dev,
						TEGRA_SE_RSA2048_INPUT_SIZE,
						GFP_KERNEL);
			se_dev->dh_buf2 = (u32 *)devm_kzalloc(se_dev->dev,
							TEGRA_SE_RSA2048_INPUT_SIZE,
							GFP_KERNEL);
			if (!se_dev->dh_buf1 || !se_dev->dh_buf2)
				goto fail_kpp;
		}
	}

	se_dev->sg_in_buf = dmam_alloc_coherent(
				&pdev->dev, DISK_ENCR_BUF_SZ,
				&se_dev->sg_in_buf_adr, GFP_KERNEL);

	se_dev->sg_out_buf = dmam_alloc_coherent(
			&pdev->dev, DISK_ENCR_BUF_SZ, &se_dev->sg_out_buf_adr,
			GFP_KERNEL);

#if defined(CONFIG_PM)
	if (!se_dev->chipdata->drbg_supported)
		se_dev->ctx_save_buf = dma_alloc_coherent(
			se_dev->dev, SE_CONTEXT_BUFER_SIZE,
			&se_dev->ctx_save_buf_adr, GFP_KERNEL | GFP_DMA);
	else
		se_dev->ctx_save_buf = dma_alloc_coherent(
			se_dev->dev, SE_CONTEXT_DRBG_BUFER_SIZE,
			&se_dev->ctx_save_buf_adr, GFP_KERNEL | GFP_DMA);

	if (!se_dev->ctx_save_buf) {
		dev_err(se_dev->dev, "Context save buffer alloc filed\n");
		err = -ENOMEM;
		goto fail_ctx_buf;
	}
#endif

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if (se_dev->chipdata->drbg_supported &&
	    (tegra_get_chip_id() != TEGRA114)) {
		se_writel(se_dev,
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC(DRBG_RO_ENT_SRC_ENABLE) |
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC_LOCK(
						DRBG_RO_ENT_SRC_LOCK_ENABLE),
			  SE_RNG_SRC_CONFIG_REG_OFFSET);
		se_dev->chipdata->drbg_src_entropy_clk_enable = true;
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	if (se_dev->chipdata->drbg_src_entropy_clk_enable) {
		/* Initialize the entropy clock */
		se_dev->enclk = devm_clk_get(&pdev->dev, "entropy");
		if (IS_ERR(se_dev->enclk)) {
			err = PTR_ERR(se_dev->enclk);
			dev_err(se_dev->dev,
				"entropy clock init failed(%d)\n", err);
			goto fail_clk_get;
		}
	}

	dev_info(se_dev->dev, "%s: complete", __func__);
	return 0;

fail_clk_get:
#if defined(CONFIG_PM)
	dma_free_coherent(se_dev->dev, !se_dev->chipdata->drbg_supported ?
			  SE_CONTEXT_BUFER_SIZE : SE_CONTEXT_DRBG_BUFER_SIZE,
			  se_dev->ctx_save_buf, se_dev->ctx_save_buf_adr);
fail_ctx_buf:
#endif
	if (is_algo_supported(node, "rsa")) {
		if (is_algo_supported_in_hw(se_dev, rsa_alg.base.cra_name))
			crypto_unregister_kpp(&dh_algs[0]);
	}
fail_kpp:
	if (is_algo_supported(node, "rsa")) {
		if (is_algo_supported_in_hw(se_dev, rsa_alg.base.cra_name))
			crypto_unregister_akcipher(&rsa_alg);
	}
fail_akcipher:
	if (is_algo_supported(node, "sha")) {
		for (k = 1; k < j; k++)
			crypto_unregister_ahash(&hash_algs[k]);
	}
fail_ahash:
	if (is_algo_supported(node, "sha")) {
		for (k = 0; k < j; k++)
			crypto_unregister_shash(&shash_algs[k]);
	}
fail_shash:
	if (is_algo_supported(node, "aes"))
		crypto_unregister_ahash(&hash_algs[0]);
fail_alg1:
	if (is_algo_supported(node, "aes")) {
		for (k = 0; k < i; k++)
			crypto_unregister_alg(&aes_algs[k]);
	}
fail_alg:
	if (is_algo_supported(node, "drbg")) {
		if (is_algo_supported_in_hw(se_dev, rng_algs[0].base.cra_name))
			crypto_unregister_rng(&rng_algs[0]);
	}
fail_rng:
	tegra_se_free_ll_buf(se_dev);
fail:
	pm_runtime_disable(se_dev->dev);
	sg_tegra_se_dev = NULL;
	destroy_workqueue(se_work_q);

	return err;
}

static int tegra_se_remove(struct platform_device *pdev)
{
	struct tegra_se_dev *se_dev = platform_get_drvdata(pdev);
	struct device_node *node;
	int i;

	if (!se_dev) {
		pr_err("Device is NULL\n");
		return -ENODEV;
	}
	node = of_node_get(se_dev->dev->of_node);

	pm_runtime_disable(se_dev->dev);

	cancel_work_sync(&se_work);
	if (se_work_q)
		destroy_workqueue(se_work_q);

#if defined(CONFIG_PM)
	dma_free_coherent(se_dev->dev, !se_dev->chipdata->drbg_supported ?
			  SE_CONTEXT_BUFER_SIZE : SE_CONTEXT_DRBG_BUFER_SIZE,
			  se_dev->ctx_save_buf, se_dev->ctx_save_buf_adr);
#endif
	if (is_algo_supported(node, "rsa")) {
		if (is_algo_supported_in_hw(se_dev, rsa_alg.base.cra_name)) {
			crypto_unregister_akcipher(&rsa_alg);
			crypto_unregister_kpp(&dh_algs[0]);
		}
	}

	if (is_algo_supported(node, "sha")) {
		for (i = 1; i < ARRAY_SIZE(hash_algs); i++)
			crypto_unregister_ahash(&hash_algs[i]);

		for (i = 0; i < ARRAY_SIZE(shash_algs); i++)
			crypto_unregister_shash(&shash_algs[i]);
	}

	if (is_algo_supported(node, "aes")) {
		for (i = 0; i < ARRAY_SIZE(aes_algs); i++)
			crypto_unregister_alg(&aes_algs[i]);
		crypto_unregister_ahash(&hash_algs[0]);
	}

	if (is_algo_supported(node, "aes")) {
		if (is_algo_supported_in_hw(se_dev, rng_algs[0].base.cra_name))
			crypto_unregister_rng(&rng_algs[0]);
	}
	tegra_se_free_ll_buf(se_dev);
	sg_tegra_se_dev = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int tegra_se_generate_rng_key(struct tegra_se_dev *se_dev)
{
	int ret = 0;
	u32 val = 0;

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;

	/* Configure algorithm */
	val = SE_CONFIG_ENC_ALG(ALG_RNG) | SE_CONFIG_ENC_MODE(MODE_KEY128) |
		SE_CONFIG_DST(DST_KEYTAB);
	se_writel(se_dev, val, SE_CONFIG_REG_OFFSET);

	/* Configure destination key index number */
	val = SE_CRYPTO_KEYTABLE_DST_KEY_INDEX(srk_slot.slot_num) |
		SE_CRYPTO_KEYTABLE_DST_WORD_QUAD(KEYS_0_3);
	se_writel(se_dev, val, SE_CRYPTO_KEYTABLE_DST_REG_OFFSET);

	/* Configure crypto */
	val = SE_CRYPTO_INPUT_SEL(INPUT_RANDOM) |
		SE_CRYPTO_XOR_POS(XOR_BYPASS) |
		SE_CRYPTO_CORE_SEL(CORE_ENCRYPT) |
		SE_CRYPTO_HASH(HASH_DISABLE) |
		SE_CRYPTO_KEY_INDEX(ssk_slot.slot_num) |
		SE_CRYPTO_IV_SEL(IV_ORIGINAL);
	se_writel(se_dev, val, SE_CRYPTO_REG_OFFSET);

	ret = tegra_se_start_operation(se_dev, TEGRA_SE_KEY_128_SIZE,
				       false, true);

	return ret;
}

static int tegra_se_generate_srk(struct tegra_se_dev *se_dev)
{
	int ret = 0;
	u32 val = 0;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if (se_dev->chipdata->drbg_src_entropy_clk_enable) {
		/* enable clock for entropy */
		ret = clk_prepare_enable(se_dev->enclk);
		if (ret) {
			dev_err(se_dev->dev, "entropy clock enable failed\n");
			goto out;
		}
	}

	ret = tegra_se_generate_rng_key(se_dev);
	if (ret)
		goto fail;

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;

	val = SE_CONFIG_ENC_ALG(ALG_RNG) | SE_CONFIG_ENC_MODE(MODE_KEY128) |
		SE_CONFIG_DEC_ALG(ALG_NOP) | SE_CONFIG_DST(DST_SRK);

	se_writel(se_dev, val, SE_CONFIG_REG_OFFSET);

	if (!se_dev->chipdata->drbg_supported)
		val = SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT) |
				SE_CRYPTO_HASH(HASH_DISABLE) |
				SE_CRYPTO_KEY_INDEX(srk_slot.slot_num) |
				SE_CRYPTO_IV_SEL(IV_UPDATED);
	else
		val = SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT) |
				SE_CRYPTO_HASH(HASH_DISABLE) |
				SE_CRYPTO_KEY_INDEX(srk_slot.slot_num) |
				SE_CRYPTO_IV_SEL(IV_UPDATED) |
				SE_CRYPTO_INPUT_SEL(INPUT_RANDOM);
	se_writel(se_dev, val, SE_CRYPTO_REG_OFFSET);

	if (se_dev->chipdata->drbg_supported) {
		se_writel(se_dev, SE_RNG_CONFIG_MODE(DRBG_MODE_FORCE_RESEED) |
			  SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
			  SE_RNG_CONFIG_REG_OFFSET);
		se_writel(se_dev, RNG_RESEED_INTERVAL,
			  SE_RNG_RESEED_INTERVAL_REG_OFFSET);
	}
	ret = tegra_se_start_operation(se_dev, TEGRA_SE_KEY_128_SIZE,
				       false, true);
fail:
	if (se_dev->chipdata->drbg_src_entropy_clk_enable)
		clk_disable_unprepare(se_dev->enclk);
out:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_generate_random_data(struct tegra_se_dev *se_dev)
{
	struct tegra_se_ll *src_ll, *dst_ll;
	int ret = 0;
	u32 val;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if (se_dev->chipdata->drbg_src_entropy_clk_enable) {
		/* enable clock for entropy */
		ret = clk_prepare_enable(se_dev->enclk);
		if (ret) {
			dev_err(se_dev->dev, "entropy clock enable failed\n");
			goto out;
		}
	}

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	src_ll->addr = se_dev->ctx_save_buf_adr;
	src_ll->data_len = SE_CONTEXT_SAVE_RANDOM_DATA_SIZE;
	dst_ll->addr = se_dev->ctx_save_buf_adr;
	dst_ll->data_len = SE_CONTEXT_SAVE_RANDOM_DATA_SIZE;

	tegra_se_config_algo(se_dev, SE_AES_OP_MODE_RNG_X931, true,
			     TEGRA_SE_KEY_128_SIZE);

	/* Configure crypto */
	val = SE_CRYPTO_INPUT_SEL(INPUT_RANDOM) |
		SE_CRYPTO_XOR_POS(XOR_BYPASS) |
		SE_CRYPTO_CORE_SEL(CORE_ENCRYPT) |
		SE_CRYPTO_HASH(HASH_DISABLE) |
		SE_CRYPTO_KEY_INDEX(srk_slot.slot_num) |
		SE_CRYPTO_IV_SEL(IV_ORIGINAL);

	se_writel(se_dev, val, SE_CRYPTO_REG_OFFSET);
	if (se_dev->chipdata->drbg_supported)
		se_writel(se_dev, SE_RNG_CONFIG_MODE(DRBG_MODE_FORCE_RESEED) |
			  SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
			  SE_RNG_CONFIG_REG_OFFSET);

	ret = tegra_se_start_operation(
			se_dev, SE_CONTEXT_SAVE_RANDOM_DATA_SIZE, false, true);

	if (se_dev->chipdata->drbg_src_entropy_clk_enable)
		clk_disable_unprepare(se_dev->enclk);
out:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_encrypt_context_data(struct tegra_se_dev *se_dev,
					    u32 context_offset, u32 data_size)
{
	struct tegra_se_ll *src_ll, *dst_ll;
	int ret = 0;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	src_ll->addr = se_dev->ctx_save_buf_adr + context_offset;
	src_ll->data_len = data_size;
	dst_ll->addr = se_dev->ctx_save_buf_adr + context_offset;
	dst_ll->data_len = data_size;

	se_writel(se_dev, SE_CONTEXT_SAVE_SRC(MEM),
		  SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);

	ret = tegra_se_start_operation(se_dev, data_size, true, true);

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_sticky_bits_context_save(struct tegra_se_dev *se_dev)
{
	struct tegra_se_ll *dst_ll;
	int ret = 0, i = 0;
	u32 val = 0;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	dst_ll->addr = se_dev->ctx_save_buf_adr +
		SE_CONTEXT_SAVE_STICKY_BITS_OFFSET;
	dst_ll->data_len = SE_CONTEXT_SAVE_STICKY_BITS_SIZE;

	if (!se_dev->chipdata->drbg_supported) {
		se_writel(se_dev, SE_CONTEXT_SAVE_SRC(STICKY_BITS),
			  SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
		ret = tegra_se_start_operation(
			se_dev, SE_CONTEXT_SAVE_STICKY_BITS_SIZE, true, true);
	} else {
		for (i = 0; i < 2; i++) {
			val = SE_CONTEXT_SAVE_SRC(STICKY_BITS) |
				SE_CONTEXT_SAVE_STICKY_WORD_QUAD(i);
			se_writel(se_dev, val,
				  SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
			ret = tegra_se_start_operation(
				se_dev, SE_CONTEXT_SAVE_STICKY_BITS_SIZE,
				true, true);
			if (ret)
				break;
			dst_ll->addr += SE_CONTEXT_SAVE_STICKY_BITS_SIZE;
		}
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_keytable_context_save(struct tegra_se_dev *se_dev)
{
	struct tegra_se_ll *dst_ll;
	int ret = 0, i, j;
	u32 val = 0;

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->dst_ll_buf = 0;
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	if (!se_dev->chipdata->drbg_supported)
		dst_ll->addr = se_dev->ctx_save_buf_adr +
				SE_CONTEXT_SAVE_KEYS_OFFSET;
	else
		dst_ll->addr = se_dev->ctx_save_buf_adr +
				SE11_CONTEXT_SAVE_KEYS_OFFSET;

	dst_ll->data_len = TEGRA_SE_KEY_128_SIZE;

	for (i = 0; i < TEGRA_SE_KEYSLOT_COUNT; i++) {
		for (j = 0; j < 2; j++) {
			val = SE_CONTEXT_SAVE_SRC(KEYTABLE) |
				SE_CONTEXT_SAVE_KEY_INDEX(i) |
				SE_CONTEXT_SAVE_WORD_QUAD(j);
			se_writel(se_dev,
				  val, SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
			ret = tegra_se_start_operation(
				se_dev, TEGRA_SE_KEY_128_SIZE, true, true);
			if (ret)
				break;
			dst_ll->addr += TEGRA_SE_KEY_128_SIZE;
		}
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_rsakeytable_context_save(struct tegra_se_dev *se_dev)
{
	struct tegra_se_ll *dst_ll;
	int ret = 0, word_quad, k, slot;
	u32 val = 0, index;

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->dst_ll_buf = 0;
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	dst_ll->addr = se_dev->ctx_save_buf_adr +
				SE_CONTEXT_SAVE_RSA_KEYS_OFFSET;
	dst_ll->data_len = TEGRA_SE_KEY_128_SIZE;

	for (slot = 0; slot < TEGRA_SE_RSA_CONTEXT_SAVE_KEYSLOT_COUNT; slot++) {
		/* First the modulus and then the exponent must be
		 * encrypted and saved. This is repeated for SLOT 0
		 * and SLOT 1. Hence the order:
		 * SLOT 0 modulus : RSA_KEY_INDEX : 1
		 * SLOT 0 exponent : RSA_KEY_INDEX : 0
		 * SLOT 1 modulus : RSA_KEY_INDEX : 3
		 * SLOT 1 exponent : RSA_KEY_INDEX : 2
		 */
		if (slot == 0)
			index = 1;
		else
			index = 3;

		/* loop for modulus and exponent */
		for (k = 0; k < 2; k++, index--) {
			for (word_quad = 0; word_quad < 16; word_quad++) {
				val = SE_CONTEXT_SAVE_SRC(RSA_KEYTABLE) |
					SE_CONTEXT_SAVE_RSA_KEY_INDEX(index) |
					SE_CONTEXT_RSA_WORD_QUAD(word_quad);
				se_writel(se_dev, val,
					  SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
				ret = tegra_se_start_operation(
					se_dev, TEGRA_SE_KEY_128_SIZE,
					true, true);
				if (ret) {
					dev_err(se_dev->dev,
						"rsa key context save error\n");
					break;
				}
				dst_ll->addr += TEGRA_SE_KEY_128_SIZE;
			}
		}
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_lp_iv_context_save(struct tegra_se_dev *se_dev,
				       bool org_iv, u32 context_offset)
{
	struct tegra_se_ll *dst_ll;
	int ret = 0, i;
	u32 val = 0;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	*se_dev->dst_ll_buf = 0;
	dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);
	dst_ll->addr = se_dev->ctx_save_buf_adr + context_offset;
	dst_ll->data_len = TEGRA_SE_AES_IV_SIZE;

	for (i = 0; i < TEGRA_SE_KEYSLOT_COUNT; i++) {
		val = SE_CONTEXT_SAVE_SRC(KEYTABLE) |
			SE_CONTEXT_SAVE_KEY_INDEX(i) |
			(org_iv ? SE_CONTEXT_SAVE_WORD_QUAD(ORIG_IV) :
			SE_CONTEXT_SAVE_WORD_QUAD(UPD_IV));
		se_writel(se_dev, val, SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
		ret = tegra_se_start_operation(se_dev, TEGRA_SE_AES_IV_SIZE,
					       true, true);
		if (ret)
			break;
		dst_ll->addr += TEGRA_SE_AES_IV_SIZE;
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int tegra_se_save_SRK(struct tegra_se_dev *se_dev)
{
	int ret, val;

	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	se_writel(se_dev, SE_CONTEXT_SAVE_SRC(SRK),
		  SE_CONTEXT_SAVE_CONFIG_REG_OFFSET);
	ret = tegra_se_start_operation(se_dev, 0, true, true);

	if (ret < 0) {
		dev_err(se_dev->dev, "\n LP SRK operation failed\n");
		goto out;
	}

	if ((tegra_get_chip_id() == TEGRA114) &&
	    se_dev->chipdata->drbg_supported) {
		/* clear any pending interrupts */
		val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
		se_writel(se_dev, val, SE_INT_STATUS_REG_OFFSET);

		if (!se_dev->polling) {
			/* enable interupts */
			val = SE_INT_ERROR(INT_ENABLE) |
				SE_INT_OP_DONE(INT_ENABLE);
			se_writel(se_dev, val, SE_INT_ENABLE_REG_OFFSET);

			reinit_completion(&se_dev->complete);
		}

		val = SE_CONFIG_ENC_ALG(ALG_NOP) |
			SE_CONFIG_DEC_ALG(ALG_NOP);
		se_writel(se_dev, val, SE_CRYPTO_REG_OFFSET);

		se_writel(se_dev, SE_OPERATION(OP_CTX_SAVE),
			  SE_OPERATION_REG_OFFSET);

		if (se_dev->polling) {
			/*polling*/
			val = se_readl(se_dev, SE_INT_STATUS_REG_OFFSET);
			while (!SE_OP_DONE(val, OP_DONE))
				val = se_readl(se_dev,
					       SE_INT_STATUS_REG_OFFSET);
		} else {
			ret = wait_for_completion_timeout(
				&se_dev->complete, msecs_to_jiffies(1000));
			if (ret == 0) {
				dev_err(se_dev->dev,
					"\n LP SRK timed out no interrupt\n");
				ret = -ETIMEDOUT;
			}
		}
	}
out:
	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return ret;
}

static int se_suspend(struct device *dev, bool polling)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_se_dev *se_dev = platform_get_drvdata(pdev);
	int err = 0, i;
	unsigned char *dt_buf = NULL;
	u8 pdata[SE_CONTEXT_KNOWN_PATTERN_SIZE] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

	if (!se_dev) {
		pr_err("Device is NULL\n");
		return -ENODEV;
	}

	save_se_device = dev;

	if (!se_dev->chipdata->handle_sc7) {
		/* SC7 is handled in Secure OS drivers */
		return 0;
	}

	se_dev->polling = polling;

	/* Generate SRK */
	err = tegra_se_generate_srk(se_dev);
	if (err) {
		dev_err(se_dev->dev, "\n LP SRK genration failed\n");
		goto out;
	}

	/* Generate random data*/
	err = tegra_se_lp_generate_random_data(se_dev);
	if (err) {
		dev_err(se_dev->dev, "\n LP random pattern generation failed\n");
		goto out;
	}

	/* Encrypt random data */
	err = tegra_se_lp_encrypt_context_data(
			se_dev, SE_CONTEXT_SAVE_RANDOM_DATA_OFFSET,
			SE_CONTEXT_SAVE_RANDOM_DATA_SIZE);
	if (err) {
		dev_err(se_dev->dev, "\n LP random pattern encryption failed\n");
		goto out;
	}

	/* Sticky bits context save*/
	err = tegra_se_lp_sticky_bits_context_save(se_dev);
	if (err) {
		dev_err(se_dev->dev, "\n LP sticky bits context save failure\n");
		goto out;
	}

	/* Key table context save*/
	err = tegra_se_lp_keytable_context_save(se_dev);
	if (err) {
		dev_err(se_dev->dev, "\n LP key table  save failure\n");
		goto out;
	}

	/* Original iv context save*/
	err = tegra_se_lp_iv_context_save(se_dev, true,
					  (se_dev->chipdata->drbg_supported ?
					  SE11_CONTEXT_ORIGINAL_IV_OFFSET :
					  SE_CONTEXT_ORIGINAL_IV_OFFSET));
	if (err) {
		dev_err(se_dev->dev, "\n LP original iv save failure\n");
		goto out;
	}

	/* Updated iv context save*/
	err = tegra_se_lp_iv_context_save(se_dev, false,
					  (se_dev->chipdata->drbg_supported ?
					  SE11_CONTEXT_UPDATED_IV_OFFSET :
					  SE_CONTEXT_UPDATED_IV_OFFSET));
	if (err) {
		dev_err(se_dev->dev, "\n LP updated iv save failure\n");
		goto out;
	}

	if (se_dev->chipdata->drbg_supported) {
		/* rsa-key slot table context save*/
		err = tegra_se_lp_rsakeytable_context_save(se_dev);
		if (err) {
			dev_err(se_dev->dev, "\n LP RSA key table save failure\n");
			goto out;
		}
		/* Encrypt known pattern */
		dt_buf = (unsigned char *)se_dev->ctx_save_buf;
		dt_buf += SE_CONTEXT_SAVE_RSA_KNOWN_PATTERN_OFFSET;
		for (i = 0; i < SE_CONTEXT_KNOWN_PATTERN_SIZE; i++)
			dt_buf[i] = pdata[i];
		err = tegra_se_lp_encrypt_context_data(
			se_dev, SE_CONTEXT_SAVE_RSA_KNOWN_PATTERN_OFFSET,
			SE_CONTEXT_KNOWN_PATTERN_SIZE);
	} else {
		/* Encrypt known pattern */
		dt_buf = (unsigned char *)se_dev->ctx_save_buf;
		dt_buf += SE_CONTEXT_SAVE_KNOWN_PATTERN_OFFSET;
		for (i = 0; i < SE_CONTEXT_KNOWN_PATTERN_SIZE; i++)
			dt_buf[i] = pdata[i];
		err = tegra_se_lp_encrypt_context_data(
			se_dev, SE_CONTEXT_SAVE_KNOWN_PATTERN_OFFSET,
			SE_CONTEXT_KNOWN_PATTERN_SIZE);
	}
	if (err) {
		dev_err(se_dev->dev, "LP known pattern save failure\n");
		goto out;
	}

	/* Write lp context buffer address into PMC scratch register */
	err = tegra_pmc_save_se_context_buffer_address(
			page_to_phys(vmalloc_to_page(se_dev->ctx_save_buf)));
	if (err != 0) {
		dev_info(se_dev->dev, "failed to save SE context buffer address\n");
		goto out;
	}

	/* Saves SRK in secure scratch */
	err = tegra_se_save_SRK(se_dev);
	if (err < 0) {
		dev_err(se_dev->dev, "LP SRK save failure\n");
		goto out;
	}

out:
	/* put the device into runtime suspend state - disable clock */
	pm_runtime_put_sync(dev);

	return err;
}
EXPORT_SYMBOL(se_suspend);

struct device *get_se_device(void)
{
	return save_se_device;
}
EXPORT_SYMBOL(get_se_device);

static int tegra_se_suspend(struct device *dev)
{
	int ret = 0;

	ret = se_suspend(dev, false);
	return ret;
}

static int tegra_se_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_se_dev *se_dev = platform_get_drvdata(pdev);

	if (!se_dev->chipdata->handle_sc7) {
		/* SC7 is handled in Secure OS drivers */
		return 0;
	}

	/* pair with tegra_se_suspend, no need to actually enable clock */
	pm_runtime_get_noresume(dev);

	/* take access to the hw */
	mutex_lock(&se_hw_lock);
	pm_runtime_get_sync(se_dev->dev);

	if ((tegra_get_chip_id() != TEGRA30) &&
	    (tegra_get_chip_id() != TEGRA114)) {
		se_writel(se_dev,
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC(DRBG_RO_ENT_SRC_ENABLE) |
			  SE_RNG_SRC_CONFIG_RO_ENT_SRC_LOCK(
						DRBG_RO_ENT_SRC_LOCK_ENABLE),
			  SE_RNG_SRC_CONFIG_REG_OFFSET);
		se_dev->chipdata->drbg_src_entropy_clk_enable = true;
	}

	pm_runtime_put(se_dev->dev);
	mutex_unlock(&se_hw_lock);

	return 0;
}

static int tegra_se_runtime_suspend(struct device *dev)
{
	/*
	 * do a dummy read, to avoid scenarios where you have unposted writes
	 * still on the bus, before disabling clocks
	 */
	se_readl(sg_tegra_se_dev, SE_CONFIG_REG_OFFSET);

	clk_disable_unprepare(sg_tegra_se_dev->pclk);

	return 0;
}

static int tegra_se_runtime_resume(struct device *dev)
{
	clk_prepare_enable(sg_tegra_se_dev->pclk);

	return 0;
}

static const struct dev_pm_ops tegra_se_dev_pm_ops = {
	.runtime_suspend = tegra_se_runtime_suspend,
	.runtime_resume = tegra_se_runtime_resume,
	.suspend = tegra_se_suspend,
	.resume = tegra_se_resume,
};
#endif /* CONFIG_PM */

static struct platform_device_id tegra_dev_se_devtype[] = {
	{
		.name = "tegra-se",
		.driver_data = (unsigned long)&tegra_se_chipdata,
	},
	{
		.name = "tegra11-se",
		.driver_data = (unsigned long)&tegra11_se_chipdata,
	},
	{
		.name = "tegra12-se",
		.driver_data = (unsigned long)&tegra11_se_chipdata,
	},
	{
		.name = "tegra21-se",
		.driver_data = (unsigned long)&tegra21_se_chipdata,
	},
	{
	}
};

static struct platform_driver tegra_se_driver = {
	.probe  = tegra_se_probe,
	.remove = tegra_se_remove,
	.id_table = tegra_dev_se_devtype,
	.driver = {
		.name   = "tegra-se",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_of_match),
#if defined(CONFIG_PM)
		.pm = &tegra_se_dev_pm_ops,
#endif
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

MODULE_DESCRIPTION("Tegra Crypto algorithm support");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se");
