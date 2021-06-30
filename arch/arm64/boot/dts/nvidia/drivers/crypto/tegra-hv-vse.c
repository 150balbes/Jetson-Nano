/*
 * Cryptographic API.
 * drivers/crypto/tegra-hv-vse.c
 *
 * Support for Tegra Virtual Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2016-2019, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/delay.h>
#include <linux/tegra-ivc.h>
#include <linux/iommu.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>

#define TEGRA_HV_VSE_SHA_MAX_LL_NUM 26
#define TEGRA_HV_VSE_SHA_MAX_LL_NUM_1 24
#define TEGRA_HV_VSE_AES_MAX_LL_NUM 17
#define TEGRA_HV_VSE_CRYPTO_QUEUE_LENGTH 100
#define TEGRA_VIRTUAL_SE_KEY_128_SIZE 16
#define TEGRA_HV_VSE_AES_CMAC_MAX_LL_NUM 36
#define TEGRA_HV_VSE_MAX_TASKS_PER_SUBMIT 64
#define TEGRA_HV_VSE_IVC_REQ_FRAME_SIZE 320
#define TEGRA_HV_VSE_IVC_FRAME_SIZE 0x5040
#define TEGRA_HV_VSE_TIMEOUT (msecs_to_jiffies(10000))
#define TEGRA_HV_VSE_NUM_SERVER_REQ 4
#define TEGRA_VIRTUAL_SE_RNG1_SIZE 300
#define TEGRA_HV_VSE_SHA_MAX_BLOCK_SIZE 128

static struct task_struct *tegra_vse_task;
static bool vse_thread_start;
static bool is_rng1_registered;

/* Security Engine Linked List */
struct tegra_virtual_se_ll {
	dma_addr_t addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

struct tegra_vse_tag {
	unsigned int *priv_data;
	u8 reserved[12];
};

/* Tegra Virtual Security Engine commands */
enum tegra_virtual_se_command {
	VIRTUAL_SE_AES_CRYPTO,
	VIRTUAL_SE_KEY_SLOT,
	VIRTUAL_SE_PROCESS,
	VIRTUAL_RNG1_PROCESS,
};

/* RNG1 response */
struct tegra_vse_rng1_data {
	u8 status;
	u32 bytes_returned;
	u8 data[TEGRA_VIRTUAL_SE_RNG1_SIZE];
};

struct tegra_vse_priv_data {
	struct ablkcipher_request *reqs[TEGRA_HV_VSE_MAX_TASKS_PER_SUBMIT];
	struct tegra_virtual_se_dev *se_dev;
	struct completion alg_complete;
	int req_cnt;
	void (*call_back_vse)(void *);
	int cmd;
	int slot_num;
	int gather_buf_sz;
	struct scatterlist sg;
	void *buf;
	dma_addr_t buf_addr;
	struct tegra_vse_rng1_data rng1;
	int rx_status;
};

struct tegra_virtual_se_dev {
	struct device *dev;
	u32 stream_id;
	/* lock for Crypto queue access*/
	spinlock_t lock;
	/* Security Engine crypto queue */
	struct crypto_queue queue;
	/* Work queue busy status */
	bool work_q_busy;
	struct work_struct se_work;
	struct workqueue_struct *vse_work_q;
	struct mutex mtx;
	int req_cnt;
	struct ablkcipher_request *reqs[TEGRA_HV_VSE_MAX_TASKS_PER_SUBMIT];
	atomic_t ivc_count;
	int gather_buf_sz;
	/* Engine id */
	unsigned int engine_id;
	/* Engine suspend state */
	atomic_t se_suspended;
	/* Mutex lock for SE server */
	struct mutex server_lock;
	/* Disable a keyslot label as a key */
	bool disable_keyslot_label;
};

struct tegra_virtual_se_addr {
	u32 lo;
	u32 hi;
};

struct tegra_virtual_se_linklist {
	u8 number;
	struct tegra_virtual_se_addr addr[TEGRA_HV_VSE_SHA_MAX_LL_NUM];
};

struct tegra_virtual_se_linklist_1 {
	u8 number;
	struct tegra_virtual_se_addr addr[TEGRA_HV_VSE_SHA_MAX_LL_NUM_1];
};

struct tegra_virtual_se_aes_linklist {
	u8 number;
	struct tegra_virtual_se_addr addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
};

union tegra_virtual_se_aes_args {
	struct keyiv {
		u8 slot;
		u8 length;
		u8 type;
		u8 data[32];
		u8 oiv[16];
		u8 uiv[16];
	} key;
	struct aes_encdec {
		u8 streamid;
		u8 keyslot;
		u8 key_length;
		u8 mode;
		u8 ivsel;
		u8 lctr[16];
		u8 ctr_cntn;
		u32 data_length;
		u8 src_ll_num;
		struct tegra_virtual_se_addr
			src_addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
		u8 dst_ll_num;
		struct tegra_virtual_se_addr
			dst_addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
	} op;
	struct aes_cmac {
		u8 streamid;
		u8 keyslot;
		u8 key_length;
		u8 ivsel;
		u32 data_length;
		u64 dst;
		struct tegra_virtual_se_aes_linklist src;
	} op_cmac;
	struct aes_rng {
		u8 streamid;
		u32 data_length;
		u64 dst;
	} op_rng;
};

union tegra_virtual_se_rsa_args {
	struct tegra_virtual_key {
		u8 slot;
		u32 length;
		u8 data[256];
	} key;
	struct encdec {
		u64 p_src;
		u64 p_dst;
		u32 mod_length;
		u32 exp_length;
		u8 streamid;
		u8 keyslot;
	} op;
};

struct tegra_virtual_se_rng1_args {
	u32 length;
};

union tegra_virtual_se_sha_args {
	struct hash {
		u32 msg_block_length;
		u32 msg_total_length;
		u32 msg_left_length;
		u8 mode;
		u8 streamid;
		u32 hash[16];
		u64 dst;
		struct tegra_virtual_se_linklist src;
	} op_hash;
	struct hash_1 {
		u32 msg_block_length;
		u32 msg_total_length0;
		u32 msg_total_length1;
		u32 msg_total_length2;
		u32 msg_total_length3;
		u32 msg_left_length0;
		u32 msg_left_length1;
		u32 msg_left_length2;
		u32 msg_left_length3;
		u8 mode;
		u8 streamid;
		u32 hash[16];
		u64 dst;
		struct tegra_virtual_se_linklist_1 src;
	} op_hash1;
};

struct tegra_virtual_se_ivc_resp_msg_t {
	u8 engine;
	u8 tag;
	u8 status;
	u8 keyslot;
	u8 reserved[TEGRA_HV_VSE_IVC_REQ_FRAME_SIZE - 4];
};

struct tegra_virtual_se_ivc_rng {
	u8 engine;
	u8 tag;
	u8 status;
	u8 data_len[4];
	u8 data[TEGRA_VIRTUAL_SE_RNG1_SIZE];
};

struct tegra_virtual_se_ivc_tx_msg_t {
	u8 engine;
	u8 tag;
	u8 cmd;
	union {
		union tegra_virtual_se_aes_args aes;
		union tegra_virtual_se_sha_args sha;
		union tegra_virtual_se_rsa_args rsa;
		struct tegra_virtual_se_rng1_args rng1;
	} args;
};

struct tegra_virtual_se_ivc_hdr_t {
	u32 num_reqs;
	u8 tag[0x10];
	u32 status;
	u8 reserved[0x40 - 0x18];
};

struct tegra_virtual_se_ivc_msg_t {
	struct tegra_virtual_se_ivc_hdr_t hdr;
	union {
		struct tegra_virtual_se_ivc_tx_msg_t tx;
		struct tegra_virtual_se_ivc_resp_msg_t rx;
	} d[TEGRA_HV_VSE_MAX_TASKS_PER_SUBMIT];
};

/* Security Engine SHA context */
struct tegra_virtual_se_sha_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* SHA operation mode */
	u32 op_mode;
	unsigned int digest_size;
	u8 mode;
};

struct sha_zero_length_vector {
	unsigned int size;
	char *digest;
};

/* Tegra Virtual Security Engine operation modes */
enum tegra_virtual_se_op_mode {
	/* Secure Hash Algorithm-1 (SHA1) mode */
	VIRTUAL_SE_OP_MODE_SHA1,
	/* Secure Hash Algorithm-224  (SHA224) mode */
	VIRTUAL_SE_OP_MODE_SHA224 = 4,
	/* Secure Hash Algorithm-256  (SHA256) mode */
	VIRTUAL_SE_OP_MODE_SHA256,
	/* Secure Hash Algorithm-384  (SHA384) mode */
	VIRTUAL_SE_OP_MODE_SHA384,
	/* Secure Hash Algorithm-512  (SHA512) mode */
	VIRTUAL_SE_OP_MODE_SHA512,
};

/* Security Engine AES context */
struct tegra_virtual_se_aes_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	struct ablkcipher_request *req;
	/* Security Engine key slot */
	u32 aes_keyslot;
	/* key length in bytes */
	u32 keylen;
	/* AES operation mode */
	u32 op_mode;
	/* Is key slot */
	bool is_key_slot_allocated;
	/* Whether key is a keyslot label */
	bool is_keyslot_label;
};

enum tegra_virtual_se_aes_op_mode {
	AES_CBC,
	AES_ECB,
	AES_CTR,
	AES_OFB,
};

/* Security Engine request context */
struct tegra_virtual_se_aes_req_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* Security Engine operation mode */
	enum tegra_virtual_se_aes_op_mode op_mode;
	/* Operation type */
	bool encrypt;
	/* Engine id */
	u8 engine_id;
};

/* Security Engine request context */
struct tegra_virtual_se_req_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	unsigned int digest_size;
	u8 mode;			/* SHA operation mode */
	u8 *sha_buf;			/* Buffer to store residual data */
	dma_addr_t sha_buf_addr;	/* DMA address to residual data */
	u8 *hash_result;		/* Intermediate hash result */
	dma_addr_t hash_result_addr;	/* Intermediate hash result dma addr */
	u64 total_count;		/* Total bytes in all the requests */
	u32 residual_bytes;		/* Residual byte count */
	u32 blk_size;			/* SHA block size */
	bool is_first;			/* Represents first block */
	bool req_context_initialized;	/* Mark initialization status */
	bool force_align;		/* Enforce buffer alignment */
};

/* Security Engine AES CMAC context */
struct tegra_virtual_se_aes_cmac_context {
	u32 aes_keyslot;
	/* key length in bits */
	u32 keylen;
	/* Key1 */
	u8 K1[TEGRA_VIRTUAL_SE_KEY_128_SIZE];
	/* Key2 */
	u8 K2[TEGRA_VIRTUAL_SE_KEY_128_SIZE];
	bool is_key_slot_allocated;
};

/* Security Engine random number generator context */
struct tegra_virtual_se_rng_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* RNG buffer pointer */
	u32 *rng_buf;
	/* RNG buffer dma address */
	dma_addr_t rng_buf_adr;
};

/* Security Engine rng1 trng context */
struct tegra_virtual_se_rng1_trng_ctx {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
};

enum se_engine_id {
	VIRTUAL_SE_AES0,
	VIRTUAL_SE_AES1,
	VIRTUAL_SE_RSA,
	VIRTUAL_SE_SHA,
	VIRTUAL_SE_RNG1,
	VIRTUAL_MAX_SE_ENGINE_NUM,
};

#define VIRTUAL_SE_CMD_SHA_HASH 1
#define VIRTUAL_SE_CMD_SHA_HASH_1   2

enum tegra_virual_se_aes_iv_type {
	AES_ORIGINAL_IV,
	AES_UPDATED_IV,
	AES_IV_REG,
};

#define VIRTUAL_SE_AES_BLOCK_SIZE		16
#define VIRTUAL_SE_AES_MIN_KEY_SIZE		16
#define VIRTUAL_SE_AES_MAX_KEY_SIZE		32
#define VIRTUAL_SE_AES_IV_SIZE			16
#define VIRTUAL_SE_CMD_AES_ALLOC_KEY		1
#define VIRTUAL_SE_CMD_AES_RELEASE_KEY		2
#define VIRTUAL_SE_CMD_AES_SET_KEY		3
#define VIRTUAL_SE_CMD_AES_ENCRYPT		4
#define VIRTUAL_SE_CMD_AES_DECRYPT		5
#define VIRTUAL_SE_CMD_AES_CMAC			6
#define VIRTUAL_SE_CMD_AES_RNG_DBRG		7
#define VIRTUAL_SE_CMD_AES_ENCRYPT_WITH_KEY	8
#define VIRTUAL_SE_CMD_AES_DECRYPT_WITH_KEY	9

#define VIRTUAL_SE_CMD_ELP_RNG1_RAND		1
#define VIRTUAL_SE_CMD_ELP_RNG1_TRNG		2

#define VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT (512 / 8)
#define VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT (1024 / 8)

#define VIRTUAL_SE_CMD_RSA_ALLOC_KEY	1
#define VIRTUAL_SE_CMD_RSA_RELEASE_KEY	2
#define VIRTUAL_SE_CMD_RSA_SET_EXPKEY	3
#define VIRTUAL_SE_CMD_RSA_SET_MODKEY	4
#define VIRTUAL_SE_CMD_RSA_ENCDEC	5

#define TEGRA_VIRTUAL_SE_TIMEOUT_1S 1000000

#define TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE	64
#define TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE	128
#define TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE	192
#define TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE	256

#define TEGRA_VIRTUAL_SE_AES_CMAC_STATE_SIZE	16
#define TEGRA_VIRTUAL_SE_SHA1_STATE_SIZE	20
#define TEGRA_VIRTUAL_SE_SHA224_STATE_SIZE	32
#define TEGRA_VIRTUAL_SE_SHA256_STATE_SIZE	32
#define TEGRA_VIRTUAL_SE_SHA384_STATE_SIZE	64
#define TEGRA_VIRTUAL_SE_SHA512_STATE_SIZE	64

#define TEGRA_VIRTUAL_SE_RSA512_STATE_SIZE	32
#define TEGRA_VIRTUAL_SE_RSA1024_STATE_SIZE	32
#define TEGRA_VIRTUAL_SE_RSA1536_STATE_SIZE	32
#define TEGRA_VIRTUAL_SE_RSA2048_STATE_SIZE	32

#define TEGRA_VIRTUAL_SE_MAX_BUFFER_SIZE 0x1000000

#define AES_KEYTBL_TYPE_KEY 1
#define AES_KEYTBL_TYPE_OIV 2
#define AES_KEYTBL_TYPE_UIV 4

#define TEGRA_VIRTUAL_SE_AES_KEYSLOT_LABEL	"NVSEAES"

#define TEGRA_VIRTUAL_SE_AES_IV_SIZE 16
#define TEGRA_VIRTUAL_SE_AES_LCTR_SIZE 16
#define TEGRA_VIRTUAL_SE_AES_LCTR_CNTN 1

#define TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE 16
#define TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE 16

#define TEGRA_VIRTUAL_SE_RNG_IV_SIZE 16
#define TEGRA_VIRTUAL_SE_RNG_DT_SIZE 16
#define TEGRA_VIRTUAL_SE_RNG_KEY_SIZE 16
#define TEGRA_VIRTUAL_SE_RNG_SEED_SIZE (TEGRA_VIRTUAL_SE_RNG_IV_SIZE + \
					TEGRA_VIRTUAL_SE_RNG_KEY_SIZE + \
					TEGRA_VIRTUAL_SE_RNG_DT_SIZE)

/* Security Engine RSA context */
struct tegra_virtual_se_rsa_context {
	struct tegra_virtual_se_dev *se_dev;
	u32 rsa_keyslot;
	u32 exponent_length;
	u32 module_length;
	bool key_alloated;
};

/* Lock for IVC channel */
static DEFINE_MUTEX(se_ivc_lock);

static struct tegra_hv_ivc_cookie *g_ivck;
static struct tegra_virtual_se_dev *g_virtual_se_dev[VIRTUAL_MAX_SE_ENGINE_NUM];
static struct completion tegra_vse_complete;

#define GET_MSB(x)  ((x) >> (8*sizeof(x)-1))
static void tegra_virtual_se_leftshift_onebit(u8 *in_buf, u32 size, u8 *org_msb)
{
	u8 carry;
	u32 i;

	*org_msb = GET_MSB(in_buf[0]);

	/* left shift one bit */
	in_buf[0] <<= 1;
	for (carry = 0, i = 1; i < size; i++) {
		carry = GET_MSB(in_buf[i]);
		in_buf[i-1] |= carry;
		in_buf[i] <<= 1;
	}
}

static int tegra_hv_vse_send_ivc(
	struct tegra_virtual_se_dev *se_dev,
	struct tegra_hv_ivc_cookie *pivck,
	void *pbuf,
	int length)
{
	u32 timeout;

	timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;
	mutex_lock(&se_ivc_lock);
	while (tegra_hv_ivc_channel_notified(pivck) != 0) {
		if (!timeout) {
			mutex_unlock(&se_ivc_lock);
			dev_err(se_dev->dev, "ivc reset timeout\n");
			return -EINVAL;
		}
		udelay(1);
		timeout--;
	}

	timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;
	while (tegra_hv_ivc_can_write(pivck) == 0) {
		if (!timeout) {
			mutex_unlock(&se_ivc_lock);
			dev_err(se_dev->dev, "ivc send message timeout\n");
			return -EINVAL;
		}
		udelay(1);
		timeout--;
	}

	tegra_hv_ivc_write(pivck, pbuf, length);
	mutex_unlock(&se_ivc_lock);
	return 0;
}

static int tegra_hv_vse_prepare_ivc_linked_list(
	struct tegra_virtual_se_dev *se_dev, struct scatterlist *sg,
	u32 total_len, int max_ll_len, int block_size,
	struct tegra_virtual_se_addr *src_addr,
	int *num_lists, enum dma_data_direction dir,
	unsigned int *num_mapped_sgs)
{
	struct scatterlist *src_sg;
	int err = 0;
	int sg_count = 0;
	int i = 0;
	int len, process_len;
	u32 addr, addr_offset;

	src_sg = sg;
	while (src_sg && total_len) {
		err = dma_map_sg(se_dev->dev, src_sg, 1, dir);
		if (!err) {
			dev_err(se_dev->dev, "dma_map_sg() error\n");
			err = -EINVAL;
			goto exit;
		}
		sg_count++;
		len = min(src_sg->length, (size_t)total_len);
		addr = sg_dma_address(src_sg);
		addr_offset = 0;
		while (len >= TEGRA_VIRTUAL_SE_MAX_BUFFER_SIZE) {
			process_len = TEGRA_VIRTUAL_SE_MAX_BUFFER_SIZE -
				block_size;
			if (i > max_ll_len) {
				dev_err(se_dev->dev,
					"Unsupported no. of list %d\n", i);
				err = -EINVAL;
				goto exit;
			}
			src_addr[i].lo = addr + addr_offset;
			src_addr[i].hi = process_len;
			i++;
			addr_offset += process_len;
			total_len -= process_len;
			len -= process_len;
		}
		if (len) {
			if (i > max_ll_len) {
				dev_err(se_dev->dev,
					"Unsupported no. of list %d\n", i);
				err = -EINVAL;
				goto exit;
			}
			src_addr[i].lo = addr + addr_offset;
			src_addr[i].hi = len;
			i++;
		}
		total_len -= len;
		src_sg = sg_next(src_sg);
	}
	*num_lists = (i + *num_lists);
	*num_mapped_sgs = sg_count;

	return 0;
exit:
	src_sg = sg;
	while (src_sg && sg_count--) {
		dma_unmap_sg(se_dev->dev, src_sg, 1, dir);
		src_sg = sg_next(src_sg);
	}
	*num_mapped_sgs = 0;

	return err;
}

static int tegra_hv_vse_count_sgs(struct scatterlist *sl, u32 nbytes)
{
	struct scatterlist *sg = sl;
	int sg_nents = 0;

	while (sg) {
		sg = sg_next(sg);
		sg_nents++;
	}

	return sg_nents;
}

static int tegra_hv_vse_send_sha_data(struct tegra_virtual_se_dev *se_dev,
				struct ahash_request *req,
				struct tegra_virtual_se_ivc_msg_t *ivc_req_msg,
				u32 count, bool islast)
{
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx = NULL;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_virtual_se_req_context *req_ctx;
	struct tegra_vse_tag *priv_data_ptr;
	union tegra_virtual_se_sha_args *psha = NULL;
	int time_left;
	int err = 0;
	u64 total_count = 0, msg_len = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	if (!ivc_req_msg) {
		dev_err(se_dev->dev,
			"%s Invalid ivc_req_msg\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	req_ctx = ahash_request_ctx(req);
	total_count = req_ctx->total_count;

	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_tx->engine = VIRTUAL_SE_SHA;
	ivc_tx->cmd = VIRTUAL_SE_CMD_SHA_HASH_1;

	psha = &(ivc_tx->args.sha);
	psha->op_hash1.streamid = se_dev->stream_id;
	psha->op_hash1.mode = req_ctx->mode;
	psha->op_hash1.msg_total_length0 = count;
	psha->op_hash1.msg_total_length1 = 0;
	psha->op_hash1.msg_total_length2 = 0;
	psha->op_hash1.msg_total_length3 = 0;
	psha->op_hash1.msg_block_length = count;
	psha->op_hash1.msg_left_length0 = count;
	psha->op_hash1.msg_left_length1 = 0;
	psha->op_hash1.msg_left_length2 = 0;
	psha->op_hash1.msg_left_length3 = 0;

	if (islast) {
		psha->op_hash1.msg_total_length0 = total_count & 0xFFFFFFFF;
		psha->op_hash1.msg_total_length1 = total_count >> 32;
	} else {
		msg_len = count + 8;
		psha->op_hash1.msg_left_length0 = msg_len & 0xFFFFFFFF;
		psha->op_hash1.msg_left_length1 = msg_len >> 32;

		if (req_ctx->is_first) {
			psha->op_hash1.msg_total_length0 = msg_len & 0xFFFFFFFF;
			psha->op_hash1.msg_total_length1 = msg_len >> 32;
		} else {
			msg_len += 8;
			psha->op_hash1.msg_total_length0 = msg_len & 0xFFFFFFFF;
			psha->op_hash1.msg_total_length1 = msg_len >> 32;
		}
	}

	ivc_req_msg->hdr.num_reqs = 1;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	vse_thread_start = true;
	init_completion(&priv->alg_complete);

	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		err = -ENODEV;
		goto exit;
	}

	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err)
		goto exit;

	time_left = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	if (time_left == 0) {
		dev_err(se_dev->dev, "%s timeout\n", __func__);
		err = -ETIMEDOUT;
	}
exit:
	mutex_unlock(&se_dev->server_lock);
	devm_kfree(se_dev->dev, priv);

	return err;
}

static int tegra_hv_vse_sha_send_one(struct ahash_request *req,
				u32 nbytes, bool islast)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx = NULL;
	struct tegra_virtual_se_addr *src_addr = NULL;
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	int err = 0;

	ivc_req_msg = devm_kzalloc(se_dev->dev, sizeof(*ivc_req_msg),
			GFP_KERNEL);
	if (!ivc_req_msg)
		return -ENOMEM;

	ivc_tx = &ivc_req_msg->d[0].tx;
	src_addr = ivc_tx->args.sha.op_hash1.src.addr;

	src_addr[0].lo = req_ctx->sha_buf_addr;
	src_addr[0].hi = nbytes;

	ivc_tx->args.sha.op_hash1.src.number = 1;
	ivc_tx->args.sha.op_hash1.dst = (u64)req_ctx->hash_result_addr;
	memcpy(ivc_tx->args.sha.op_hash1.hash, req_ctx->hash_result,
		req_ctx->digest_size);

	err = tegra_hv_vse_send_sha_data(se_dev, req, ivc_req_msg,
				nbytes, islast);
	if (err) {
		dev_err(se_dev->dev, "%s error %d\n", __func__, err);
		goto exit;
	}
exit:
	devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

static int tegra_hv_vse_sha_fast_path(struct ahash_request *req,
					bool is_last, bool process_cur_req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	u32 bytes_process_in_req = 0, num_blks;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx = NULL;
	struct tegra_virtual_se_addr *src_addr = NULL;
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	u32 num_mapped_sgs = 0;
	u32 num_lists = 0;
	struct scatterlist *sg;
	int err = 0;
	u32 nbytes_in_req = req->nbytes;

	/* process_cur_req  is_last :
	 *     false         false  : update()                   -> hash
	 *     true          true   : finup(), digest()          -> hash
	 *                   true   : finup(), digest(), final() -> result
	 */
	if ((process_cur_req == false && is_last == false) ||
		(process_cur_req == true && is_last == true)) {
		/* When calling update(), if req->nbytes is aligned with
		 * req_ctx->blk_size, reduce req->nbytes with req_ctx->blk_size
		 * to avoid hashing zero length input at the end.
		 */
		if (req_ctx->residual_bytes == req_ctx->blk_size) {
			err = tegra_hv_vse_sha_send_one(req,
					req_ctx->residual_bytes, false);
			if (err) {
				dev_err(se_dev->dev,
					"%s: failed to send residual data %u\n",
					__func__, req_ctx->residual_bytes);
				return err;
			}
			req_ctx->residual_bytes = 0;
		}

		num_blks = nbytes_in_req / req_ctx->blk_size;
		req_ctx->residual_bytes =
			nbytes_in_req - (num_blks * req_ctx->blk_size);

		if (num_blks > 0 && req_ctx->residual_bytes == 0) {
			/* blk_size aligned. reduce size with one blk and
			 * handle it in the next call.
			 */
			req_ctx->residual_bytes = req_ctx->blk_size;
			req_ctx->total_count += req_ctx->residual_bytes;
			num_blks--;
			sg_pcopy_to_buffer(req->src, sg_nents(req->src),
				req_ctx->sha_buf, req_ctx->residual_bytes,
				num_blks * req_ctx->blk_size);
		} else {
			/* not aligned at all */
			req_ctx->total_count += req_ctx->residual_bytes;
			sg_pcopy_to_buffer(req->src, sg_nents(req->src),
				req_ctx->sha_buf, req_ctx->residual_bytes,
				num_blks * req_ctx->blk_size);
		}
		nbytes_in_req -= req_ctx->residual_bytes;

		dev_dbg(se_dev->dev, "%s: req_ctx->residual_bytes %u\n",
			__func__, req_ctx->residual_bytes);

		if (num_blks > 0) {
			ivc_req_msg = devm_kzalloc(se_dev->dev,
				sizeof(*ivc_req_msg), GFP_KERNEL);
			if (!ivc_req_msg)
				return -ENOMEM;

			ivc_tx = &ivc_req_msg->d[0].tx;
			src_addr = ivc_tx->args.sha.op_hash1.src.addr;

			bytes_process_in_req = num_blks * req_ctx->blk_size;
			dev_dbg(se_dev->dev, "%s: bytes_process_in_req %u\n",
				__func__, bytes_process_in_req);

			err = tegra_hv_vse_prepare_ivc_linked_list(se_dev,
					req->src, bytes_process_in_req,
					(TEGRA_HV_VSE_SHA_MAX_LL_NUM_1 -
						num_lists),
					req_ctx->blk_size,
					src_addr,
					&num_lists,
					DMA_TO_DEVICE, &num_mapped_sgs);
			if (err) {
				dev_err(se_dev->dev, "%s: ll error %d\n",
					__func__, err);
				goto unmap;
			}

			dev_dbg(se_dev->dev, "%s: num_lists %u\n",
				__func__, num_lists);

			ivc_tx->args.sha.op_hash1.src.number = num_lists;
			ivc_tx->args.sha.op_hash1.dst
				= (u64)req_ctx->hash_result_addr;
			memcpy(ivc_tx->args.sha.op_hash1.hash,
				req_ctx->hash_result, req_ctx->digest_size);

			req_ctx->total_count += bytes_process_in_req;

			err = tegra_hv_vse_send_sha_data(se_dev, req,
				ivc_req_msg, bytes_process_in_req, false);
			if (err) {
				dev_err(se_dev->dev, "%s error %d\n",
					__func__, err);
				goto unmap;
			}
unmap:
			sg = req->src;
			while (sg && num_mapped_sgs--) {
				dma_unmap_sg(se_dev->dev, sg, 1, DMA_TO_DEVICE);
				sg = sg_next(sg);
			}
			devm_kfree(se_dev->dev, ivc_req_msg);
		}

		if (req_ctx->residual_bytes > 0 &&
			req_ctx->residual_bytes < req_ctx->blk_size) {
			/* At this point, the buffer is not aligned with
			 * blk_size. Thus, buffer alignment need to be done via
			 * slow path.
			 */
			req_ctx->force_align = true;
		}
	}

	req_ctx->is_first = false;
	if (is_last) {
		/* handle the last data in finup() , digest() */
		if (req_ctx->residual_bytes > 0) {
			err = tegra_hv_vse_sha_send_one(req,
					req_ctx->residual_bytes, true);
			if (err) {
				dev_err(se_dev->dev,
					"%s: failed to send last data %u\n",
					__func__, req_ctx->residual_bytes);
				return err;
			}
			req_ctx->residual_bytes = 0;
		}

		if (req->result) {
			memcpy(req->result, req_ctx->hash_result,
				req_ctx->digest_size);
		} else {
			dev_err(se_dev->dev, "Invalid clinet result buffer\n");
		}
	}

	return err;
}

static int tegra_hv_vse_sha_slow_path(struct ahash_request *req,
					bool is_last, bool process_cur_req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	u32 nblk_bytes = 0, num_blks, buflen = SZ_4M;
	u32 length = 0, skip = 0, offset = 0;
	u64 total_bytes = 0, left_bytes = 0;
	int err = 0;

	if ((process_cur_req == false && is_last == false) ||
		(process_cur_req == true && is_last == true)) {

		total_bytes = req_ctx->residual_bytes + req->nbytes;
		num_blks = total_bytes / req_ctx->blk_size;
		nblk_bytes = num_blks * req_ctx->blk_size;
		offset = req_ctx->residual_bytes;

		/* if blk_size aligned, reduce 1 blk_size for the last hash */
		if ((total_bytes - nblk_bytes) == 0)
			total_bytes -= req_ctx->blk_size;

		left_bytes = req->nbytes;

		while (total_bytes >= req_ctx->blk_size) {
			/* Copy to linear buffer */
			num_blks = total_bytes / req_ctx->blk_size;
			nblk_bytes = num_blks * req_ctx->blk_size;
			length = min(buflen, nblk_bytes) - offset;

			sg_pcopy_to_buffer(req->src, sg_nents(req->src),
				req_ctx->sha_buf + offset, length, skip);
			skip += length;
			req_ctx->total_count += length;

			/* Hash */
			err = tegra_hv_vse_sha_send_one(req,
						length + offset, false);
			if (err) {
				dev_err(se_dev->dev,
					"%s: failed to send one %u\n",
					__func__, length + offset);
				return err;
			}
			total_bytes -= (length + offset);
			left_bytes -= length;
			offset = 0;
		}

		/* left_bytes <= req_ctx->blk_size */
		if ((req_ctx->residual_bytes + req->nbytes) >=
						req_ctx->blk_size) {
			/* Processed in while() loop */
			sg_pcopy_to_buffer(req->src, sg_nents(req->src),
					req_ctx->sha_buf, left_bytes, skip);
			req_ctx->total_count += left_bytes;
			req_ctx->residual_bytes = left_bytes;
		} else {
			/* Accumulate the request */
			sg_pcopy_to_buffer(req->src, sg_nents(req->src),
				req_ctx->sha_buf + req_ctx->residual_bytes,
				req->nbytes, skip);
			req_ctx->total_count += req->nbytes;
			req_ctx->residual_bytes += req->nbytes;
		}

		if (req_ctx->force_align == true &&
			req_ctx->residual_bytes == req_ctx->blk_size) {
			/* At this point, the buffer is aligned with blk_size.
			 * Thus, the next call can use fast path.
			 */
			req_ctx->force_align = false;
		}
	}

	req_ctx->is_first = false;
	if (is_last) {
		/* handle the last data in finup() , digest() */
		if (req_ctx->residual_bytes > 0) {
			err = tegra_hv_vse_sha_send_one(req,
					req_ctx->residual_bytes, true);
			if (err) {
				dev_err(se_dev->dev,
					"%s: failed to send last data%u\n",
					__func__, req_ctx->residual_bytes);
				return err;
			}
			req_ctx->residual_bytes = 0;
		}

		if (req->result) {
			memcpy(req->result, req_ctx->hash_result,
				req_ctx->digest_size);
		} else {
			dev_err(se_dev->dev, "Invalid clinet result buffer\n");
		}
	}

	return err;
}

static int tegra_hv_vse_sha_op(struct ahash_request *req, bool is_last,
			   bool process_cur_req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	u32 mode;
	u32 num_blks;
	int ret;
	struct sha_zero_length_vector zero_vec[] = {
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

	if (req->nbytes == 0) {
		if (req_ctx->total_count > 0) {
			if (is_last == false) {
				dev_info(se_dev->dev, "empty packet\n");
				return 0;
			}

			if (req_ctx->residual_bytes > 0) { /*final() */
				ret = tegra_hv_vse_sha_send_one(req,
					req_ctx->residual_bytes, true);
				if (ret) {
					dev_err(se_dev->dev,
					"%s: failed to send last data %u\n",
					__func__, req_ctx->residual_bytes);
					return ret;
				}
				req_ctx->residual_bytes = 0;
			}

			if (req->result) {
				memcpy(req->result, req_ctx->hash_result,
					req_ctx->digest_size);
			} else {
				dev_err(se_dev->dev,
					"Invalid clinet result buffer\n");
			}
			return 0;
		}

		/* If the request length is zero, SW WAR for zero length SHA
		 * operation since SE HW can't accept zero length SHA operation
		 */
		if (req_ctx->mode == VIRTUAL_SE_OP_MODE_SHA1)
			mode = VIRTUAL_SE_OP_MODE_SHA1;
		else
			mode = req_ctx->mode - VIRTUAL_SE_OP_MODE_SHA224 + 1;

		if (req->result) {
			memcpy(req->result,
				zero_vec[mode].digest, zero_vec[mode].size);
		} else {
			dev_err(se_dev->dev, "Invalid clinet result buffer\n");
		}
		return 0;
	}

	num_blks = req->nbytes / req_ctx->blk_size;

	if (req_ctx->force_align == false && num_blks > 0)
		ret = tegra_hv_vse_sha_fast_path(req, is_last, process_cur_req);
	else
		ret = tegra_hv_vse_sha_slow_path(req, is_last, process_cur_req);

	return ret;
}

static int tegra_hv_vse_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm;
	struct tegra_virtual_se_req_context *req_ctx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	req_ctx = ahash_request_ctx(req);
	if (!req_ctx) {
		dev_err(se_dev->dev, "SHA req_ctx not valid\n");
		return -EINVAL;
	}

	tfm = crypto_ahash_reqtfm(req);
	if (!tfm) {
		dev_err(se_dev->dev, "SHA transform not valid\n");
		return -EINVAL;
	}

	req_ctx->digest_size = crypto_ahash_digestsize(tfm);
	switch (req_ctx->digest_size) {
	case SHA1_DIGEST_SIZE:
		req_ctx->mode = VIRTUAL_SE_OP_MODE_SHA1;
		req_ctx->blk_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA224_DIGEST_SIZE:
		req_ctx->mode = VIRTUAL_SE_OP_MODE_SHA224;
		req_ctx->blk_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA256_DIGEST_SIZE:
		req_ctx->mode = VIRTUAL_SE_OP_MODE_SHA256;
		req_ctx->blk_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA384_DIGEST_SIZE:
		req_ctx->mode = VIRTUAL_SE_OP_MODE_SHA384;
		req_ctx->blk_size =
			VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT;
		break;
	case SHA512_DIGEST_SIZE:
		req_ctx->mode = VIRTUAL_SE_OP_MODE_SHA512;
		req_ctx->blk_size =
			VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT;
		break;
	default:
		return -EINVAL;
	}
	req_ctx->sha_buf = dma_alloc_coherent(se_dev->dev, SZ_4M,
					&req_ctx->sha_buf_addr, GFP_KERNEL);
	if (!req_ctx->sha_buf) {
		dev_err(se_dev->dev, "Cannot allocate memory to sha_buf\n");
		return -ENOMEM;
	}

	req_ctx->hash_result = dma_alloc_coherent(
			se_dev->dev, (TEGRA_HV_VSE_SHA_MAX_BLOCK_SIZE * 2),
			&req_ctx->hash_result_addr, GFP_KERNEL);
	if (!req_ctx->hash_result) {
		dma_free_coherent(se_dev->dev, SZ_4M,
				req_ctx->sha_buf, req_ctx->sha_buf_addr);
		req_ctx->sha_buf = NULL;
		dev_err(se_dev->dev, "Cannot allocate memory to hash_result\n");
		return -ENOMEM;
	}
	req_ctx->total_count = 0;
	req_ctx->is_first = true;
	req_ctx->residual_bytes = 0;
	req_ctx->req_context_initialized = true;
	req_ctx->force_align = false;

	return 0;
}

static void tegra_hv_vse_sha_req_deinit(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);

	/* dma_free_coherent does not panic if addr is NULL */
	dma_free_coherent(se_dev->dev, SZ_4M,
			req_ctx->sha_buf, req_ctx->sha_buf_addr);
	req_ctx->sha_buf = NULL;

	dma_free_coherent(
		se_dev->dev, (TEGRA_HV_VSE_SHA_MAX_BLOCK_SIZE * 2),
		req_ctx->hash_result, req_ctx->hash_result_addr);
	req_ctx->hash_result = NULL;
	req_ctx->req_context_initialized = false;
}

static int tegra_hv_vse_sha_update(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	req_ctx = ahash_request_ctx(req);
	if (!req_ctx->req_context_initialized) {
		dev_err(se_dev->dev,
			"%s Request ctx not initialized\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);

	ret = tegra_hv_vse_sha_op(req, false, false);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_update failed - %d\n", ret);

	mutex_unlock(&se_dev->mtx);

	return ret;
}

static int tegra_hv_vse_sha_finup(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	req_ctx = ahash_request_ctx(req);
	if (!req_ctx->req_context_initialized) {
		dev_err(se_dev->dev,
			"%s Request ctx not initialized\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);

	ret = tegra_hv_vse_sha_op(req, true, true);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_finup failed - %d\n", ret);

	mutex_unlock(&se_dev->mtx);

	tegra_hv_vse_sha_req_deinit(req);

	return ret;
}

static int tegra_hv_vse_sha_final(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	req_ctx = ahash_request_ctx(req);
	if (!req_ctx->req_context_initialized) {
		dev_err(se_dev->dev,
			"%s Request ctx not initialized\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);
	/* Do not process data in given request */
	ret = tegra_hv_vse_sha_op(req, true, false);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_final failed - %d\n", ret);

	mutex_unlock(&se_dev->mtx);
	tegra_hv_vse_sha_req_deinit(req);

	return ret;
}

static int tegra_hv_vse_sha_digest(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	int ret = 0;

	if (!req) {
		dev_err(se_dev->dev, "SHA request not valid\n");
		return -EINVAL;
	}

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	ret = tegra_hv_vse_sha_init(req);
	if (ret) {
		dev_err(se_dev->dev, "%s init failed - %d\n", __func__, ret);
		return ret;
	}

	mutex_lock(&se_dev->mtx);
	ret = tegra_hv_vse_sha_op(req, true, true);
	if (ret)
		dev_err(se_dev->dev, "tegra_se_sha_digest failed - %d\n", ret);
	mutex_unlock(&se_dev->mtx);

	tegra_hv_vse_sha_req_deinit(req);

	return ret;
}

static int tegra_hv_vse_sha_export(struct ahash_request *req, void *out)
{
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);

	memcpy(out, req_ctx, sizeof(*req_ctx));
	return 0;
}

static int tegra_hv_vse_sha_import(struct ahash_request *req, const void *in)
{
	struct tegra_virtual_se_req_context *req_ctx = ahash_request_ctx(req);

	memcpy(req_ctx, in, sizeof(*req_ctx));
	return 0;
}

static int tegra_hv_vse_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_virtual_se_req_context));

	return 0;
}

static void tegra_hv_vse_sha_cra_exit(struct crypto_tfm *tfm)
{
}

static int tegra_hv_vse_rsa_init(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_final(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_digest(struct ahash_request *req)
{
	struct crypto_ahash *tfm = NULL;
	struct tegra_virtual_se_rsa_context *rsa_ctx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	u32 num_sgs;
	int err = 0;
	dma_addr_t dma_addr_out;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	int time_left;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;

	if (!req)
		return -EINVAL;

	tfm = crypto_ahash_reqtfm(req);
	if (!tfm)
		return -EINVAL;

	rsa_ctx = crypto_ahash_ctx(tfm);
	if (!rsa_ctx)
		return -EINVAL;

	if (!rsa_ctx->key_alloated) {
		dev_err(se_dev->dev, "RSA key not allocated\n");
		return -EINVAL;
	}

	if (!req->nbytes)
		return -EINVAL;

	if ((req->nbytes < TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE) ||
			(req->nbytes > TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE))
		return -EINVAL;

	num_sgs = tegra_hv_vse_count_sgs(req->src, req->nbytes);
	if (num_sgs > 1) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EINVAL;
	}

	ivc_req_msg = devm_kzalloc(se_dev->dev,
		sizeof(*ivc_req_msg),
		GFP_KERNEL);
	if (!ivc_req_msg) {
		dev_err(se_dev->dev,
			"\n Memory allocation failed\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		return -ENOMEM;
	}
	dma_addr_out = dma_map_single(se_dev->dev,
		req->result, req->nbytes, DMA_FROM_DEVICE);

	dma_map_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE);
	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx->engine = VIRTUAL_SE_RSA;
	ivc_tx->cmd = VIRTUAL_SE_CMD_RSA_ENCDEC;
	ivc_tx->args.rsa.op.keyslot = rsa_ctx->rsa_keyslot;
	ivc_tx->args.rsa.op.exp_length = rsa_ctx->exponent_length;
	ivc_tx->args.rsa.op.mod_length = rsa_ctx->module_length;
	ivc_tx->args.rsa.op.p_src = sg_dma_address(req->src);
	ivc_tx->args.rsa.op.p_dst = dma_addr_out;
	ivc_tx->args.rsa.op.streamid = se_dev->stream_id;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	vse_thread_start = true;
	init_completion(&priv->alg_complete);

	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		err = -ENODEV;
		goto exit;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		goto exit;
	}

	time_left = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (time_left == 0) {
		dev_err(se_dev->dev, "RSA digest timeout\n");
		err = -ETIMEDOUT;
	}

	devm_kfree(se_dev->dev, priv);

exit:
	dma_unmap_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE);
	dma_unmap_single(se_dev->dev,
		dma_addr_out, req->nbytes, DMA_FROM_DEVICE);
	devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

static int tegra_hv_vse_rsa_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_virtual_se_rsa_context *rsa_ctx = crypto_ahash_ctx(tfm);
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	u32 module_key_length = 0;
	u32 exponent_key_length = 0;
	u32 *pkeydata = (u32 *)key;
	u32 *ivc_key_data;
	int err = 0;
	int i = 0;
	int time_left;

	if (!rsa_ctx)
		return -EINVAL;

	ivc_req_msg =
		devm_kzalloc(se_dev->dev, sizeof(*ivc_req_msg), GFP_KERNEL);
	if (!ivc_req_msg) {
		dev_err(se_dev->dev,
			"\n Memory allocation failed\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		return -ENOMEM;
	}

	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx = &ivc_req_msg->d[0].tx;
	vse_thread_start = true;
	if (!rsa_ctx->key_alloated) {
		/* Allocate RSA key slot */
		ivc_tx->engine = VIRTUAL_SE_RSA;
		ivc_tx->cmd = VIRTUAL_SE_CMD_RSA_ALLOC_KEY;
		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_KEY_SLOT;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);

		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			err = -ENODEV;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			goto exit;
		}

		err = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (err == 0) {
			dev_err(se_dev->dev, "%s timeout\n", __func__);
			goto exit;
		}

		rsa_ctx->rsa_keyslot = priv->slot_num;
		rsa_ctx->key_alloated = true;
	}

	exponent_key_length = (keylen & (0xFFFF));
	module_key_length = (keylen >> 16);
	rsa_ctx->exponent_length = exponent_key_length;
	rsa_ctx->module_length = module_key_length;

	if (exponent_key_length) {
		/* Send RSA Exponent Key */
		ivc_tx->engine = VIRTUAL_SE_RSA;
		ivc_tx->cmd = VIRTUAL_SE_CMD_RSA_SET_EXPKEY;
		ivc_tx->args.rsa.key.slot = rsa_ctx->rsa_keyslot;
		ivc_tx->args.rsa.key.length = exponent_key_length;
		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_PROCESS;
		priv->se_dev = se_dev;

		ivc_key_data = (u32 *)ivc_tx->args.rsa.key.data;
		for (i = ((exponent_key_length / 4) - 1); i >= 0; i--)
			*(ivc_key_data + i) = *pkeydata++;

		init_completion(&priv->alg_complete);
		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			err = -ENODEV;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			goto exit;
		}
		time_left = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (time_left == 0) {
			dev_err(se_dev->dev, "%s timeout\n", __func__);
			goto exit;
		}
	}

	if (module_key_length) {
		/* Send RSA Module Key */
		ivc_tx->engine = VIRTUAL_SE_RSA;
		ivc_tx->cmd = VIRTUAL_SE_CMD_RSA_SET_MODKEY;
		ivc_tx->args.rsa.key.slot = rsa_ctx->rsa_keyslot;
		ivc_tx->args.rsa.key.length = module_key_length;
		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_PROCESS;
		priv->se_dev = se_dev;

		ivc_key_data = (u32 *)ivc_tx->args.rsa.key.data;
		for (i = ((module_key_length / 4) - 1); i >= 0; i--)
			*(ivc_key_data + i) = *pkeydata++;

		init_completion(&priv->alg_complete);
		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			err = -ENODEV;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			goto exit;
		}
		time_left = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (time_left == 0) {
			dev_err(se_dev->dev, "%s timeout\n", __func__);
			err = -ETIMEDOUT;
		}
	}

exit:
	devm_kfree(se_dev->dev, priv);
	devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

static int tegra_hv_vse_rsa_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_cra_init(struct crypto_tfm *tfm)
{
	return 0;
}

static void tegra_hv_vse_rsa_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_rsa_context *rsa_ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int err;

	if (!rsa_ctx || !rsa_ctx->key_alloated)
		return;

	ivc_req_msg = devm_kzalloc(se_dev->dev,
			sizeof(*ivc_req_msg),
			GFP_KERNEL);
	if (!ivc_req_msg) {
		dev_err(se_dev->dev,
			"\n Memory allocation failed\n");
		return;
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		return;
	}

	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_tx->engine = VIRTUAL_SE_RSA;
	ivc_tx->cmd = VIRTUAL_SE_CMD_RSA_RELEASE_KEY;
	ivc_tx->args.rsa.key.slot = rsa_ctx->rsa_keyslot;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);
	vse_thread_start = true;
	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		err = -ENODEV;
		goto free_mem;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		goto free_mem;
	}

	err = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (err == 0)
		dev_err(se_dev->dev, "%s timeout\n", __func__);
	devm_kfree(se_dev->dev, priv);

free_mem:
	devm_kfree(se_dev->dev, ivc_req_msg);
}

static int tegra_hv_vse_aes_set_keyiv(struct tegra_virtual_se_dev *se_dev,
	u8 *data, u32 keylen, u8 keyslot, u8 type)
{
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int ret;

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ivc_req_msg = devm_kzalloc(se_dev->dev,
			sizeof(*ivc_req_msg),
			GFP_KERNEL);
	if (!ivc_req_msg) {
		devm_kfree(se_dev->dev, priv);
		dev_err(se_dev->dev, "\n Memory allocation failed\n");
		return -ENOMEM;
	}

	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_tx->engine = VIRTUAL_SE_AES1;
	ivc_tx->cmd = VIRTUAL_SE_CMD_AES_SET_KEY;
	ivc_tx->args.aes.key.slot = keyslot;
	ivc_tx->args.aes.key.type = type;

	if (type & AES_KEYTBL_TYPE_KEY) {
		ivc_tx->args.aes.key.length = keylen;
		memcpy(ivc_tx->args.aes.key.data, data, keylen);
	}

	if (type & AES_KEYTBL_TYPE_OIV)
		memcpy(ivc_tx->args.aes.key.oiv, data,
			TEGRA_VIRTUAL_SE_AES_IV_SIZE);

	if (type & AES_KEYTBL_TYPE_UIV)
		memcpy(ivc_tx->args.aes.key.uiv, data,
			TEGRA_VIRTUAL_SE_AES_IV_SIZE);

	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);
	vse_thread_start = true;

	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		err = -ENODEV;
		goto end;
	}
	err = tegra_hv_vse_send_ivc(se_dev,
			pivck,
			ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		goto end;
	}

	ret = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (ret == 0) {
		dev_err(se_dev->dev, "%s timeout\n", __func__);
		err = -ETIMEDOUT;
	}
	devm_kfree(se_dev->dev, priv);

end:
	devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

void tegra_hv_vse_prpare_cmd(struct tegra_virtual_se_dev *se_dev,
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx,
	struct tegra_virtual_se_aes_req_context *req_ctx,
	struct tegra_virtual_se_aes_context *aes_ctx,
	struct ablkcipher_request *req)
{
	ivc_tx->engine = req_ctx->engine_id;
	if (req_ctx->encrypt == true)
		ivc_tx->cmd = VIRTUAL_SE_CMD_AES_ENCRYPT;
	else
		ivc_tx->cmd = VIRTUAL_SE_CMD_AES_DECRYPT;

	ivc_tx->args.aes.op.keyslot = aes_ctx->aes_keyslot;
	ivc_tx->args.aes.op.key_length = aes_ctx->keylen;
	ivc_tx->args.aes.op.streamid = se_dev->stream_id;
	ivc_tx->args.aes.op.mode = req_ctx->op_mode;
	ivc_tx->args.aes.op.ivsel = AES_ORIGINAL_IV;
	if (req->info) {
		memcpy(ivc_tx->args.aes.op.lctr, req->info,
				TEGRA_VIRTUAL_SE_AES_LCTR_SIZE);

		if (req_ctx->op_mode == AES_CTR)
			ivc_tx->args.aes.op.ctr_cntn =
					TEGRA_VIRTUAL_SE_AES_LCTR_CNTN;
		else if (req_ctx->op_mode == AES_CBC)
			ivc_tx->args.aes.op.ivsel = AES_IV_REG;
		else
			ivc_tx->args.aes.op.ivsel = AES_ORIGINAL_IV | 0x80;
	}
}

static int status_to_errno(int err)
{
	switch (err) {
	case -7:	/* ERR_NOT_VALID */
		return -EPERM;
	case -8:	/* ERR_INVALID_ARGS */
		return -EINVAL;
	case -24:	/* ERR_NOT_SUPPORTED */
		return -EOPNOTSUPP;
	}
	return err;
}

static void complete_call_back(void *data)
{
	int k;
	struct ablkcipher_request *req;
	struct tegra_vse_priv_data *priv =
		(struct tegra_vse_priv_data *)data;
	int err = status_to_errno(priv->rx_status);
	int num_sgs;
	void *buf;

	if (!priv) {
		pr_err("%s:%d\n", __func__, __LINE__);
		return;
	}

	dma_sync_single_for_cpu(priv->se_dev->dev, priv->buf_addr,
		priv->gather_buf_sz, DMA_BIDIRECTIONAL);
	buf = priv->buf;
	for (k = 0; k < priv->req_cnt; k++) {
		req = priv->reqs[k];
		if (!req) {
			pr_err("\n%s:%d\n", __func__, __LINE__);
			return;
		}

		num_sgs = tegra_hv_vse_count_sgs(req->dst, req->nbytes);
		if (num_sgs == 1)
			memcpy(sg_virt(req->dst), buf, req->nbytes);
		else
			sg_copy_from_buffer(req->dst, num_sgs,
				buf, req->nbytes);
		buf += req->nbytes;
		if (req->base.complete)
			req->base.complete(&req->base, err);
	}
	dma_unmap_sg(priv->se_dev->dev, &priv->sg, 1, DMA_BIDIRECTIONAL);
	kfree(priv->buf);
}

static int tegra_hv_se_setup_ablk_req(struct tegra_virtual_se_dev *se_dev,
	struct tegra_vse_priv_data *priv)
{
	struct ablkcipher_request *req;
	void *buf;
	int i = 0;
	u32 num_sgs;

	priv->buf = kmalloc(se_dev->gather_buf_sz, GFP_KERNEL);
	if (!priv->buf)
		return -ENOMEM;

	buf = priv->buf;
	for (i = 0; i < se_dev->req_cnt; i++) {
		req = se_dev->reqs[i];
		num_sgs = tegra_hv_vse_count_sgs(req->src, req->nbytes);
		if (num_sgs == 1)
			memcpy(buf, sg_virt(req->src), req->nbytes);
		else
			sg_copy_to_buffer(req->src, num_sgs, buf, req->nbytes);

		buf += req->nbytes;
	}

	sg_init_one(&priv->sg, priv->buf, se_dev->gather_buf_sz);
	dma_map_sg(se_dev->dev, &priv->sg, 1, DMA_BIDIRECTIONAL);
	priv->buf_addr = sg_dma_address(&priv->sg);

	return 0;
}

static void tegra_hv_vse_process_new_req(struct tegra_virtual_se_dev *se_dev)
{
	struct ablkcipher_request *req;
	struct tegra_virtual_se_aes_req_context *req_ctx;
	struct tegra_virtual_se_aes_context *aes_ctx;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx = NULL;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	dma_addr_t cur_addr;
	int err = 0;
	int i, k;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg = NULL;
	int cur_map_cnt = 0;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;


	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto err_exit;
	}

	ivc_req_msg =
		devm_kzalloc(se_dev->dev, sizeof(*ivc_req_msg), GFP_KERNEL);
	if (!ivc_req_msg) {
		err = -ENOMEM;
		goto err_exit;
	}

	err = tegra_hv_se_setup_ablk_req(se_dev, priv);
	if (err) {
		dev_err(se_dev->dev,
			"\n %s failed %d\n", __func__, err);
		goto err_exit;
	}

	cur_addr = priv->buf_addr;
	for (k = 0; k < se_dev->req_cnt; k++) {
		req = se_dev->reqs[k];
		ivc_tx = &ivc_req_msg->d[k].tx;
		req_ctx = ablkcipher_request_ctx(req);
		aes_ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
		if (unlikely(!aes_ctx->is_key_slot_allocated)) {
			dev_err(se_dev->dev, "AES Key slot not allocated\n");
			err = -EINVAL;
			goto exit;
		}
		tegra_hv_vse_prpare_cmd(se_dev, ivc_tx, req_ctx, aes_ctx, req);
		ivc_tx->args.aes.op.src_ll_num = 1;
		ivc_tx->args.aes.op.dst_ll_num = 1;
		ivc_tx->args.aes.op.src_addr[0].lo = cur_addr;
		ivc_tx->args.aes.op.src_addr[0].hi = req->nbytes;
		ivc_tx->args.aes.op.dst_addr[0].lo = cur_addr;
		ivc_tx->args.aes.op.dst_addr[0].hi = req->nbytes;
		ivc_tx->args.aes.op.data_length = req->nbytes;
		cur_map_cnt++;
		cur_addr += req->nbytes;
	}
	ivc_req_msg->hdr.num_reqs = se_dev->req_cnt;

	priv->req_cnt = se_dev->req_cnt;
	priv->gather_buf_sz = se_dev->gather_buf_sz;
	priv->call_back_vse = &complete_call_back;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_AES_CRYPTO;
	priv->se_dev = se_dev;
	for (i = 0; i < se_dev->req_cnt; i++)
		priv->reqs[i] = se_dev->reqs[i];

	while (atomic_read(&se_dev->ivc_count) >=
			TEGRA_HV_VSE_NUM_SERVER_REQ)
		usleep_range(8, 10);

	atomic_add(1, &se_dev->ivc_count);
	vse_thread_start = true;
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		dev_err(se_dev->dev,
			"\n %s send ivc failed %d\n", __func__, err);
		goto exit;
	}
	goto exit_return;

exit:
	dma_unmap_sg(se_dev->dev, &priv->sg, 1, DMA_BIDIRECTIONAL);

err_exit:
	if (priv) {
		kfree(priv->buf);
		devm_kfree(se_dev->dev, priv);
	}
	for (k = 0; k < se_dev->req_cnt; k++) {
		req = se_dev->reqs[k];
		if (req->base.complete)
			req->base.complete(&req->base, err);
	}
exit_return:
	if (ivc_req_msg)
		devm_kfree(se_dev->dev, ivc_req_msg);
	se_dev->req_cnt = 0;
	se_dev->gather_buf_sz = 0;
}

static void tegra_hv_vse_work_handler(struct work_struct *work)
{
	struct tegra_virtual_se_dev *se_dev = container_of(work,
					struct tegra_virtual_se_dev, se_work);
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	unsigned long flags;
	bool process_requests;
	struct ablkcipher_request *req;

	mutex_lock(&se_dev->mtx);
	do {
		process_requests = false;
		spin_lock_irqsave(&se_dev->lock, flags);
		do {
			backlog = crypto_get_backlog(&se_dev->queue);
			async_req = crypto_dequeue_request(&se_dev->queue);
			if (!async_req)
				se_dev->work_q_busy = false;
			if (backlog) {
				backlog->complete(backlog, -EINPROGRESS);
				backlog = NULL;
			}

			if (async_req) {
				req = ablkcipher_request_cast(async_req);
				se_dev->reqs[se_dev->req_cnt] = req;
				se_dev->gather_buf_sz += req->nbytes;
				se_dev->req_cnt++;
				process_requests = true;
			} else {
				break;
			}
		} while (se_dev->queue.qlen &&
			(se_dev->req_cnt < TEGRA_HV_VSE_MAX_TASKS_PER_SUBMIT));
		spin_unlock_irqrestore(&se_dev->lock, flags);

		if (process_requests)
			tegra_hv_vse_process_new_req(se_dev);

	} while (se_dev->work_q_busy);
	mutex_unlock(&se_dev->mtx);
}

static int tegra_hv_vse_aes_queue_req(struct tegra_virtual_se_dev *se_dev,
				struct ablkcipher_request *req)
{
	unsigned long flags;
	bool idle = true;
	int err = 0;

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	if (req->nbytes % VIRTUAL_SE_AES_BLOCK_SIZE)
		return -EINVAL;

	if (!tegra_hv_vse_count_sgs(req->src, req->nbytes))
		return -EINVAL;

	spin_lock_irqsave(&se_dev->lock, flags);
	err = ablkcipher_enqueue_request(&se_dev->queue, req);
	if (se_dev->work_q_busy)
		idle = false;
	spin_unlock_irqrestore(&se_dev->lock, flags);

	if (idle) {
		spin_lock_irqsave(&se_dev->lock, flags);
		se_dev->work_q_busy = true;
		spin_unlock_irqrestore(&se_dev->lock, flags);
		queue_work(se_dev->vse_work_q, &se_dev->se_work);
	}

	return err;
}

static int tegra_hv_vse_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize =
		sizeof(struct tegra_virtual_se_aes_req_context);

	return 0;
}

static void tegra_hv_vse_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_aes_context *ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	int err;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;

	if (!ctx)
		return;

	if (!ctx->is_key_slot_allocated)
		return;

	if (ctx->is_keyslot_label)
		return;

	ivc_req_msg =
		devm_kzalloc(se_dev->dev,
			sizeof(*ivc_req_msg), GFP_KERNEL);
	if (!ivc_req_msg) {
		dev_err(se_dev->dev, "\n Memory allocation failed\n");
		return;
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto free_mem;

	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx = &ivc_req_msg->d[0].tx;
	/* Allocate AES key slot */
	ivc_tx->engine = VIRTUAL_SE_AES1;
	ivc_tx->cmd = VIRTUAL_SE_CMD_AES_RELEASE_KEY;
	ivc_tx->args.aes.key.slot = ctx->aes_keyslot;

	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);
	vse_thread_start = true;
	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		err = -ENODEV;
		goto free_mem;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		goto free_mem;
	}

	err = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (err == 0)
		dev_err(se_dev->dev, "%s timeout\n", __func__);
	devm_kfree(se_dev->dev, priv);

free_mem:
	devm_kfree(se_dev->dev, ivc_req_msg);
}

static int tegra_hv_vse_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_CBC;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_CBC;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_ECB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_ECB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_CTR;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_CTR;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_OFB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_OFB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_cmac_init(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	return 0;
}

static int tegra_hv_vse_cmac_update(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	return 0;
}

static int tegra_hv_vse_cmac_final(struct ahash_request *req)
{
	struct tegra_virtual_se_aes_cmac_context *cmac_ctx =
			crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct scatterlist *src_sg;
	struct sg_mapping_iter miter;
	u32 num_sgs, blocks_to_process, last_block_bytes = 0, bytes_to_copy = 0;
	unsigned int total_len, i = 0;
	bool padding_needed = false;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;
	u8 *temp_buffer = NULL;
	bool use_orig_iv = true;
	dma_addr_t cmac_dma_addr;
	u8 *cmac_buffer = NULL;
	dma_addr_t piv_buf_dma_addr;
	u8 *piv_buf = NULL;
	dma_addr_t result_dma_addr;
	int err = 0;
	int num_lists = 0;
	int time_left;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	unsigned int num_mapped_sgs = 0;

	blocks_to_process = req->nbytes / TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	/* num of bytes less than block size */
	if ((req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE) ||
		!blocks_to_process) {
		padding_needed = true;
		last_block_bytes =
			req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	} else {
		/* decrement num of blocks */
		blocks_to_process--;
		last_block_bytes = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	}
	ivc_req_msg = devm_kzalloc(se_dev->dev,
		sizeof(*ivc_req_msg), GFP_KERNEL);
	if (!ivc_req_msg)
		return -ENOMEM;

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		return -ENOMEM;
	}

	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_req_msg->hdr.num_reqs = 1;

	ivc_tx->engine = VIRTUAL_SE_AES1;
	ivc_tx->cmd = VIRTUAL_SE_CMD_AES_CMAC;
	src_sg = req->src;
	num_sgs = tegra_hv_vse_count_sgs(src_sg, req->nbytes);
	if (num_sgs > TEGRA_HV_VSE_AES_CMAC_MAX_LL_NUM) {
		dev_err(se_dev->dev,
			"\n Unsupported number of linked list %d\n", i);
		err = -ENOMEM;
		goto free_mem;
	}
	piv_buf = dma_alloc_coherent(se_dev->dev,
			TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			&piv_buf_dma_addr, GFP_KERNEL);
	if (!piv_buf) {
		dev_err(se_dev->dev, "can not allocate piv buffer");
		err = -ENOMEM;
		goto free_mem;
	}
	vse_thread_start = true;
	/* first process all blocks except last block */
	if (blocks_to_process) {
		total_len = blocks_to_process * TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
		ivc_tx->args.aes.op_cmac.keyslot = cmac_ctx->aes_keyslot;
		ivc_tx->args.aes.op_cmac.key_length = cmac_ctx->keylen;
		ivc_tx->args.aes.op_cmac.streamid = se_dev->stream_id;
		ivc_tx->args.aes.op_cmac.ivsel = AES_ORIGINAL_IV;

		err = tegra_hv_vse_prepare_ivc_linked_list(se_dev, req->src,
			total_len, TEGRA_HV_VSE_AES_CMAC_MAX_LL_NUM,
			TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			ivc_tx->args.aes.op_cmac.src.addr,
			&num_lists,
			DMA_TO_DEVICE, &num_mapped_sgs);
		if (err)
			goto exit;

		ivc_tx->args.aes.op_cmac.src.number = num_lists;
		ivc_tx->args.aes.op_cmac.data_length = total_len;
		ivc_tx->args.aes.op_cmac.dst = piv_buf_dma_addr;

		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_PROCESS;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);
		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			err = -ENODEV;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			goto exit;
		}

		time_left = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (time_left == 0) {
			dev_err(se_dev->dev, "cmac_final timeout\n");
			err = -ETIMEDOUT;
			goto exit;
		}
		use_orig_iv = false;
	}
	/* get the last block bytes from the sg_dma buffer using miter */
	src_sg = req->src;
	num_sgs = tegra_hv_vse_count_sgs(req->src, req->nbytes);
	sg_flags |= SG_MITER_FROM_SG;
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	local_irq_save(flags);
	total_len = 0;
	cmac_buffer = dma_alloc_coherent(se_dev->dev,
				TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
				&cmac_dma_addr, GFP_KERNEL);
	if (!cmac_buffer)
		goto exit;

	temp_buffer = cmac_buffer;
	while (sg_miter_next(&miter) && total_len < req->nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(req->nbytes - total_len));
		if ((req->nbytes - (total_len + len)) <= last_block_bytes) {
			bytes_to_copy =
				last_block_bytes -
				(req->nbytes - (total_len + len));
			memcpy(temp_buffer, miter.addr + (len - bytes_to_copy),
				bytes_to_copy);
			last_block_bytes -= bytes_to_copy;
			temp_buffer += bytes_to_copy;
		}
		total_len += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);

	/* process last block */
	if (padding_needed) {
		/* pad with 0x80, 0, 0 ... */
		last_block_bytes =
			req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
		cmac_buffer[last_block_bytes] = 0x80;
		for (i = last_block_bytes+1;
			i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] = 0;
		/* XOR with K2 */
		for (i = 0; i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] ^= cmac_ctx->K2[i];
	} else {
		/* XOR with K1 */
		for (i = 0; i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] ^= cmac_ctx->K1[i];
	}

	ivc_tx->args.aes.op_cmac.src.addr[0].lo = cmac_dma_addr;
	ivc_tx->args.aes.op_cmac.src.addr[0].hi =
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;

	if (use_orig_iv) {
		ivc_tx->args.aes.op_cmac.ivsel = AES_ORIGINAL_IV;
	} else {
		ivc_tx->args.aes.op_cmac.ivsel = AES_UPDATED_IV;
		err = tegra_hv_vse_aes_set_keyiv(se_dev, piv_buf,
				cmac_ctx->keylen,
				cmac_ctx->aes_keyslot,
				AES_KEYTBL_TYPE_UIV);
		if (err)
			goto exit;
	}
	result_dma_addr = dma_map_single(se_dev->dev,
				req->result,
				TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE,
				DMA_FROM_DEVICE);
	ivc_tx->args.aes.op_cmac.src.number = 1;
	ivc_tx->args.aes.op_cmac.keyslot = cmac_ctx->aes_keyslot;
	ivc_tx->args.aes.op_cmac.key_length = cmac_ctx->keylen;
	ivc_tx->args.aes.op_cmac.streamid = se_dev->stream_id;
	ivc_tx->args.aes.op_cmac.dst = result_dma_addr;
	ivc_tx->args.aes.op_cmac.data_length = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);
	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		err = -ENODEV;
		goto unmap_exit;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		goto unmap_exit;
	}

	time_left = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (time_left == 0) {
		dev_err(se_dev->dev, "cmac_final timeout\n");
		err = -ETIMEDOUT;
	}
unmap_exit:
	dma_unmap_single(se_dev->dev, result_dma_addr,
		TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE, DMA_FROM_DEVICE);

	src_sg = req->src;
	while (src_sg && num_mapped_sgs--) {
		dma_unmap_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE);
		src_sg = sg_next(src_sg);
	}
exit:
	if (cmac_buffer)
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			cmac_buffer, cmac_dma_addr);
	if (piv_buf)
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			piv_buf, piv_buf_dma_addr);
free_mem:
	devm_kfree(se_dev->dev, priv);
	devm_kfree(se_dev->dev, ivc_req_msg);

	return err;
}

static int tegra_hv_vse_cmac_finup(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	return 0;
}

static int tegra_hv_vse_cmac_digest(struct ahash_request *req)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	return tegra_hv_vse_cmac_init(req) ?: tegra_hv_vse_cmac_final(req);
}

static int tegra_hv_vse_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_virtual_se_aes_cmac_context *ctx =
			crypto_tfm_ctx(crypto_ahash_tfm(tfm));
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int err = 0;
	u8 piv[TEGRA_VIRTUAL_SE_AES_IV_SIZE];
	u32 *pbuf;
	dma_addr_t pbuf_adr;
	u8 const rb = 0x87;
	u8 msb;
	int time_left;

	if (!ctx)
		return -EINVAL;

	ivc_req_msg = devm_kzalloc(se_dev->dev, sizeof(*ivc_req_msg),
					GFP_KERNEL);
	if (!ivc_req_msg)
		return -ENOMEM;

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		dev_err(se_dev->dev, "Priv Data allocation failed\n");
		return -ENOMEM;
	}

	ivc_req_msg->hdr.num_reqs = 1;
	ivc_tx = &ivc_req_msg->d[0].tx;
	vse_thread_start = true;
	if (!ctx->is_key_slot_allocated) {
		/* Allocate AES key slot */
		ivc_tx->engine = VIRTUAL_SE_AES1;
		ivc_tx->cmd = VIRTUAL_SE_CMD_AES_ALLOC_KEY;
		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_KEY_SLOT;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);

		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			err = -ENODEV;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			goto exit;
		}

		time_left = wait_for_completion_timeout(
				&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (time_left == 0) {
			dev_err(se_dev->dev, "%s timeout\n",
				__func__);
			err = -ETIMEDOUT;
			goto exit;
		}
		ctx->aes_keyslot = priv->slot_num;
		ctx->is_key_slot_allocated = true;
	}

	ctx->keylen = keylen;
	err = tegra_hv_vse_aes_set_keyiv(se_dev, (u8 *)key, keylen,
			ctx->aes_keyslot, AES_KEYTBL_TYPE_KEY);
	if (err)
		goto exit;

	memset(piv, 0, TEGRA_VIRTUAL_SE_AES_IV_SIZE);
	err = tegra_hv_vse_aes_set_keyiv(se_dev, piv,
				ctx->keylen,
				ctx->aes_keyslot,
				AES_KEYTBL_TYPE_OIV);
	if (err)
		goto exit;

	pbuf = dma_alloc_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&pbuf_adr, GFP_KERNEL);
	if (!pbuf) {
		dev_err(se_dev->dev, "can not allocate dma buffer");
		err = -ENOMEM;
		goto exit;
	}
	memset(pbuf, 0, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);

	ivc_tx->engine = VIRTUAL_SE_AES1;
	ivc_tx->cmd = VIRTUAL_SE_CMD_AES_ENCRYPT;
	ivc_tx->args.aes.op.keyslot = ctx->aes_keyslot;
	ivc_tx->args.aes.op.key_length = ctx->keylen;
	ivc_tx->args.aes.op.streamid = se_dev->stream_id;
	ivc_tx->args.aes.op.mode = AES_CBC;
	ivc_tx->args.aes.op.ivsel = AES_ORIGINAL_IV;
	ivc_tx->args.aes.op.src_addr[0].lo = pbuf_adr;
	ivc_tx->args.aes.op.src_addr[0].hi = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	ivc_tx->args.aes.op.src_ll_num = 1;
	ivc_tx->args.aes.op.dst_addr[0].lo = pbuf_adr;
	ivc_tx->args.aes.op.dst_addr[0].hi = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	ivc_tx->args.aes.op.dst_ll_num = 1;
	ivc_tx->args.aes.op.data_length = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;

	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);

	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		err = -ENODEV;
		goto free_exit;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		goto free_exit;
	}
	time_left = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (time_left == 0) {
		dev_err(se_dev->dev, "cmac_final timeout\n");
		err = -ETIMEDOUT;
		goto free_exit;
	}

	/* compute K1 subkey */
	memcpy(ctx->K1, pbuf, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);

	tegra_virtual_se_leftshift_onebit(ctx->K1,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&msb);
	if (msb)
		ctx->K1[TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE - 1] ^= rb;

	/* compute K2 subkey */
	memcpy(ctx->K2,
		ctx->K1,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);
	tegra_virtual_se_leftshift_onebit(ctx->K2,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&msb);

	if (msb)
		ctx->K2[TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE - 1] ^= rb;

free_exit:
	if (pbuf) {
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			pbuf, pbuf_adr);
	}

exit:
	devm_kfree(se_dev->dev, priv);
	devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

static int tegra_hv_vse_cmac_cra_init(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
			 sizeof(struct tegra_virtual_se_aes_cmac_context));

	return 0;
}

static void tegra_hv_vse_cmac_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_aes_cmac_context *ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx = NULL;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int err;
	int time_left;

	if (!ctx)
		return;

	if (!ctx->is_key_slot_allocated)
		return;
	ivc_req_msg = devm_kzalloc(se_dev->dev, sizeof(*ivc_req_msg),
				GFP_KERNEL);
	if (!ivc_req_msg)
		return;

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		devm_kfree(se_dev->dev, ivc_req_msg);
		return;
	}

	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_req_msg->hdr.num_reqs = 1;

	/* Allocate AES key slot */
	ivc_tx->engine = VIRTUAL_SE_AES1;
	ivc_tx->cmd = VIRTUAL_SE_CMD_AES_RELEASE_KEY;
	ivc_tx->args.aes.key.slot = ctx->aes_keyslot;
	priv_data_ptr = (struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
	priv_data_ptr->priv_data = (unsigned int *)priv;
	priv->cmd = VIRTUAL_SE_PROCESS;
	priv->se_dev = se_dev;
	init_completion(&priv->alg_complete);
	vse_thread_start = true;

	mutex_lock(&se_dev->server_lock);
	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended)) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		err = -ENODEV;
		goto free_mem;
	}
	err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t));
	if (err) {
		mutex_unlock(&se_dev->server_lock);
		devm_kfree(se_dev->dev, priv);
		goto free_mem;
	}

	time_left = wait_for_completion_timeout(&priv->alg_complete,
			TEGRA_HV_VSE_TIMEOUT);
	mutex_unlock(&se_dev->server_lock);
	if (time_left == 0)
		dev_err(se_dev->dev, "cmac_final timeout\n");

	devm_kfree(se_dev->dev, priv);

free_mem:
	devm_kfree(se_dev->dev, ivc_req_msg);
	ctx->is_key_slot_allocated = false;
}

static int tegra_hv_vse_rng_drbg_init(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES0];

	/* Return error if engine is in suspended state */
	if (atomic_read(&se_dev->se_suspended))
		return -ENODEV;

	rng_ctx->se_dev = se_dev;
	rng_ctx->rng_buf =
		dma_alloc_coherent(rng_ctx->se_dev->dev,
		TEGRA_VIRTUAL_SE_RNG_DT_SIZE,
		&rng_ctx->rng_buf_adr, GFP_KERNEL);
	if (!rng_ctx->rng_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		return -ENOMEM;
	}

	return 0;
}

static void tegra_hv_vse_rng_drbg_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);

	if (rng_ctx->rng_buf) {
		dma_free_coherent(rng_ctx->se_dev->dev,
			TEGRA_VIRTUAL_SE_RNG_DT_SIZE, rng_ctx->rng_buf,
			rng_ctx->rng_buf_adr);
	}
	rng_ctx->se_dev = NULL;
}

static int tegra_hv_vse_rng_drbg_get_random(struct crypto_rng *tfm,
	const u8 *src, unsigned int slen, u8 *rdata, unsigned int dlen)
{
	struct tegra_virtual_se_rng_context *rng_ctx = crypto_rng_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = rng_ctx->se_dev;
	u8 *rdata_addr;
	int err = 0, j, num_blocks, data_len = 0;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int time_left;

	num_blocks = (dlen / TEGRA_VIRTUAL_SE_RNG_DT_SIZE);
	data_len = (dlen % TEGRA_VIRTUAL_SE_RNG_DT_SIZE);
	if (data_len == 0)
		num_blocks = num_blocks - 1;

	ivc_req_msg = devm_kzalloc(se_dev->dev,
				sizeof(*ivc_req_msg),
				GFP_KERNEL);
	if (!ivc_req_msg)
		return 0;

	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_req_msg->hdr.num_reqs = 1;
	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(se_dev->dev, "Priv Data allocation failed\n");
		devm_kfree(se_dev->dev, ivc_req_msg);
		return 0;
	}

	for (j = 0; j <= num_blocks; j++) {
		ivc_tx->engine = VIRTUAL_SE_AES0;
		ivc_tx->cmd = VIRTUAL_SE_CMD_AES_RNG_DBRG;
		ivc_tx->args.aes.op_rng.streamid = se_dev->stream_id;
		ivc_tx->args.aes.op_rng.data_length =
			TEGRA_VIRTUAL_SE_RNG_DT_SIZE;
		ivc_tx->args.aes.op_rng.dst = rng_ctx->rng_buf_adr;
		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_PROCESS;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);
		vse_thread_start = true;

		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			dlen = 0;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			dlen = 0;
			goto exit;
		}

		time_left = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if (time_left == 0) {
			dev_err(se_dev->dev, "%s timeout\n", __func__);
			dlen = 0;
			goto exit;
		}

		rdata_addr =
			(rdata + (j * TEGRA_VIRTUAL_SE_RNG_DT_SIZE));
		if (data_len && num_blocks == j) {
			memcpy(rdata_addr, rng_ctx->rng_buf, data_len);
		} else {
			memcpy(rdata_addr,
				rng_ctx->rng_buf,
				TEGRA_VIRTUAL_SE_RNG_DT_SIZE);
		}
	}
exit:
	devm_kfree(se_dev->dev, priv);
	devm_kfree(se_dev->dev, ivc_req_msg);
	return dlen;
}

static int tegra_hv_vse_rng_drbg_reset(struct crypto_rng *tfm,
	const u8 *seed, unsigned int slen)
{
	return 0;
}

static int tegra_hv_vse_rng1_get_trng(struct crypto_rng *tfm,
		const u8 *src, unsigned int slen,
		u8 *rdata, unsigned int dlen)
{
	struct tegra_virtual_se_rng1_trng_ctx *rng_ctx = crypto_rng_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = rng_ctx->se_dev;
	u8 *rdata_addr;
	int err = 0, j, num_blocks, data_len = 0;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	int time_left, total;

	num_blocks = (dlen / TEGRA_VIRTUAL_SE_RNG1_SIZE);
	data_len = (dlen % TEGRA_VIRTUAL_SE_RNG1_SIZE);
	if (data_len == 0)
		num_blocks = num_blocks - 1;
	total = dlen;
	ivc_req_msg = devm_kzalloc(se_dev->dev,
				sizeof(*ivc_req_msg),
				GFP_KERNEL);
	if (!ivc_req_msg)
		return 0;

	ivc_tx = &ivc_req_msg->d[0].tx;
	ivc_req_msg->hdr.num_reqs = 1;
	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(se_dev->dev, "Priv Data allocation failed\n");
		devm_kfree(se_dev->dev, ivc_req_msg);
		return 0;
	}

	for (j = 0; j <= num_blocks; j++) {
		ivc_tx->engine = VIRTUAL_SE_RNG1;
		ivc_tx->cmd = VIRTUAL_SE_CMD_ELP_RNG1_TRNG;

		if (total < TEGRA_VIRTUAL_SE_RNG1_SIZE)
			ivc_tx->args.rng1.length = total;
		else
			ivc_tx->args.rng1.length =
				TEGRA_VIRTUAL_SE_RNG1_SIZE;

		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_RNG1_PROCESS;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);
		vse_thread_start = true;

		mutex_lock(&se_dev->server_lock);
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			dlen = 0;
			goto exit;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			dlen = 0;
			goto exit;
		}

		time_left = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		mutex_unlock(&se_dev->server_lock);
		if ((time_left == 0) || (priv->rng1.status)) {
			dev_err(se_dev->dev, "%s rng1 failed\n", __func__);
			dlen = 0;
			goto exit;
		}
		rdata_addr =
			(rdata + (j * TEGRA_VIRTUAL_SE_RNG1_SIZE));
		if (data_len && num_blocks == j) {
			memcpy(rdata_addr, priv->rng1.data, data_len);
		} else {
			memcpy(rdata_addr,
				priv->rng1.data,
				TEGRA_VIRTUAL_SE_RNG1_SIZE);
		}
		total -= TEGRA_VIRTUAL_SE_RNG1_SIZE;
	}
exit:
	devm_kfree(se_dev->dev, priv);
	devm_kfree(se_dev->dev, ivc_req_msg);
	return dlen;
}

static int egra_hv_vse_rng1_trng_seed(struct crypto_rng *tfm, const u8 *seed,
		unsigned int slen)
{
	return 0;
}

static int tegra_hv_vse_rng1_trng_init(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_rng1_trng_ctx *rng1_ctx =
			crypto_tfm_ctx(tfm);
	rng1_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_RNG1];

	/* Return error if engine is in suspended state */
	if (atomic_read(&rng1_ctx->se_dev->se_suspended))
		return -ENODEV;

	return 0;
}

static void tegra_hv_vse_rng1_trng_exit(struct crypto_tfm *tfm)
{
}

static int tegra_hv_vse_aes_setkey(struct crypto_ablkcipher *tfm,
	const u8 *key, u32 keylen)
{
	struct tegra_virtual_se_aes_context *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_msg_t *ivc_req_msg = NULL;
	struct tegra_virtual_se_ivc_tx_msg_t *ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;
	struct tegra_vse_priv_data *priv = NULL;
	struct tegra_vse_tag *priv_data_ptr;
	s8 label[VIRTUAL_SE_AES_MAX_KEY_SIZE];
	s32 slot;

	if (!ctx)
		return -EINVAL;

	/* format: 'NVSEAES 1234567\0' */
	if (!se_dev->disable_keyslot_label) {
		bool is_keyslot_label = strnlen(key, keylen) < keylen &&
			sscanf(key, "%s %d", label, &slot) == 2 &&
			!strcmp(label, TEGRA_VIRTUAL_SE_AES_KEYSLOT_LABEL);

		if (is_keyslot_label) {
			if (slot < 0 || slot > 15) {
				dev_err(se_dev->dev,
					"\n Invalid keyslot: %u\n", slot);
				return -EINVAL;
			}
			ctx->keylen = keylen;
			ctx->aes_keyslot = (u32)slot;
			ctx->is_key_slot_allocated = true;
			ctx->is_keyslot_label = true;
			return tegra_hv_vse_aes_set_keyiv(se_dev, (u8 *)key,
					keylen, slot, AES_KEYTBL_TYPE_KEY);
		}
	}

	priv = devm_kzalloc(se_dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (!ctx->is_key_slot_allocated) {
		ivc_req_msg = devm_kzalloc(se_dev->dev,
				sizeof(struct tegra_virtual_se_ivc_msg_t),
				GFP_KERNEL);
		if (!ivc_req_msg) {
			dev_err(se_dev->dev, "\n Memory allocation failed\n");
			devm_kfree(se_dev->dev, priv);
			return -ENOMEM;
		}

		/* Allocate AES key slot */
		ivc_req_msg->hdr.num_reqs = 1;
		ivc_tx = &ivc_req_msg->d[0].tx;
		ivc_tx->engine = VIRTUAL_SE_AES1;
		ivc_tx->cmd = VIRTUAL_SE_CMD_AES_ALLOC_KEY;

		priv_data_ptr =
			(struct tegra_vse_tag *)ivc_req_msg->hdr.tag;
		priv_data_ptr->priv_data = (unsigned int *)priv;
		priv->cmd = VIRTUAL_SE_KEY_SLOT;
		priv->se_dev = se_dev;
		init_completion(&priv->alg_complete);
		vse_thread_start = true;

		mutex_lock(&se_dev->server_lock);
		/* Return error if engine is in suspended state */
		if (atomic_read(&se_dev->se_suspended)) {
			mutex_unlock(&se_dev->server_lock);
			devm_kfree(se_dev->dev, priv);
			err = -ENODEV;
			goto free_mem;
		}
		err = tegra_hv_vse_send_ivc(se_dev, pivck, ivc_req_msg,
				sizeof(struct tegra_virtual_se_ivc_msg_t));
		if (err) {
			mutex_unlock(&se_dev->server_lock);
			devm_kfree(se_dev->dev, priv);
			goto free_mem;
		}

		err = wait_for_completion_timeout(&priv->alg_complete,
				TEGRA_HV_VSE_TIMEOUT);
		if (err == 0) {
			mutex_unlock(&se_dev->server_lock);
			devm_kfree(se_dev->dev, priv);
			dev_err(se_dev->dev, "%s timeout\n", __func__);
			err = -ETIMEDOUT;
			goto free_mem;
		}
		mutex_unlock(&se_dev->server_lock);
		ctx->aes_keyslot = priv->slot_num;
		ctx->is_key_slot_allocated = true;
	}
	devm_kfree(se_dev->dev, priv);

	ctx->keylen = keylen;
	err = tegra_hv_vse_aes_set_keyiv(se_dev, (u8 *)key, keylen,
			ctx->aes_keyslot, AES_KEYTBL_TYPE_KEY);

free_mem:
	if (ivc_req_msg)
		devm_kfree(se_dev->dev, ivc_req_msg);
	return err;
}

static struct crypto_alg aes_algs[] = {
	{
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-tegra",
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_cbc_encrypt,
			.decrypt = tegra_hv_vse_aes_cbc_decrypt,
		}
	}, {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-tegra",
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ecb_encrypt,
			.decrypt = tegra_hv_vse_aes_ecb_decrypt,
		}
	}, {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-tegra",
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ctr_encrypt,
			.decrypt = tegra_hv_vse_aes_ctr_decrypt,
			.geniv = "eseqiv",
		}
	}, {
		.cra_name = "ofb(aes)",
		.cra_driver_name = "ofb-aes-tegra",
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ofb_encrypt,
			.decrypt = tegra_hv_vse_aes_ofb_decrypt,
				.geniv = "eseqiv",
		}
	},
};

static struct ahash_alg cmac_alg = {
	.init = tegra_hv_vse_cmac_init,
	.update = tegra_hv_vse_cmac_update,
	.final = tegra_hv_vse_cmac_final,
	.finup = tegra_hv_vse_cmac_finup,
	.digest = tegra_hv_vse_cmac_digest,
	.setkey = tegra_hv_vse_cmac_setkey,
	.halg.digestsize = TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE,
	.halg.statesize = TEGRA_VIRTUAL_SE_AES_CMAC_STATE_SIZE,
	.halg.base = {
		.cra_name = "cmac(aes)",
		.cra_driver_name = "tegra-hv-vse-cmac(aes)",
		.cra_priority = 400,
		.cra_flags = CRYPTO_ALG_TYPE_AHASH,
		.cra_blocksize = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_cmac_context),
		.cra_alignmask = 0,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_cmac_cra_init,
		.cra_exit = tegra_hv_vse_cmac_cra_exit,
	}
};

static struct rng_alg rng_alg[] = {
	{
		.generate	= tegra_hv_vse_rng_drbg_get_random,
		.seed		= tegra_hv_vse_rng_drbg_reset,
		.seedsize	= TEGRA_VIRTUAL_SE_RNG_SEED_SIZE,
		.base		= {
			.cra_name = "rng_drbg",
			.cra_driver_name = "rng_drbg-aes-tegra",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_RNG,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rng_context),
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rng_drbg_init,
			.cra_exit = tegra_hv_vse_rng_drbg_exit,
		}
	}
};

static struct rng_alg rng1_trng_alg[] = {
	{
		.generate = tegra_hv_vse_rng1_get_trng,
		.seed = egra_hv_vse_rng1_trng_seed,
		.base = {
			.cra_name = "rng1_trng",
			.cra_driver_name = "rng1-elp-tegra",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_RNG,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rng1_trng_ctx),
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rng1_trng_init,
			.cra_exit = tegra_hv_vse_rng1_trng_exit,
		}
	}
};

static struct ahash_alg sha_algs[] = {
	{
		.init = tegra_hv_vse_sha_init,
		.update = tegra_hv_vse_sha_update,
		.final = tegra_hv_vse_sha_final,
		.finup = tegra_hv_vse_sha_finup,
		.digest = tegra_hv_vse_sha_digest,
		.export = tegra_hv_vse_sha_export,
		.import = tegra_hv_vse_sha_import,
		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.statesize = sizeof(struct tegra_virtual_se_req_context),
		.halg.base = {
			.cra_name = "sha1",
			.cra_driver_name = "tegra-hv-vse-sha1",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_sha_cra_init,
			.cra_exit = tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_sha_init,
		.update = tegra_hv_vse_sha_update,
		.final = tegra_hv_vse_sha_final,
		.finup = tegra_hv_vse_sha_finup,
		.digest = tegra_hv_vse_sha_digest,
		.export = tegra_hv_vse_sha_export,
		.import = tegra_hv_vse_sha_import,
		.halg.digestsize = SHA224_DIGEST_SIZE,
		.halg.statesize = sizeof(struct tegra_virtual_se_req_context),
		.halg.base = {
			.cra_name = "sha224",
			.cra_driver_name = "tegra-hv-vse-sha224",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA224_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_sha_cra_init,
			.cra_exit = tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_sha_init,
		.update = tegra_hv_vse_sha_update,
		.final = tegra_hv_vse_sha_final,
		.finup = tegra_hv_vse_sha_finup,
		.digest = tegra_hv_vse_sha_digest,
		.export = tegra_hv_vse_sha_export,
		.import = tegra_hv_vse_sha_import,
		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.statesize = sizeof(struct tegra_virtual_se_req_context),
		.halg.base = {
			.cra_name = "sha256",
			.cra_driver_name = "tegra-hv-vse-sha256",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_sha_cra_init,
			.cra_exit = tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_sha_init,
		.update = tegra_hv_vse_sha_update,
		.final = tegra_hv_vse_sha_final,
		.finup = tegra_hv_vse_sha_finup,
		.digest = tegra_hv_vse_sha_digest,
		.export = tegra_hv_vse_sha_export,
		.import = tegra_hv_vse_sha_import,
		.halg.digestsize = SHA384_DIGEST_SIZE,
		.halg.statesize = sizeof(struct tegra_virtual_se_req_context),
		.halg.base = {
			.cra_name = "sha384",
			.cra_driver_name = "tegra-hv-vse-sha384",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA384_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_sha_cra_init,
			.cra_exit = tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_sha_init,
		.update = tegra_hv_vse_sha_update,
		.final = tegra_hv_vse_sha_final,
		.finup = tegra_hv_vse_sha_finup,
		.digest = tegra_hv_vse_sha_digest,
		.export = tegra_hv_vse_sha_export,
		.import = tegra_hv_vse_sha_import,
		.halg.digestsize = SHA512_DIGEST_SIZE,
		.halg.statesize = sizeof(struct tegra_virtual_se_req_context),
		.halg.base = {
			.cra_name = "sha512",
			.cra_driver_name = "tegra-hv-vse-sha512",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_sha_cra_init,
			.cra_exit = tegra_hv_vse_sha_cra_exit,
		}
	},
};

static struct ahash_alg rsa_algs[] = {
	{
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE,
		.halg.statesize = TEGRA_VIRTUAL_SE_RSA512_STATE_SIZE,
		.halg.base = {
			.cra_name = "rsa512",
			.cra_driver_name = "tegra-hv-vse-rsa512",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE,
		.halg.statesize = TEGRA_VIRTUAL_SE_RSA1024_STATE_SIZE,
		.halg.base = {
			.cra_name = "rsa1024",
			.cra_driver_name = "tegra-hv-vse-rsa1024",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE,
		.halg.statesize = TEGRA_VIRTUAL_SE_RSA1536_STATE_SIZE,
		.halg.base = {
			.cra_name = "rsa1536",
			.cra_driver_name = "tegra-hv-vse-rsa1536",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE,
		.halg.statesize = TEGRA_VIRTUAL_SE_RSA2048_STATE_SIZE,
		.halg.base = {
			.cra_name = "rsa2048",
			.cra_driver_name = "tegra-hv-vse-rsa2048",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}
};

static struct of_device_id tegra_hv_vse_of_match[] = {
	{
		.compatible = "nvidia,tegra186-hv-vse",
	}, {}
};
MODULE_DEVICE_TABLE(of, tegra_hv_vse_of_match);

static irqreturn_t tegra_vse_irq_handler(int irq, void *data)
{
	complete(&tegra_vse_complete);
	return IRQ_HANDLED;
}

static int tegra_vse_kthread(void *unused)
{
	struct tegra_virtual_se_dev *se_dev = NULL;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_msg_t *ivc_resp_msg;
	int err = 0;
	struct tegra_vse_tag *p_dat;
	struct tegra_vse_priv_data *priv;
	int timeout;
	struct tegra_virtual_se_ivc_resp_msg_t *ivc_rx;
	struct tegra_virtual_se_ivc_rng *rng_res;
	int ret;
	int read_size = 0;
	u32 len_32;

	ivc_resp_msg =
		kmalloc(sizeof(struct tegra_virtual_se_ivc_msg_t), GFP_KERNEL);
	if (!ivc_resp_msg)
		return -ENOMEM;

	disable_irq(g_ivck->irq);
	while (!kthread_should_stop()) {
		enable_irq(g_ivck->irq);
		err = 0;
		ret = wait_for_completion_interruptible(&tegra_vse_complete);
		disable_irq(g_ivck->irq);
		if (ret < 0) {
			pr_err("%s completion err\n", __func__);
			reinit_completion(&tegra_vse_complete);
			continue;
		}

		if (!vse_thread_start) {
			reinit_completion(&tegra_vse_complete);
			continue;
		}
		timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;
		while (tegra_hv_ivc_channel_notified(pivck) != 0) {
			if (!timeout) {
				reinit_completion(
					&tegra_vse_complete);
				pr_err("%s:%d ivc channel_notifier timeout\n",
					__func__, __LINE__);
				err = -EAGAIN;
				break;
			}
			udelay(1);
			timeout--;
		}

		if (err == -EAGAIN) {
			err = 0;
			continue;
		}

		if (!tegra_hv_ivc_can_read(pivck)) {
			reinit_completion(&tegra_vse_complete);
			continue;
		}
		while ((read_size = tegra_hv_ivc_read(pivck,
			ivc_resp_msg,
			sizeof(struct tegra_virtual_se_ivc_msg_t))) > 0) {
			if (read_size < TEGRA_HV_VSE_IVC_FRAME_SIZE) {
				dev_err(se_dev->dev,
					"Wrong read msg len %d\n", read_size);
				continue;
			}
			p_dat =
				(struct tegra_vse_tag *)ivc_resp_msg->hdr.tag;
			priv = (struct tegra_vse_priv_data *)p_dat->priv_data;
			if (!priv) {
				pr_err("%s no call back info\n", __func__);
				continue;
			}
			se_dev = priv->se_dev;

			switch (priv->cmd) {
			case VIRTUAL_SE_AES_CRYPTO:
				priv->rx_status =
					(s8)ivc_resp_msg->d[0].rx.status;
				priv->call_back_vse(priv);
				atomic_sub(1, &se_dev->ivc_count);
				devm_kfree(se_dev->dev, priv);
				break;
			case VIRTUAL_SE_KEY_SLOT:
				ivc_rx = &ivc_resp_msg->d[0].rx;
				priv->slot_num = ivc_rx->keyslot;
				complete(&priv->alg_complete);
				break;
			case VIRTUAL_SE_PROCESS:
				complete(&priv->alg_complete);
				break;
			case VIRTUAL_RNG1_PROCESS:
				rng_res = (struct tegra_virtual_se_ivc_rng *)&ivc_resp_msg->d[0].rx;
				len_32 = *(u32 *)&rng_res->data_len[0];
				priv->rng1.status = rng_res->status;
				priv->rng1.bytes_returned = len_32;
				/* Copy rng data if status is success */
				if (!rng_res->status) {
					if (len_32 > TEGRA_VIRTUAL_SE_RNG1_SIZE)
						len_32 = TEGRA_VIRTUAL_SE_RNG1_SIZE;
					memcpy(priv->rng1.data, rng_res->data,
						len_32);
				}
				complete(&priv->alg_complete);
				break;
			default:
				dev_err(se_dev->dev, "Unknown command\n");
			}
		}
		reinit_completion(&tegra_vse_complete);
	}
	kfree(ivc_resp_msg);
	return 0;
}

static int tegra_hv_vse_probe(struct platform_device *pdev)
{
	struct tegra_virtual_se_dev *se_dev = NULL;
	int err = 0;
	int i;
	unsigned int ivc_id;
	unsigned int engine_id;

	se_dev = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_virtual_se_dev),
				GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	se_dev->dev = &pdev->dev;
	err = of_property_read_u32(pdev->dev.of_node, "se-engine-id",
				&engine_id);
	if (err) {
		dev_err(&pdev->dev, "se-engine-id property not present\n");
		err = -ENODEV;
		goto exit;
	}

	if (engine_id != VIRTUAL_SE_RNG1) {
		se_dev->stream_id = iommu_get_hwid(pdev->dev.archdata.iommu,
				&pdev->dev, 0);
		dev_info(se_dev->dev, "Virtual SE Stream ID: %d",
			se_dev->stream_id);
	}

	if (!g_ivck) {
		err = of_property_read_u32(pdev->dev.of_node, "ivc", &ivc_id);
		if (err) {
			dev_err(&pdev->dev, "ivc property not present\n");
			err = -ENODEV;
			goto exit;
		}
		dev_info(se_dev->dev, "Virtual SE channel number: %d", ivc_id);

		g_ivck = tegra_hv_ivc_reserve(NULL, ivc_id, NULL);
		if (IS_ERR_OR_NULL(g_ivck)) {
			dev_err(&pdev->dev, "Failed reserve channel number\n");
			err = -ENODEV;
			goto exit;
		}
		tegra_hv_ivc_channel_reset(g_ivck);
		init_completion(&tegra_vse_complete);

		tegra_vse_task = kthread_run(tegra_vse_kthread,
				NULL, "tegra_vse_kthread");
		if (IS_ERR(tegra_vse_task)) {
			dev_err(se_dev->dev,
				"Couldn't create kthread for vse\n");
			err = PTR_ERR(tegra_vse_task);
			goto exit;
		}

		if (request_irq(g_ivck->irq,
			tegra_vse_irq_handler, 0, "vse", se_dev)) {
			dev_err(se_dev->dev,
				"Failed to request irq %d\n", g_ivck->irq);
			err = -EINVAL;
			goto exit;
		}
	}

	if (of_property_read_bool(pdev->dev.of_node, "disable-keyslot-label"))
		se_dev->disable_keyslot_label = true;

	g_virtual_se_dev[engine_id] = se_dev;
	mutex_init(&se_dev->mtx);

	if (engine_id == VIRTUAL_SE_AES0) {
		err = crypto_register_rng(&rng_alg[0]);
		if (err) {
			dev_err(&pdev->dev,
				"rng alg register failed. Err %d\n", err);
			goto exit;
		}
	}

	if (engine_id == VIRTUAL_SE_AES1) {
		INIT_WORK(&se_dev->se_work, tegra_hv_vse_work_handler);
		crypto_init_queue(&se_dev->queue,
			TEGRA_HV_VSE_CRYPTO_QUEUE_LENGTH);
		spin_lock_init(&se_dev->lock);
		se_dev->vse_work_q = alloc_workqueue("vse_work_q",
			WQ_HIGHPRI | WQ_UNBOUND, 1);

		if (!se_dev->vse_work_q) {
			err = -ENOMEM;
			dev_err(se_dev->dev, "alloc_workqueue failed\n");
			goto exit;
		}
		for (i = 0; i < ARRAY_SIZE(aes_algs); i++) {
			err = crypto_register_alg(&aes_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"aes alg register failed idx[%d]\n", i);
				goto exit;
			}
		}

		err = crypto_register_ahash(&cmac_alg);
		if (err) {
			dev_err(&pdev->dev,
				"cmac alg register failed. Err %d\n", err);
			goto exit;
		}
		atomic_set(&se_dev->ivc_count, 0);
	}

	if (engine_id == VIRTUAL_SE_SHA) {
		for (i = 0; i < ARRAY_SIZE(sha_algs); i++) {
			err = crypto_register_ahash(&sha_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"sha alg register failed idx[%d]\n", i);
				goto exit;
			}
		}
	}

	if (engine_id == VIRTUAL_SE_RSA) {
		for (i = 0; i < ARRAY_SIZE(rsa_algs); i++) {
			err = crypto_register_ahash(&rsa_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"RSA alg register failed idx[%d]\n", i);
				goto exit;
			}
		}
	}

	if (engine_id == VIRTUAL_SE_RNG1) {
		err = crypto_register_rng(&rng1_trng_alg[0]);
		if (err) {
			dev_err(&pdev->dev,
				"rng1 alg register failed. Err %d\n", err);
			goto exit;
		}
		is_rng1_registered = true;
	}
	se_dev->engine_id = engine_id;

	/* Set Engine suspended state to false*/
	atomic_set(&se_dev->se_suspended, 0);
	platform_set_drvdata(pdev, se_dev);
	mutex_init(&se_dev->server_lock);

	return 0;

exit:
	return err;
}

static void tegra_hv_vse_shutdown(struct platform_device *pdev)
{
	struct tegra_virtual_se_dev *se_dev = platform_get_drvdata(pdev);

	/* Set engine to suspend state */
	atomic_set(&se_dev->se_suspended, 1);

	if (se_dev->engine_id == VIRTUAL_SE_AES1) {
		/* Make sure to complete pending async requests */
		flush_workqueue(se_dev->vse_work_q);

		/* Make sure that there are no pending tasks with SE server */
		while (atomic_read(&se_dev->ivc_count) != 0)
			usleep_range(8, 10);
	}

	/* Wait for  SE server to be free*/
	while (mutex_is_locked(&se_dev->server_lock))
		usleep_range(8, 10);
}

static int tegra_hv_vse_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sha_algs); i++)
		crypto_unregister_ahash(&sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(rsa_algs); i++)
		crypto_unregister_ahash(&rsa_algs[i]);

	if (is_rng1_registered)
		crypto_unregister_rng(&rng1_trng_alg[0]);

	return 0;
}

#if defined(CONFIG_PM)
static int tegra_hv_vse_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	/* Keep engine in suspended state */
	tegra_hv_vse_shutdown(pdev);
	return 0;
}

static int tegra_hv_vse_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_virtual_se_dev *se_dev = platform_get_drvdata(pdev);

	/* Set engine to suspend state to 1 to make it as false */
	atomic_set(&se_dev->se_suspended, 0);

	return 0;
}

static const struct dev_pm_ops tegra_hv_pm_ops = {
	.suspend = tegra_hv_vse_suspend,
	.resume = tegra_hv_vse_resume,
};
#endif /* CONFIG_PM */

static struct platform_driver tegra_hv_vse_driver = {
	.probe = tegra_hv_vse_probe,
	.remove = tegra_hv_vse_remove,
	.shutdown = tegra_hv_vse_shutdown,
	.driver = {
		.name = "tegra_hv_vse",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_vse_of_match),
#if defined(CONFIG_PM)
		.pm = &tegra_hv_pm_ops,
#endif
	},
};

static int __init tegra_hv_vse_module_init(void)
{
	return platform_driver_register(&tegra_hv_vse_driver);
}

static void __exit tegra_hv_vse_module_exit(void)
{
	platform_driver_unregister(&tegra_hv_vse_driver);
}

module_init(tegra_hv_vse_module_init);
module_exit(tegra_hv_vse_module_exit);

MODULE_AUTHOR("Mallikarjun Kasoju <mkasoju@nvidia.com>");
MODULE_DESCRIPTION("Virtual Security Engine driver over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");
