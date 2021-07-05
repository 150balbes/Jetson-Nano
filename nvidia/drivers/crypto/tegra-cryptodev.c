/*
 * drivers/misc/tegra-cryptodev.c
 *
 * crypto dev node for NVIDIA tegra aes hardware
 *
 * Copyright (c) 2010-2019, NVIDIA Corporation. All Rights Reserved.
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
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/nospec.h>
#include <soc/tegra/chip-id.h>
#include <crypto/rng.h>
#include <crypto/hash.h>
#include <linux/platform/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <crypto/akcipher.h>
#include <crypto/internal/skcipher.h>

#include <uapi/misc/tegra-cryptodev.h>
#include <asm/barrier.h>

#define NBUFS 2
#define XBUFSIZE 8
#define RNG_DRBG 1
#define RNG 0
#define NUM_RSA_ALGO 4
#define ECC_MODE_MIN_INDEX 7
#define ECC_MODE_MAX_INDEX 13
#define MAX_RSA_MSG_LEN 256
#define MAX_RSA1_MSG_LEN 512

enum tegra_se_pka1_ecc_type {
	ECC_POINT_MUL,
	ECC_POINT_ADD,
	ECC_POINT_DOUBLE,
	ECC_POINT_VER,
	ECC_SHAMIR_TRICK,
	ECC_INVALID,
};

struct tegra_crypto_ctx {
	/*ecb, cbc, ofb, ctr */
	struct crypto_skcipher *aes_tfm[TEGRA_CRYPTO_MAX];
	/* rsa512, rsa1024, rsa1536, rsa2048 */
	struct crypto_akcipher *rsa_tfm[4];
	/* rsa512, rsa768, rsa1024, rsa1536, rsa2048, rsa3072, rsa4096 */
	struct crypto_akcipher *pka1_rsa_tfm;
	/* sha1, sha224, sha256, sha384, sha512, cmac */
	struct crypto_ahash *sha_tfm[6];
	struct crypto_shash *shash_tfm[6];
	struct crypto_rng *rng;
	struct crypto_rng *rng_drbg;
	u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	int use_ssk;
	bool skip_exit;
};

struct tegra_crypto_completion {
	struct completion restart;
	int req_err;
};

static inline unsigned int crypto_shash_reqsize(struct crypto_shash *stfm)
{
	return stfm->descsize;
}

static inline void shash_request_set_tfm(struct shash_desc *desc,
					 struct crypto_shash *stfm)
{
	desc->tfm = (struct crypto_shash *)crypto_shash_tfm(stfm);
}

static inline struct shash_desc *shash_request_alloc(
	struct crypto_shash *stfm, gfp_t gfp)
{
	struct shash_desc *desc;

	desc = kmalloc(sizeof(struct shash_desc) +
		      crypto_shash_reqsize(stfm), gfp);

	if (likely(desc))
		shash_request_set_tfm(desc, stfm);

	return desc;
}

static inline void shash_request_free(struct shash_desc *desc)
{
	kzfree(desc);
}

static int alloc_bufs(unsigned long *buf[NBUFS])
{
	int i;

	for (i = 0; i < NBUFS; i++) {
		buf[i] = (void *)__get_free_page(GFP_KERNEL);
		if (!buf[i])
			goto err_free_buf;
	}

	return 0;

err_free_buf:
	while (i-- > 0)
		free_page((unsigned long)buf[i]);

	return -ENOMEM;
}

static void free_bufs(unsigned long *buf[NBUFS])
{
	int i;

	for (i = 0; i < NBUFS; i++)
		free_page((unsigned long)buf[i]);
}

static int tegra_crypto_dev_open(struct inode *inode, struct file *filp)
{
	struct tegra_crypto_ctx *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(struct tegra_crypto_ctx), GFP_KERNEL);
	if (!ctx) {
		pr_err("no memory for context\n");
		return -ENOMEM;
	}

	/* CBC tfm is allocated during device_open itself
	 * for (LP0) CTX_SAVE test that is performed using CBC
	 */
	ctx->aes_tfm[TEGRA_CRYPTO_CBC] =
		crypto_alloc_skcipher("cbc-aes-tegra",
		CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC, 0);
	if (IS_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC])) {
		pr_err("Failed to load transform for cbc-aes-tegra: %ld\n",
			PTR_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC]));
		ret = PTR_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC]);
		kfree(ctx);
		return ret;
	}
	filp->private_data = ctx;
	return ret;
}

static int tegra_crypto_dev_release(struct inode *inode, struct file *filp)
{
	struct tegra_crypto_ctx *ctx = filp->private_data;
	int i = 0;
	static int tfm_index;
	int ret = 0;

	/* store_tfm is needed to store the tfms in order to free them
	 * later when skip_exit becomes false
	 */
	static struct crypto_skcipher *store_tfm[
					TEGRA_CRYPTO_AES_TEST_KEYSLOTS];

	/* Only when skip_exit is false, the concerned tfm is freed,
	 * else it is just saved in store_tfm that is freed later
	 */
	if (tfm_index >= TEGRA_CRYPTO_AES_TEST_KEYSLOTS) {
		pr_err("Invalid key slot index usage: %d\n", tfm_index);
		ret = -ERANGE;
		goto out;
	}

	if (ctx->aes_tfm[TEGRA_CRYPTO_CBC] && ctx->skip_exit)
		store_tfm[tfm_index++] = ctx->aes_tfm[TEGRA_CRYPTO_CBC];

	if (ctx->aes_tfm[TEGRA_CRYPTO_CBC] && !ctx->skip_exit) {
		crypto_free_skcipher(ctx->aes_tfm[TEGRA_CRYPTO_CBC]);

		for (i = tfm_index - 1; i >= 0; i--)
			crypto_free_skcipher(store_tfm[i]);
		tfm_index = 0;
	}
out:
	kfree(ctx);
	filp->private_data = NULL;

	return ret;
}

static void tegra_crypt_complete(struct crypto_async_request *req, int err)
{
	struct tegra_crypto_completion *done = req->data;

	if (err != -EINPROGRESS) {
		done->req_err = err;
		complete(&done->restart);
	}
}

static int process_crypt_req(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_crypt_req *crypt_req)
{
	struct crypto_skcipher *tfm;
	struct skcipher_request *req = NULL;
	struct scatterlist in_sg;
	struct scatterlist out_sg;
	unsigned long *xbuf[NBUFS];
	int ret = 0, size = 0;
	unsigned long total = 0;
	const u8 *key = NULL;
	struct tegra_crypto_completion tcrypt_complete;
	char aes_algo[5][10] = {"ecb(aes)", "cbc(aes)", "ofb(aes)", "ctr(aes)",
				"xts(aes)"};
	const char *algo;

	if (crypt_req->op != TEGRA_CRYPTO_CBC) {
		if (crypt_req->op >= TEGRA_CRYPTO_MAX)
			return -EINVAL;

		crypt_req->op = array_index_nospec(crypt_req->op,
							TEGRA_CRYPTO_MAX);

		tfm = crypto_alloc_skcipher(aes_algo[crypt_req->op],
			CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for %s: %ld\n",
				aes_algo[crypt_req->op], PTR_ERR(tfm));
			ret = PTR_ERR(tfm);
			goto out;
		}

		ctx->aes_tfm[crypt_req->op] = tfm;
		filp->private_data = ctx;
	} else {
		tfm = ctx->aes_tfm[TEGRA_CRYPTO_CBC];
		ctx->skip_exit = crypt_req->skip_exit;
		filp->private_data = ctx;
	}

	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("%s: Failed to allocate request\n", __func__);
		ret = -ENOMEM;
		goto free_tfm;
	}

	if (((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_128_SIZE) &&
		((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_192_SIZE) &&
		((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_256_SIZE) &&
		((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_512_SIZE)) {
		ret = -EINVAL;
		pr_err("crypt_req keylen invalid");
		goto process_req_out;
	}

	crypto_skcipher_clear_flags(tfm, ~0);

	if (!ctx->use_ssk)
		key = crypt_req->key;

	if (!crypt_req->skip_key) {
		algo = crypto_tfm_alg_driver_name(crypto_skcipher_tfm(tfm));
		if (!algo) {
			pr_err("Not a avilable algo");
			ret = -EINVAL;
			goto process_req_out;
		}

		/* Null key is only allowed in SE driver */
		if (!strstr(algo, "tegra")) {
			ret = -EINVAL;
			goto process_req_out;
		}

		ret = crypto_skcipher_setkey(tfm, key, crypt_req->keylen);
		if (ret < 0) {
			pr_err("setkey failed");
			goto process_req_out;
		}
	}

	ret = alloc_bufs(xbuf);
	if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto process_req_out;
	}

	init_completion(&tcrypt_complete.restart);

	skcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
		tegra_crypt_complete, &tcrypt_complete);

	total = crypt_req->plaintext_sz;
	while (total > 0) {
		size = min(total, PAGE_SIZE);
		ret = copy_from_user((void *)xbuf[0],
			(void __user *)crypt_req->plaintext, size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
			goto process_req_buf_out;
		}
		sg_init_one(&in_sg, xbuf[0], size);
		sg_init_one(&out_sg, xbuf[1], size);

		if (!crypt_req->skip_iv) {
			skcipher_request_set_crypt(req, &in_sg,
				&out_sg, size, crypt_req->iv);
			/*
			 * Setting IV for the first block only. AES CBC
			 * should use updated IV generated from the last block.
			 * Which is already being maintained by SE.
			 */
			crypt_req->skip_iv = true;
		} else {
			skcipher_request_set_crypt(req, &in_sg,
				&out_sg, size, NULL);
		}

		reinit_completion(&tcrypt_complete.restart);

		tcrypt_complete.req_err = 0;

		ret = crypt_req->encrypt ?
			crypto_skcipher_encrypt(req) :
			crypto_skcipher_decrypt(req);
		if ((ret == -EINPROGRESS) || (ret == -EBUSY)) {
			/* crypto driver is asynchronous */
			ret = wait_for_completion_timeout(&tcrypt_complete.restart,
						msecs_to_jiffies(5000));
			if (ret == 0)
				goto process_req_buf_out;

			if (tcrypt_complete.req_err < 0) {
				ret = tcrypt_complete.req_err;
				goto process_req_buf_out;
			}
		} else if (ret < 0) {
			pr_debug("%scrypt failed (%d)\n",
				crypt_req->encrypt ? "en" : "de", ret);
			goto process_req_buf_out;
		}

		ret = copy_to_user((void __user *)crypt_req->result,
			(const void *)xbuf[1], size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_to_user failed (%d)\n", __func__,
					ret);
			goto process_req_buf_out;
		}

		total -= size;
		crypt_req->result += size;
		crypt_req->plaintext += size;
	}

process_req_buf_out:
	free_bufs(xbuf);
process_req_out:
	skcipher_request_free(req);
free_tfm:
	if (crypt_req->op != TEGRA_CRYPTO_CBC)
		crypto_free_skcipher(tfm);
out:
	return ret;
}

static int wait_async_op(struct tegra_crypto_completion *tr, int ret)
{
	if (ret == -EINPROGRESS || ret == -EBUSY) {
		wait_for_completion(&tr->restart);
		reinit_completion(&tr->restart);
		ret = tr->req_err;
	}
	return ret;
}

static int sha_shash_hash_op(struct shash_desc *desc,
				struct tegra_crypto_completion *tr,
				int ret)
{
	if (ret == -EINPROGRESS || ret == -EBUSY) {
		ret = wait_for_completion_interruptible(&tr->restart);
		if (!ret)
			ret = tr->req_err;
		reinit_completion(&tr->restart);
	}
	return ret;
}

static int tegra_cryptodev_rsa_set_key(struct crypto_akcipher *tfm,
	char *key, unsigned int keylen,
	unsigned int max_rsa_key_len,
	enum tegra_rsa_op_mode op_mode)
{
	unsigned int total_key_len;
	char *key_mem;
	int ret = 0;

	if (!keylen)
		return -EINVAL;

	if ((((keylen >> 16) & 0xFFFF) > max_rsa_key_len) ||
	    ((keylen & 0xFFFF) > max_rsa_key_len)) {
		pr_err("Invalid rsa key length\n");
		return -EINVAL;
	}

	total_key_len = (((keylen >> 16) & 0xFFFF) +
		(keylen & 0xFFFF));
	key_mem = kzalloc(total_key_len, GFP_KERNEL);
	if (!key_mem)
		return -ENOMEM;

	ret = copy_from_user(key_mem, (void __user *)key,
			     total_key_len);
	if (ret) {
		pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
		kfree(key_mem);
		return -EINVAL;
	}

	if (op_mode == RSA_SET_PUB)
		ret = crypto_akcipher_set_pub_key(tfm,
						  key_mem,
						  keylen);
	else
		ret = crypto_akcipher_set_priv_key(tfm,
						   key_mem,
						   keylen);
	kfree(key_mem);

	return ret;
}

static int tegra_crypt_rsa(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_rsa_req *rsa_req)
{
	struct crypto_akcipher *tfm = NULL;
	struct akcipher_request *req;
	struct scatterlist sg[2];
	void *src_buff, *dst_buff;
	int ret = 0;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;

	if (rsa_req->op_mode == RSA_INIT) {
		tfm = crypto_alloc_akcipher("rsa-pka0",
					CRYPTO_ALG_TYPE_AKCIPHER, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for rsa-pka0: %ld\n",
					PTR_ERR(tfm));
			return PTR_ERR(tfm);
		}

		ctx->rsa_tfm[rsa_req->algo] = tfm;
		filp->private_data = ctx;
		return 0;
	} else {
		ctx = filp->private_data;
		tfm =  ctx->rsa_tfm[rsa_req->algo];
	}

	if ((rsa_req->op_mode == RSA_SET_PUB) ||
	    (rsa_req->op_mode == RSA_SET_PRIV)) {
		if (rsa_req->skip_key) {
			pr_err("RSA skip key is set\n");
			return -EINVAL;
		}

		ret = tegra_cryptodev_rsa_set_key(tfm, rsa_req->key,
						  rsa_req->keylen,
						  MAX_RSA_MSG_LEN,
						  rsa_req->op_mode);
		if (ret < 0) {
			pr_err("alg: rsa: set key failed\n");
			return ret;
		}
	} else if (rsa_req->op_mode == RSA_ENCRYPT ||
		rsa_req->op_mode == RSA_DECRYPT	|| rsa_req->op_mode == RSA_SIGN
		|| rsa_req->op_mode == RSA_VERIFY) {
		req = akcipher_request_alloc(tfm, GFP_KERNEL);
		if (!req) {
			pr_err("alg: rsa: Failed to allocate request: %s\n",
					__func__);
			return -ENOMEM;
		}

		ret = alloc_bufs(xbuf);
		if (ret < 0) {
			pr_err("alloc_bufs failed");
			goto buf_fail;
		}

		init_completion(&rsa_complete.restart);
		rsa_complete.req_err = 0;

		src_buff = xbuf[0];
		dst_buff = xbuf[1];

		ret = copy_from_user(src_buff, (void __user *)rsa_req->message,
			rsa_req->msg_len);
		if (ret) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user failed\n", __func__);
			goto rsa_fail;
		}

		memset(dst_buff, 0, rsa_req->msg_len);

		sg_init_one(&sg[0], src_buff, rsa_req->msg_len);
		sg_init_one(&sg[1], dst_buff, rsa_req->msg_len);

		akcipher_request_set_crypt(req, &sg[0], &sg[1],
					rsa_req->msg_len, rsa_req->msg_len);

		if (rsa_req->op_mode == RSA_ENCRYPT) {
			ret = crypto_akcipher_encrypt(req);
			if (ret) {
				pr_err("alg: rsa: encrypt failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_DECRYPT) {
			ret = crypto_akcipher_decrypt(req);
			if (ret) {
				pr_err("alg: rsa: decrypt failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_SIGN) {
			ret = crypto_akcipher_sign(req);
			if (ret) {
				pr_err("alg: rsa: sign failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_VERIFY) {
			ret = crypto_akcipher_verify(req);
			if (ret) {
				pr_err("alg: rsa: verification failed\n");
				goto rsa_fail;
			}
		}

		ret = copy_to_user((void __user *)rsa_req->result,
				(const void *)xbuf[1], rsa_req->msg_len);
		if (ret) {
			ret = -EFAULT;
			pr_err("alg: rsa: copy_to_user failed (%d)\n", ret);
		}
rsa_fail:
		free_bufs(xbuf);
buf_fail:
		akcipher_request_free(req);
	} else if (rsa_req->op_mode == RSA_EXIT) {
		crypto_free_akcipher(tfm);
	} else {
		pr_err("alg: rsa: invalid rsa operation\n");
	}

	return ret;
}

static int tegra_crypt_rsa_ahash(struct file *filp,
				struct tegra_crypto_ctx *ctx,
				struct tegra_rsa_req_ahash *rsa_req_ah)
{
	struct crypto_ahash *tfm = NULL;
	struct ahash_request *req = NULL;
	struct scatterlist sg[1];
	char *result = NULL;
	void *hash_buff;
	int ret = 0;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;
	char rsa_algo[4][10] = {"rsa512", "rsa1024", "rsa1536", "rsa2048"};
	unsigned int total_key_len;
	char *key_mem;

	if ((((rsa_req_ah->keylen >> 16) & 0xFFFF) >
			MAX_RSA_MSG_LEN) ||
		((rsa_req_ah->keylen & 0xFFFF) >
			MAX_RSA_MSG_LEN)) {
		pr_err("Invalid rsa key length\n");
		return -EINVAL;
	}

	total_key_len = (((rsa_req_ah->keylen >> 16) & 0xFFFF) +
				(rsa_req_ah->keylen & 0xFFFF));

	key_mem = kzalloc(total_key_len, GFP_KERNEL);
	if (!key_mem)
		return -ENOMEM;

	ret = copy_from_user(key_mem, (void __user *)rsa_req_ah->key,
			total_key_len);
	if (ret) {
		pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
		kfree(key_mem);
		return -EINVAL;
	}

	rsa_req_ah->key = key_mem;
	tfm = crypto_alloc_ahash(rsa_algo[rsa_req_ah->algo],
					CRYPTO_ALG_TYPE_AHASH, 0);
	if (IS_ERR(tfm)) {
	pr_err("Failed to load transform for %s: %ld\n",
			rsa_algo[rsa_req_ah->algo], PTR_ERR(tfm));
		ret = PTR_ERR(tfm);
		goto out;
	}

	filp->private_data = ctx;

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("alg: hash: Failed to allocate request: %s\n", __func__);
		ret = -ENOMEM;
		goto req_fail;
	}
	ret = alloc_bufs(xbuf);
	 if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto buf_fail;
	}

	init_completion(&rsa_complete.restart);
	rsa_complete.req_err = 0;

	result = kzalloc(rsa_req_ah->keylen >> 16, GFP_KERNEL);
	if (!result)
		goto result_fail;

	hash_buff = xbuf[0];

	ret = copy_from_user(hash_buff, (void __user *)rsa_req_ah->message,
		rsa_req_ah->msg_len);
	if (ret) {
		ret = -EFAULT;
		pr_err("%s: copy_from_user failed\n", __func__);
		goto rsa_fail;
	}

	sg_init_one(&sg[0], hash_buff, rsa_req_ah->msg_len);
	if (!(rsa_req_ah->keylen))
		goto rsa_fail;

	if (!rsa_req_ah->skip_key) {
		ret = crypto_ahash_setkey(tfm,
				rsa_req_ah->key, rsa_req_ah->keylen);
		if (ret) {
			pr_err("alg: hash: setkey failed\n");
			goto rsa_fail;
		}
	}

	ahash_request_set_crypt(req, sg, result, rsa_req_ah->msg_len);

	ret = crypto_ahash_digest(req);

	if (ret == -EINPROGRESS || ret == -EBUSY) {
		ret = wait_for_completion_interruptible(&rsa_complete.restart);
		if (!ret)
			ret = rsa_complete.req_err;
		reinit_completion(&rsa_complete.restart);
	}

	if (ret) {
		pr_err("alg: hash: digest failed\n");
	goto rsa_fail;
	}

	ret = copy_to_user((void __user *)rsa_req_ah->result,
		(const void *)result,
		crypto_ahash_digestsize(tfm));
	if (ret) {
		ret = -EFAULT;
	pr_err("alg: hash: copy_to_user failed (%d)\n", ret);
	}

rsa_fail:
	kfree(result);
result_fail:
	free_bufs(xbuf);
buf_fail:
	ahash_request_free(req);
req_fail:
	crypto_free_ahash(tfm);
out:
	kfree(key_mem);
	return ret;
}

static int tegra_crypt_pka1_eddsa(struct tegra_pka1_eddsa_request *eddsa_req)
{
	struct crypto_akcipher *tfm = NULL;
	struct akcipher_request *req = NULL;
	u8 *m_str = NULL;
	struct scatterlist src, dst;
	struct scatterlist src_tab[3];
	struct tegra_crypto_completion result;
	unsigned int outbuf_maxlen;
	void *outbuf = NULL;
	void *key = NULL;
	u8 *key_buf = NULL;
	unsigned int nbytes;
	int err;
	u8 *keymem = NULL;
	u8 *output = NULL;
	u8 *public_key = NULL;

	tfm = crypto_alloc_akcipher("eddsa",
				CRYPTO_ALG_TYPE_AKCIPHER, 0);
	if (IS_ERR(tfm)) {
		pr_err("Failed to load transform for eddsa: %ld\n",
				PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	/* Alloc akcipher request */
	req = akcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("Failed to allocate akcipher request\n");
		err = -ENOMEM;
		goto free_tfm;
	}

	keymem = kzalloc(eddsa_req->keylen, GFP_KERNEL);
	if (!keymem) {
		err = -ENOMEM;
		goto free_req;
	}

	err = copy_from_user(keymem, (void __user *)eddsa_req->key,
			     eddsa_req->keylen);
	if (err) {
		err = -EFAULT;
		pr_err("copy_from_user failed (%d) for eddsa key\n", err);
		goto free_mem;
	}

	/* Set private key */
	err = crypto_akcipher_set_priv_key(tfm, keymem, eddsa_req->keylen);
	if (err) {
		pr_err("eddsa set priv key failed\n");
		goto free_mem;
	}

	nbytes = eddsa_req->nbytes;

	key = kzalloc(nbytes, GFP_KERNEL);
	if (!key) {
		err = -ENOMEM;
		goto free_mem;
	}

	/* Set up result callback */
	init_completion(&result.restart);
	akcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      tegra_crypt_complete, &result);

	/* Generate pub key */
	err = wait_async_op(&result,
			    crypto_akcipher_set_pub_key(tfm, key, nbytes));
	if (err) {
		pr_err("alg:eddsa set_pub_key test failed\n");
		goto free_key;
	}

	key_buf = (u8 *)key;

	public_key = kzalloc(nbytes, GFP_KERNEL);
	if (!public_key) {
		err = -ENOMEM;
		goto free_key;
	}

	err = copy_from_user(public_key, (void __user *)eddsa_req->public_key,
			     nbytes);
	if (err) {
		pr_err("copy_from_user failed (%d) for eddsa public key\n",
		       err);
		err = -EFAULT;
		goto free_pub_key;
	}

	if (memcmp(key, public_key, nbytes)) {
		err = -EINVAL;
		pr_err("alg:eddsa set_pub_key test failed. Invalid Output\n");
		goto free_pub_key;
	}

	m_str = kzalloc(eddsa_req->msize, GFP_KERNEL);
	if (!m_str) {
		err = -ENOMEM;
		goto free_pub_key;
	}

	err = copy_from_user(m_str, (void __user *)eddsa_req->message,
			     eddsa_req->msize);
	if (err) {
		pr_err("copy_from_user failed (%d) for eddsa message\n", err);
		err = -EFAULT;
		goto free_msg;
	}

	output = kzalloc(nbytes * 2, GFP_KERNEL);
	if (!output) {
		err = -ENOMEM;
		goto free_msg;
	}

	sg_init_one(&src, m_str, eddsa_req->msize);
	sg_init_one(&dst, output, nbytes * 2);

	akcipher_request_set_crypt(req, &src, &dst,
				   eddsa_req->msize, nbytes * 2);

	/* Set up result callback */
	init_completion(&result.restart);
	akcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      tegra_crypt_complete, &result);

	/* Run eddsa sign operation on message digest */
	err = wait_async_op(&result, crypto_akcipher_sign(req));
	if (err) {
		pr_err("alg:eddsa sign test failed\n");
		goto free_output;
	}

	/* verify that signature (r,s) is valid */
	if (req->dst_len != 2 * nbytes) {
		err = -EINVAL;
		goto free_output;
	}

	err = copy_to_user((void __user *)eddsa_req->signature,
			   output, 2 * nbytes);
	if (err) {
		err = -EFAULT;
		pr_debug("%s: copy_to_user failed (%d)\n", __func__, err);
		goto free_output;
	}

	outbuf_maxlen = crypto_akcipher_maxsize(tfm);
	outbuf = kzalloc(outbuf_maxlen, GFP_KERNEL);
	if (!outbuf) {
		pr_err("Failed to allocate outbuf memory\n");
		err = -ENOMEM;
		goto free_output;
	}

	/* Set src and dst buffers */
	sg_init_table(src_tab, 3);
	sg_set_buf(&src_tab[0], m_str, eddsa_req->msize);
	sg_set_buf(&src_tab[1], output, nbytes);
	sg_set_buf(&src_tab[2], output + nbytes, nbytes);
	sg_init_one(&dst, outbuf, outbuf_maxlen);

	akcipher_request_set_crypt(req, src_tab, &dst,
				   eddsa_req->msize + 2 * nbytes,
				   outbuf_maxlen);

	/* Set up result callback */
	init_completion(&result.restart);
	akcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      tegra_crypt_complete, &result);

	/* Run eddsa verify operation on sig (r,s) */
	err = wait_async_op(&result, crypto_akcipher_verify(req));

	kfree(outbuf);
free_output:
	kfree(output);
free_msg:
	kfree(m_str);
free_pub_key:
	kfree(public_key);
free_key:
	kfree(key);
free_mem:
	kfree(keymem);
free_req:
	akcipher_request_free(req);
free_tfm:
	crypto_free_akcipher(tfm);

	return err;
}

static int tegra_crypt_pka1_ecc(struct tegra_se_pka1_ecc_request *ecc_req)
{
	struct tegra_se_pka1_ecc_request temp_ecc_req;
	int ret;

	if ((ecc_req->op_mode < ECC_MODE_MIN_INDEX) ||
	    (ecc_req->op_mode > ECC_MODE_MAX_INDEX)) {
		pr_err("Invalid value of ecc opmode index %d\n",
			ecc_req->op_mode);
		return -EINVAL;
	}

	temp_ecc_req.op_mode = ecc_req->op_mode;
	temp_ecc_req.size = ecc_req->size;
	temp_ecc_req.type = ecc_req->type;

	temp_ecc_req.modulus = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.modulus) {
		ret = -ENOMEM;
		goto mod_fail;
	}
	temp_ecc_req.curve_param_a = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.curve_param_a) {
		ret = -ENOMEM;
		goto param_a_fail;
	}
	temp_ecc_req.curve_param_b = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.curve_param_b) {
		ret = -ENOMEM;
		goto param_b_fail;
	}
	temp_ecc_req.base_pt_x = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.base_pt_x) {
		ret =  -ENOMEM;
		goto base_px_fail;
	}
	temp_ecc_req.base_pt_y = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.base_pt_y) {
		ret = -ENOMEM;
		goto base_py_fail;
	}
	temp_ecc_req.res_pt_x = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.res_pt_x) {
		ret = -ENOMEM;
		goto res_px_fail;
	}
	temp_ecc_req.res_pt_y = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.res_pt_y) {
		ret = -ENOMEM;
		goto res_py_fail;
	}
	temp_ecc_req.key = kzalloc(ecc_req->size, GFP_KERNEL);
	if (!temp_ecc_req.key) {
		ret = -ENOMEM;
		goto key_fail;
	}

	ret = copy_from_user(temp_ecc_req.modulus,
			     (void __user *)ecc_req->modulus, ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
		goto free_all;
	}

	ret = copy_from_user(temp_ecc_req.curve_param_a,
			     (void __user *)ecc_req->curve_param_a,
			     ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
		goto free_all;
	}

	if ((ecc_req->type == ECC_POINT_VER) ||
	    (ecc_req->type == ECC_SHAMIR_TRICK)) {
		ret = copy_from_user(temp_ecc_req.curve_param_b,
				     (void __user *)ecc_req->curve_param_b,
				     ecc_req->size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				 __func__, ret);
			goto free_all;
		}
	}

	if (ecc_req->type != ECC_POINT_DOUBLE) {
		ret = copy_from_user(temp_ecc_req.base_pt_x,
				     (void __user *)ecc_req->base_pt_x,
				     ecc_req->size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				__func__, ret);
			goto free_all;
		}

		ret = copy_from_user(temp_ecc_req.base_pt_y,
				     (void __user *)ecc_req->base_pt_y,
				     ecc_req->size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				__func__, ret);
			goto free_all;
		}
	}

	ret = copy_from_user(temp_ecc_req.res_pt_x,
			     (void __user *)ecc_req->res_pt_x, ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
		goto free_all;
	}

	ret = copy_from_user(temp_ecc_req.res_pt_y,
			     (void __user *)ecc_req->res_pt_y, ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
		goto free_all;
	}

	if ((ecc_req->type == ECC_POINT_MUL) ||
	    (ecc_req->type == ECC_SHAMIR_TRICK)) {
		ret = copy_from_user(temp_ecc_req.key,
				     (void __user *)ecc_req->key,
				     ecc_req->size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				 __func__, ret);
			goto free_all;
		}
	}

	ret = tegra_se_pka1_ecc_op(&temp_ecc_req);
	if (ret) {
		pr_debug("\ntegra_se_pka1_ecc_op failed(%d) for ECC\n", ret);
		goto free_all;
	}

	ret = copy_to_user((void __user *)ecc_req->res_pt_x,
			   temp_ecc_req.res_pt_x, ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_to_user failed (%d)\n", __func__, ret);
		goto free_all;
	}

	ret = copy_to_user((void __user *)ecc_req->res_pt_y,
			   temp_ecc_req.res_pt_y, ecc_req->size);
	if (ret) {
		ret = -EFAULT;
		pr_debug("%s: copy_to_user failed (%d)\n", __func__, ret);
	}
free_all:
		kfree(temp_ecc_req.key);
key_fail:
		kfree(temp_ecc_req.res_pt_y);
res_py_fail:
		kfree(temp_ecc_req.res_pt_x);
res_px_fail:
		kfree(temp_ecc_req.base_pt_y);
base_py_fail:
		kfree(temp_ecc_req.base_pt_x);
base_px_fail:
		kfree(temp_ecc_req.curve_param_b);
param_b_fail:
		kfree(temp_ecc_req.curve_param_a);
param_a_fail:
		kfree(temp_ecc_req.modulus);
mod_fail:
	return ret;
}

static int tegra_crypt_pka1_rsa(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_pka1_rsa_request *rsa_req)
{
	struct crypto_akcipher *tfm = NULL;
	struct akcipher_request *req;
	struct scatterlist sg[2];
	int ret = 0;
	int len;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;

	if (rsa_req->op_mode == RSA_INIT) {
		tfm = crypto_alloc_akcipher("rsa", CRYPTO_ALG_TYPE_AKCIPHER, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for rsa: %ld\n",
				PTR_ERR(tfm));
			return PTR_ERR(tfm);
		}
		ctx->pka1_rsa_tfm = tfm;
		filp->private_data = ctx;

		return 0;
	}

	ctx = filp->private_data;
	tfm =  ctx->pka1_rsa_tfm;

	if ((rsa_req->op_mode == RSA_SET_PUB) ||
	    (rsa_req->op_mode == RSA_SET_PRIV)) {
		ret = tegra_cryptodev_rsa_set_key(tfm, rsa_req->key,
						  rsa_req->keylen,
						  MAX_RSA1_MSG_LEN,
						  rsa_req->op_mode);
		if (ret < 0) {
			pr_err("alg: pka1:rsa: set key failed\n");
			return ret;
		}
	} else if (rsa_req->op_mode == RSA_ENCRYPT ||
		   rsa_req->op_mode == RSA_DECRYPT ||
		   rsa_req->op_mode == RSA_SIGN ||
		   rsa_req->op_mode == RSA_VERIFY) {
		req = akcipher_request_alloc(tfm, GFP_KERNEL);
		if (!req) {
			pr_err("alg: rsa: Failed to allocate request: %s\n",
				__func__);
			ret = -ENOMEM;
			goto out_tfm;
		}

		ret = alloc_bufs(xbuf);
		if (ret < 0) {
			pr_err("alloc_bufs failed");
			goto buf_fail;
		}

		init_completion(&rsa_complete.restart);
		rsa_complete.req_err = 0;

		len = crypto_akcipher_maxsize(tfm);
		if (len < 0) {
			ret = -EINVAL;
			goto buf_fail;
		}

		ret = copy_from_user((void *)xbuf[0],
				     (void __user *)rsa_req->message, len);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				 __func__, ret);
			goto copy_fail;
		}

		sg_init_one(&sg[0], xbuf[0], len);
		sg_init_one(&sg[1], xbuf[1], len);

		akcipher_request_set_crypt(req, &sg[0], &sg[1], len, len);

		if (rsa_req->op_mode == RSA_ENCRYPT) {
			ret = crypto_akcipher_encrypt(req);
			if (ret) {
				pr_err("alg: pka1:rsa: encrypt failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_DECRYPT) {
			ret = crypto_akcipher_decrypt(req);
			if (ret) {
				pr_err("alg: pka1:rsa: decrypt failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_SIGN) {
			ret = crypto_akcipher_sign(req);
			if (ret) {
				pr_err("alg: pka1:rsa: sign failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_VERIFY) {
			ret = crypto_akcipher_verify(req);
			if (ret) {
				pr_err("alg: pka1:rsa: verification failed\n");
				goto copy_fail;
			}
		}

		ret = copy_to_user((void __user *)rsa_req->result,
				   (const void *)xbuf[1], len);
		if (ret) {
			ret = -EFAULT;
			pr_err("alg: pka1:rsa: copy_to_user failed (%d)\n",
				ret);
		}
copy_fail:
		free_bufs(xbuf);
buf_fail:
		akcipher_request_free(req);
	} else if (rsa_req->op_mode == RSA_EXIT) {
		crypto_free_akcipher(tfm);
	} else {
		pr_err("alg: pka1:rsa: invalid rsa operation\n");
	}
out_tfm:
	if (ret)
		crypto_free_akcipher(tfm);

	return ret;
}

static int tegra_crypto_sha(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_sha_req *sha_req)
{

	struct crypto_ahash *tfm;
	struct scatterlist sg[1];
	char result[64];
	char algo[64];
	struct ahash_request *req;
	struct tegra_crypto_completion sha_complete;
	void *hash_buff;
	unsigned long *xbuf[XBUFSIZE];
	int ret = -ENOMEM;

	if (sha_req->plaintext_sz > PAGE_SIZE) {
		pr_err("alg:hash: invalid plaintext_sz for sha_req\n");
		return -EINVAL;
	}

	if (strncpy_from_user(algo, sha_req->algo, sizeof(algo) - 1) < 0) {
		pr_err("alg:hash: invalid algo for sha_req\n");
		return -EFAULT;
	}
	algo[sizeof(algo) - 1] = '\0';

	tfm = crypto_alloc_ahash(algo, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("alg:hash:Failed to load transform for %s:%ld\n",
			algo, PTR_ERR(tfm));
		goto out_alloc;
	}

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("alg:hash:Failed to allocate request for %s\n", algo);
		goto out_noreq;
	}

	ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				   tegra_crypt_complete, &sha_complete);

	ret = alloc_bufs(xbuf);
	if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto out_buf;
	}

	init_completion(&sha_complete.restart);
	sha_complete.req_err = 0;

	memset(result, 0, 64);

	hash_buff = xbuf[0];

	ret = copy_from_user((void *)hash_buff,
			     (void __user *)sha_req->plaintext,
			     sha_req->plaintext_sz);
	if (ret) {
		ret = -EFAULT;
		pr_err("%s: copy_from_user failed (%d)\n", __func__, ret);
			goto out;
	}

	sg_init_one(&sg[0], hash_buff, sha_req->plaintext_sz);

	if (sha_req->keylen) {
		crypto_ahash_clear_flags(tfm, ~0);
		ret = crypto_ahash_setkey(tfm, sha_req->key,
					  sha_req->keylen);
		if (ret) {
			pr_err("alg:hash:setkey failed on %s:ret=%d\n",
				sha_req->algo, ret);

			goto out;
		}
	}

	ahash_request_set_crypt(req, sg, result, sha_req->plaintext_sz);

	ret = wait_async_op(&sha_complete, crypto_ahash_init(req));
	if (ret) {
		pr_err("alg: hash: init failed for %s: ret=%d\n",
			sha_req->algo, ret);
		goto out;
	}

	ret = wait_async_op(&sha_complete, crypto_ahash_update(req));
	if (ret) {
		pr_err("alg: hash: update failed for %s: ret=%d\n",
			sha_req->algo, ret);
		goto out;
	}

	ret = wait_async_op(&sha_complete, crypto_ahash_final(req));
	if (ret) {
		pr_err("alg: hash: final failed for %s: ret=%d\n",
			sha_req->algo, ret);
		goto out;
	}

	ret = copy_to_user((void __user *)sha_req->result,
		(const void *)result, crypto_ahash_digestsize(tfm));
	if (ret) {
		ret = -EFAULT;
		pr_err("alg: hash: copy_to_user failed (%d) for %s\n",
				ret, sha_req->algo);
	}

out:
	free_bufs(xbuf);

out_buf:
	ahash_request_free(req);

out_noreq:
	crypto_free_ahash(tfm);

out_alloc:
	return ret;
}

static int tegra_crypto_sha_shash(struct file *filp,
				  struct tegra_crypto_ctx *ctx,
				  struct tegra_sha_req_shash *sha_req_shash)
{
	struct crypto_shash *stfm;
	char result[64];
	struct shash_desc *desc;
	struct tegra_crypto_completion sha_complete;
	void *shash_buff;
	unsigned long *xbuf[XBUFSIZE];
	int ret = -ENOMEM;
	char sha_algo[5][10] = {"sha1", "sha224", "sha256",
				"sha384", "sha512"};

	if (sha_req_shash->plaintext_sz > PAGE_SIZE) {
		pr_err("alg:shash: invalid plaintext_sz for sha_req_shash\n");
		return -EINVAL;
	}

	if (sha_req_shash->algo > SHA512 || sha_req_shash->algo < 0) {
		pr_err("alg: shash: invalid index algo in sha_req_shash\n");
		return -EINVAL;
	}

	sha_req_shash->algo = array_index_nospec(sha_req_shash->algo, SHA512);

	stfm = crypto_alloc_shash(sha_algo[sha_req_shash->algo], 0, 0);
	if (IS_ERR(stfm)) {
		pr_err("alg:shash:Failed to load transform for %s:%ld\n",
			sha_algo[sha_req_shash->algo], PTR_ERR(stfm));
		goto out_alloc;
	}

	ctx->shash_tfm[sha_req_shash->algo] = stfm;

	filp->private_data = ctx;
	desc = shash_request_alloc(stfm, GFP_KERNEL);
	if (!desc) {
		pr_err("alg:shash:Failed to allocate request for %s\n",
			sha_algo[sha_req_shash->algo]);
		goto out_noreq;
	}

	ret = alloc_bufs(xbuf);
	if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto out_buf;
	}

	init_completion(&sha_complete.restart);
	sha_complete.req_err = 0;
	shash_buff = xbuf[0];
	ret = copy_from_user((void *)shash_buff,
				(void __user *)sha_req_shash->plaintext,
				sha_req_shash->plaintext_sz);
	if (ret) {
		ret = -EFAULT;
		pr_err("%s: copy_from_user failed (%d)\n", __func__, ret);
		goto out;
	}
	desc->tfm = stfm;
	ret = sha_shash_hash_op(desc, &sha_complete, crypto_shash_init(desc));
	if (ret) {
		pr_err("alg: shash: init failed for %s: ret=%d\n",
			sha_algo[sha_req_shash->algo], ret);
		goto out;
	}

	ret = sha_shash_hash_op(desc, &sha_complete,
		crypto_shash_update(desc, shash_buff,
					sha_req_shash->plaintext_sz));
	if (ret) {
		pr_err("alg: shash: update failed for %s: ret=%d\n",
			sha_algo[sha_req_shash->algo], ret);
		goto out;
	}

	ret = sha_shash_hash_op(desc, &sha_complete,
			crypto_shash_final(desc, result));
	if (ret) {
		pr_err("alg: shash: final failed for %s: ret=%d\n",
			sha_algo[sha_req_shash->algo], ret);
		goto out;
	}

	ret = copy_to_user((void __user *)sha_req_shash->result,
		(const void *)result, crypto_shash_digestsize(stfm));
	if (ret) {
		ret = -EFAULT;
		pr_err("alg: shash: copy_to_user failed (%d) for %s\n",
				ret, sha_algo[sha_req_shash->algo]);
	}

out:
	free_bufs(xbuf);

out_buf:
	shash_request_free(desc);

out_noreq:
	crypto_free_shash(stfm);

out_alloc:
	return ret;
}

static long tegra_crypto_dev_ioctl(struct file *filp,
	unsigned int ioctl_num, unsigned long arg)
{
	struct tegra_crypto_ctx *ctx = filp->private_data;
	struct crypto_rng *tfm = NULL;
	struct tegra_pka1_rsa_request pka1_rsa_req;
	struct tegra_se_pka1_ecc_request pka1_ecc_req;
	struct tegra_pka1_eddsa_request pka1_eddsa_req;
	struct tegra_crypt_req crypt_req;
	struct tegra_rng_req rng_req;
	struct tegra_sha_req sha_req;
	struct tegra_sha_req_shash sha_req_shash;
	struct tegra_rsa_req rsa_req;
	struct tegra_rsa_req_ahash rsa_req_ah;
#ifdef CONFIG_COMPAT
	struct tegra_crypt_req_32 crypt_req_32;
	struct tegra_rng_req_32 rng_req_32;
	struct tegra_sha_req_32 sha_req_32;
	int i = 0;
#endif
	char *rng;
	int ret = 0;

	switch (ioctl_num) {
	case TEGRA_CRYPTO_IOCTL_NEED_SSK:
		ctx->use_ssk = (int)arg;
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_PROCESS_REQ_32:
		ret = copy_from_user(&crypt_req_32, (void __user *)arg,
			sizeof(struct tegra_crypt_req_32));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		if (crypt_req_32.keylen > TEGRA_CRYPTO_MAX_KEY_SIZE) {
			pr_err("key length %d exceeds max value %d\n",
				crypt_req_32.keylen, TEGRA_CRYPTO_MAX_KEY_SIZE);
			return -EINVAL;
		}
		crypt_req.op = crypt_req_32.op;
		crypt_req.encrypt = crypt_req_32.encrypt;
		crypt_req.skip_key = crypt_req_32.skip_key;
		crypt_req.skip_iv = crypt_req_32.skip_iv;
		for (i = 0; i < crypt_req_32.keylen; i++)
			crypt_req.key[i] = crypt_req_32.key[i];
		crypt_req.keylen = crypt_req_32.keylen;
		for (i = 0; i < TEGRA_CRYPTO_IV_SIZE; i++)
			crypt_req.iv[i] = crypt_req_32.iv[i];
		crypt_req.ivlen = crypt_req_32.ivlen;
		crypt_req.plaintext =
			(u8 __user *)(void *)(__u64)(crypt_req_32.plaintext);
		crypt_req.plaintext_sz = crypt_req_32.plaintext_sz;
		crypt_req.result =
			(u8 __user *)(void *)(__u64)(crypt_req_32.result);

		ret = process_crypt_req(filp, ctx, &crypt_req);
		break;
#endif
	case TEGRA_CRYPTO_IOCTL_PROCESS_REQ:
		ret = copy_from_user(&crypt_req, (void __user *)arg,
			sizeof(crypt_req));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		ret = process_crypt_req(filp, ctx, &crypt_req);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_SET_SEED_32:
		if (copy_from_user(&rng_req_32, (void __user *)arg,
			sizeof(rng_req_32))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		for (i = 0; i < TEGRA_CRYPTO_RNG_SEED_SIZE; i++)
			rng_req.seed[i] = rng_req_32.seed[i];
		rng_req.type = rng_req_32.type;
		/* fall through */
#endif

	case TEGRA_CRYPTO_IOCTL_SET_SEED:
		if (ioctl_num == TEGRA_CRYPTO_IOCTL_SET_SEED) {
			if (copy_from_user(&rng_req, (void __user *)arg,
				sizeof(rng_req))) {
				pr_err("%s: copy_from_user fail(%d)\n",
						__func__, ret);
				return -EFAULT;
			}
		}

		memcpy(ctx->seed, rng_req.seed, TEGRA_CRYPTO_RNG_SEED_SIZE);

		if (rng_req.type == RNG_DRBG) {
			if (tegra_get_chip_id() == TEGRA20 ||
				tegra_get_chip_id() == TEGRA30) {
				return -EINVAL;
			}
			ctx->rng_drbg = crypto_alloc_rng("rng_drbg-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng_drbg)) {
				pr_err("Failed to load transform for "
				"rng_drbg tegra: %ld\n",
					PTR_ERR(ctx->rng_drbg));
				ret = PTR_ERR(ctx->rng_drbg);
				goto out;
			}
			tfm = ctx->rng_drbg;
			filp->private_data = ctx;
			ret = crypto_rng_reset(ctx->rng_drbg, ctx->seed,
				crypto_rng_seedsize(ctx->rng_drbg));
		} else {
			ctx->rng = crypto_alloc_rng("rng-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng)) {
				pr_err("Failed to load transform for "
					"tegra rng: %ld\n", PTR_ERR(ctx->rng));
				ret = PTR_ERR(ctx->rng);
				goto out;
			}
			tfm = ctx->rng;
			filp->private_data = ctx;
			ret = crypto_rng_reset(ctx->rng, ctx->seed,
				crypto_rng_seedsize(ctx->rng));
		}
		if (ret)
			pr_err("crypto_rng_reset failed: %d\n", ret);
		crypto_free_rng(tfm);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_GET_RANDOM_32:
		if (copy_from_user(&rng_req_32, (void __user *)arg,
			sizeof(rng_req_32))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		rng_req.nbytes = rng_req_32.nbytes;
		rng_req.type = rng_req_32.type;
		rng_req.rdata = (u8 __user *)(void *)(__u64)rng_req_32.rdata;
		/* fall through */
#endif

	case TEGRA_CRYPTO_IOCTL_GET_RANDOM:
		if (ioctl_num == TEGRA_CRYPTO_IOCTL_GET_RANDOM) {
			if (copy_from_user(&rng_req, (void __user *)arg,
				sizeof(rng_req))) {
				pr_err("%s: copy_from_user fail(%d)\n",
						__func__, ret);
				return -EFAULT;
			}
		}

		rng = kzalloc(rng_req.nbytes, GFP_KERNEL);
		if (!rng) {
			if (rng_req.type == RNG_DRBG)
				pr_err("mem alloc for rng_drbg fail");
			else
				pr_err("mem alloc for rng fail");

			return -ENODATA;
		}

		if (rng_req.type == RNG_DRBG) {
			if (tegra_get_chip_id() == TEGRA20 ||
				tegra_get_chip_id() == TEGRA30) {
				ret = -EINVAL;
				goto rng_out;
			}
			ctx->rng_drbg = crypto_alloc_rng("rng_drbg-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng_drbg)) {
				pr_err("Failed to load transform for "
				"rng_drbg tegra: %ld\n",
						PTR_ERR(ctx->rng_drbg));
				ret = PTR_ERR(ctx->rng_drbg);
				goto rng_out;
			}
			tfm = ctx->rng_drbg;
			filp->private_data = ctx;
			ret = crypto_rng_get_bytes(ctx->rng_drbg, rng,
				rng_req.nbytes);
		} else {
			ctx->rng = crypto_alloc_rng("rng-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng)) {
				pr_err("Failed to load transform for "
					"tegra rng: %ld\n", PTR_ERR(ctx->rng));
				ret = PTR_ERR(ctx->rng);
				goto rng_out;
			}
			tfm = ctx->rng;
			filp->private_data = ctx;
			ret = crypto_rng_get_bytes(ctx->rng, rng,
				rng_req.nbytes);
		}

		if (ret != rng_req.nbytes) {
			if (rng_req.type == RNG_DRBG)
				pr_err("rng_drbg failed");
			else
				pr_err("rng failed");
			ret = -ENODATA;
			goto free_tfm;
		}
		ret = copy_to_user((void __user *)rng_req.rdata,
			(const void *)rng, rng_req.nbytes);
		if (ret) {
			ret = -EFAULT;
			pr_err("%s: copy_to_user fail(%d)\n", __func__, ret);
			goto free_tfm;
		}
free_tfm:
		crypto_free_rng(tfm);
rng_out:
		kfree(rng);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_GET_SHA_32:
		ret = copy_from_user(&sha_req_32, (void __user *)arg,
			sizeof(sha_req_32));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		if (sha_req_32.keylen > TEGRA_CRYPTO_MAX_KEY_SIZE) {
			pr_err("key length %d not within the range [0,%d]\n",
				sha_req_32.keylen, TEGRA_CRYPTO_MAX_KEY_SIZE);
			return -EINVAL;
		}
		for (i = 0; i < sha_req_32.keylen; i++)
			sha_req.key[i] = sha_req_32.key[i];
		sha_req.keylen = sha_req_32.keylen;
		sha_req.algo =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.algo);
		sha_req.plaintext =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.plaintext);
		sha_req.plaintext_sz = sha_req_32.plaintext_sz;
		sha_req.result =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.result);

		ret = tegra_crypto_sha(filp, ctx, &sha_req);
		break;
#endif

	case TEGRA_CRYPTO_IOCTL_GET_SHA:
		if (tegra_get_chip_id() == TEGRA20)
			return -EINVAL;
		if (copy_from_user(&sha_req, (void __user *)arg,
				sizeof(sha_req))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		if (sha_req.keylen > TEGRA_CRYPTO_MAX_KEY_SIZE) {
			pr_err("key length %d not within the range [0,%d]\n",
				sha_req.keylen, TEGRA_CRYPTO_MAX_KEY_SIZE);
			return -EINVAL;
		}
		ret = tegra_crypto_sha(filp, ctx, &sha_req);
		break;

	case TEGRA_CRYPTO_IOCTL_GET_SHA_SHASH:
		if (copy_from_user(&sha_req_shash, (void __user *)arg,
				sizeof(sha_req_shash))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		ret = tegra_crypto_sha_shash(filp, ctx, &sha_req_shash);
		break;

	case TEGRA_CRYPTO_IOCTL_RSA_REQ_AHASH:
		if (copy_from_user(&rsa_req_ah, (void __user *)arg,
			sizeof(rsa_req_ah))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		if (rsa_req_ah.algo >= NUM_RSA_ALGO) {
			pr_err("Invalid value of algo index %d\n",
				rsa_req_ah.algo);
			return -EINVAL;
		}
		rsa_req_ah.algo = array_index_nospec(rsa_req_ah.algo,
								NUM_RSA_ALGO);
		if (rsa_req_ah.msg_len > MAX_RSA_MSG_LEN) {
			pr_err("Illegal message from user of length = %d\n",
				rsa_req_ah.msg_len);
			return -EINVAL;
		}
		rsa_req_ah.msg_len = array_index_nospec(rsa_req_ah.msg_len,
							MAX_RSA_MSG_LEN + 1);
		ret = tegra_crypt_rsa_ahash(filp, ctx, &rsa_req_ah);
		break;

	case TEGRA_CRYPTO_IOCTL_RSA_REQ:
		if (copy_from_user(&rsa_req, (void __user *)arg,
			sizeof(rsa_req))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		if (rsa_req.msg_len > MAX_RSA_MSG_LEN) {
			pr_err("Illegal message from user of length = %d\n",
				rsa_req.msg_len);
			return -EINVAL;
		}
		rsa_req.msg_len = array_index_nospec(rsa_req.msg_len,
							MAX_RSA_MSG_LEN + 1);
		if (rsa_req.algo >= NUM_RSA_ALGO) {
			pr_err("Invalid value of algo index %d\n",
				rsa_req.algo);
			return -EINVAL;
		}
		rsa_req.algo = array_index_nospec(rsa_req.algo,
							NUM_RSA_ALGO);

		ret = tegra_crypt_rsa(filp, ctx, &rsa_req);
		break;

	case TEGRA_CRYPTO_IOCTL_PKA1_RSA_REQ:
		if (copy_from_user(&pka1_rsa_req, (void __user *)arg,
			sizeof(pka1_rsa_req))) {
			pr_err("%s: copy_from_user fail(%d) for pka1_rsa_req\n",
				__func__, ret);
			return -EFAULT;
		}

		ret = tegra_crypt_pka1_rsa(filp, ctx, &pka1_rsa_req);
		break;

	case TEGRA_CRYPTO_IOCTL_PKA1_ECC_REQ:
		if (copy_from_user(&pka1_ecc_req, (void __user *)arg,
				   sizeof(pka1_ecc_req))) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user fail(%d) for pka1_ecc_req\n",
				__func__, ret);
			return ret;
		}

		ret = tegra_crypt_pka1_ecc(&pka1_ecc_req);
		break;

	case TEGRA_CRYPTO_IOCTL_PKA1_EDDSA_REQ:
		if (copy_from_user(&pka1_eddsa_req, (void __user *)arg,
				   sizeof(pka1_eddsa_req))) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user fail(%d) for pka1_eddsa_req\n",
				__func__, ret);
			return ret;
		}

		ret = tegra_crypt_pka1_eddsa(&pka1_eddsa_req);
		break;

	case TEGRA_CRYPTO_IOCTL_RNG1_REQ:
		if (copy_from_user(&rng_req, (void __user *)arg,
			sizeof(rng_req))) {
			pr_err("%s: copy_from_user fail(%d)\n",
					__func__, ret);
			return -EFAULT;
		}

		rng = kzalloc(rng_req.nbytes, GFP_KERNEL);
		if (!rng)
			return -ENODATA;

		ctx->rng = crypto_alloc_rng("rng1-elp-tegra",
			CRYPTO_ALG_TYPE_RNG, 0);
		if (IS_ERR(ctx->rng)) {
			pr_err("Failed to alloc rng1: %ld\n",
						PTR_ERR(ctx->rng));
			ret = PTR_ERR(ctx->rng);
			goto rng1_out;
		}

		tfm = ctx->rng;
		filp->private_data = ctx;
		ret = crypto_rng_get_bytes(ctx->rng, rng,
				rng_req.nbytes);

		if (ret != rng_req.nbytes) {
			pr_err("rng failed");
			ret = -ENODATA;
			goto free_rng1_tfm;
		}

		ret = copy_to_user((void __user *)rng_req.rdata,
			(const void *)rng, rng_req.nbytes);
		if (ret) {
			ret = -EFAULT;
			pr_err("%s: copy_to_user fail(%d)\n", __func__, ret);
			goto free_rng1_tfm;
		}
free_rng1_tfm:
		crypto_free_rng(tfm);
rng1_out:
		kfree(rng);
		break;

	default:
		pr_debug("invalid ioctl code(%d)", ioctl_num);
		return -EINVAL;
	}
out:
	return ret;
}

static const struct file_operations tegra_crypto_fops = {
	.owner = THIS_MODULE,
	.open = tegra_crypto_dev_open,
	.release = tegra_crypto_dev_release,
	.unlocked_ioctl = tegra_crypto_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =  tegra_crypto_dev_ioctl,
#endif
};

static struct miscdevice tegra_crypto_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tegra-crypto",
	.fops = &tegra_crypto_fops,
};

static int __init tegra_crypto_dev_init(void)
{
	return misc_register(&tegra_crypto_device);
}

late_initcall(tegra_crypto_dev_init);

static void __exit tegra_crypto_module_exit(void)
{
	misc_deregister(&tegra_crypto_device);
}
module_exit(tegra_crypto_module_exit);

MODULE_DESCRIPTION("Tegra AES hw device node.");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
