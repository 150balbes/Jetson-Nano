/*
 * include/uapi/misc/tegra-cryptodev.h
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION. All Rights Reserved.
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

#ifndef __UAPI_TEGRA_CRYPTODEV_H
#define __UAPI_TEGRA_CRYPTODEV_H

#include <asm-generic/ioctl.h>

/* ioctl arg = 1 if you want to use ssk. arg = 0 to use normal key */
#define TEGRA_CRYPTO_IOCTL_NEED_SSK		_IOWR(0x98, 100, int)

#define AES_BLOCK_SIZE		16
#define AES_KEYSIZE_128		16
#define TEGRA_CRYPTO_MAX_KEY_SIZE	TEGRA_CRYPTO_KEY_512_SIZE
#define RSA_KEY_SIZE		512
#define TEGRA_CRYPTO_IV_SIZE	AES_BLOCK_SIZE
#define DEFAULT_RNG_BLK_SZ	16
#define TEGRA_CRYPTO_KEY_512_SIZE	64
#define TEGRA_CRYPTO_KEY_256_SIZE	32
#define TEGRA_CRYPTO_KEY_192_SIZE	24
#define TEGRA_CRYPTO_KEY_128_SIZE	16
/* 14 AES key slots chosen for keyslt_test, leaving ssk/srk keyslots for which
 * driver does not allocate slots
 */
#define TEGRA_CRYPTO_AES_TEST_KEYSLOTS	14

#define CRYPTO_KEY_LEN_MASK	0x3FF

#define TEGRA_CRYPTO_EDDSA_SIGN		0
#define TEGRA_CRYPTO_EDDSA_VERIFY	1

/* the seed consists of 16 bytes of key + 16 bytes of init vector */
#define TEGRA_CRYPTO_RNG_SEED_SIZE	AES_KEYSIZE_128 + DEFAULT_RNG_BLK_SZ
#define TEGRA_CRYPTO_RNG_SIZE	SZ_16

enum tegra_se_crypto_dev_mode {
	/* Electronic Codebook (ECB) mode */
	TEGRA_CRYPTO_ECB,
	/* Cipher Block Chaining (CBC) mode */
	TEGRA_CRYPTO_CBC,
	/* Output feedback (OFB) mode */
	TEGRA_CRYPTO_OFB,
	/* Counter (CTR) mode */
	TEGRA_CRYPTO_CTR,
	/* XEX with Ciphertext stealing (XTS) mode*/
	TEGRA_CRYPTO_XTS,
	/* max mode number */
	TEGRA_CRYPTO_MAX
};

enum tegra_rsa_op_mode {
	RSA_INIT,
	RSA_SET_PUB,
	RSA_SET_PRIV,
	RSA_ENCRYPT,
	RSA_DECRYPT,
	RSA_SIGN,
	RSA_VERIFY,
	RSA_EXIT,
};

enum sha_algo {
	SHA1,
	SHA224,
	SHA256,
	SHA384,
	SHA512,
	CMAC_AES,
};

struct tegra_se_rng1_request {
	unsigned int size;
	__u32 *rdata;
	__u32 *rdata1;
	__u32 *rdata2;
	__u32 *rdata3;
	bool test_full_cmd_flow;
	bool adv_state_on;
};
#define TEGRA_CRYPTO_IOCTL_RNG1_REQ	\
		_IOWR(0x98, 108, struct tegra_se_rng1_request)

struct tegra_pka1_eddsa_request {
	struct tegra_se_elp_dev *se_dev;
	char *message;
	unsigned int msize;
	char *key;
	unsigned int keylen;
	char *public_key;
	unsigned int nbytes;
	char *signature;
	unsigned int type;
};
#define TEGRA_CRYPTO_IOCTL_PKA1_EDDSA_REQ	\
		_IOWR(0x98, 109, struct tegra_pka1_eddsa_request)

struct tegra_se_pka1_ecc_request {
	struct tegra_se_elp_dev *se_dev;/* Security Engine device */
	char *message;
	char *result;
	char *modulus;
	char *m;
	char *r2;
	unsigned int size;
	unsigned int op_mode;
	unsigned int type;
	char *curve_param_a;
	char *curve_param_b;
	char *curve_param_c;
	char *order;
	char *base_pt_x;
	char *base_pt_y;
	char *res_pt_x;
	char *res_pt_y;
	char *key;
	bool pv_ok;
};

#define TEGRA_CRYPTO_IOCTL_PKA1_ECC_REQ	\
		_IOWR(0x98, 107, struct tegra_se_pka1_ecc_request)

struct tegra_pka1_rsa_request {
	char *key;
	char *message;
	char *result;
	unsigned int keylen;
	unsigned int algo;
	enum tegra_rsa_op_mode op_mode;
};
#define TEGRA_CRYPTO_IOCTL_PKA1_RSA_REQ	\
		_IOWR(0x98, 106, struct tegra_pka1_rsa_request)

int tegra_se_pka1_ecc_op(struct tegra_se_pka1_ecc_request *req);
int tegra_se_rng1_op(struct tegra_se_rng1_request *req);

/* a pointer to this struct needs to be passed to:
 * TEGRA_CRYPTO_IOCTL_PROCESS_REQ
 */
struct tegra_crypt_req {
	unsigned int op; /* e.g. TEGRA_CRYPTO_ECB */
	bool encrypt;
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	char iv[TEGRA_CRYPTO_IV_SIZE];
	unsigned int ivlen;
	__u8 *plaintext;
	__u8 *result;
	unsigned int plaintext_sz;
	unsigned int skip_key;
	unsigned int skip_iv;
	bool skip_exit;
};
#define TEGRA_CRYPTO_IOCTL_PROCESS_REQ	\
		_IOWR(0x98, 101, struct tegra_crypt_req)

#ifdef CONFIG_COMPAT
struct tegra_crypt_req_32 {
	int op; /* e.g. TEGRA_CRYPTO_ECB */
	bool encrypt;
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	char iv[TEGRA_CRYPTO_IV_SIZE];
	unsigned int ivlen;
	__u32 plaintext;
	__u32 result;
	unsigned int plaintext_sz;
	unsigned int skip_key;
	unsigned int skip_iv;
};
#define TEGRA_CRYPTO_IOCTL_PROCESS_REQ_32	\
		_IOWR(0x98, 121, struct tegra_crypt_req_32)
#endif

/* pointer to this struct should be passed to:
 * TEGRA_CRYPTO_IOCTL_SET_SEED
 * TEGRA_CRYPTO_IOCTL_GET_RANDOM
 */
struct tegra_rng_req {
	__u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	__u8 *rdata; /* random generated data */
	unsigned int nbytes; /* random data length */
	unsigned int type;
};
#define TEGRA_CRYPTO_IOCTL_SET_SEED	\
		_IOWR(0x98, 102, struct tegra_rng_req)
#define TEGRA_CRYPTO_IOCTL_GET_RANDOM	\
		_IOWR(0x98, 103, struct tegra_rng_req)

#ifdef CONFIG_COMPAT
struct tegra_rng_req_32 {
	__u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	__u64 rdata; /* random generated data */
	unsigned int nbytes; /* random data length */
	unsigned int type;
};
#define TEGRA_CRYPTO_IOCTL_SET_SEED_32	\
		_IOWR(0x98, 122, struct tegra_rng_req_32)
#define TEGRA_CRYPTO_IOCTL_GET_RANDOM_32	\
		_IOWR(0x98, 123, struct tegra_rng_req_32)
#endif

struct tegra_rsa_req {
	char *key;
	char *message;
	char *result;
	unsigned int algo;
	unsigned int keylen;
	unsigned int msg_len;
	unsigned int modlen;
	unsigned int pub_explen;
	unsigned int prv_explen;
	unsigned int skip_key;
	enum tegra_rsa_op_mode op_mode;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ	\
		_IOWR(0x98, 105, struct tegra_rsa_req)

struct tegra_rsa_req_ahash {
	char *key;
	char *message;
	char *result;
	unsigned int algo;
	unsigned int keylen;
	unsigned int msg_len;
	unsigned int modlen;
	unsigned int pub_explen;
	unsigned int prv_explen;
	unsigned int skip_key;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ_AHASH	\
		_IOWR(0x98, 110, struct tegra_rsa_req_ahash)

#ifdef CONFIG_COMPAT
struct tegra_rsa_req_32 {
	__u32 key;
	__u32 message;
	__u32 result;
	unsigned int algo;
	unsigned int keylen;
	unsigned int msg_len;
	unsigned int modlen;
	unsigned int pub_explen;
	unsigned int prv_explen;
	unsigned int skip_key;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ_32	\
		_IOWR(0x98, 125, struct tegra_rsa_req_32)
#endif

struct tegra_sha_req {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	unsigned char *algo;
	unsigned char *plaintext;
	unsigned char *result;
	unsigned int plaintext_sz;
};
#define TEGRA_CRYPTO_IOCTL_GET_SHA	\
		_IOWR(0x98, 104, struct tegra_sha_req)

struct tegra_sha_req_shash {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	enum sha_algo algo;
	unsigned char *plaintext;
	unsigned char *result;
	unsigned int plaintext_sz;
};
#define TEGRA_CRYPTO_IOCTL_GET_SHA_SHASH	\
		_IOWR(0x98, 106, struct tegra_sha_req_shash)

#ifdef CONFIG_COMPAT
struct tegra_sha_req_32 {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	__u32 algo;
	__u32 plaintext;
	__u32 result;
	unsigned int plaintext_sz;
};
#define TEGRA_CRYPTO_IOCTL_GET_SHA_32	\
		_IOWR(0x98, 124, struct tegra_sha_req_32)
#endif

#endif
