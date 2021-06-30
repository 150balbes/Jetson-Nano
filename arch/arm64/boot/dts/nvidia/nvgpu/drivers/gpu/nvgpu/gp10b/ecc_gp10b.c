/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/ecc.h>
#include <nvgpu/gk20a.h>

#include "gp10b/ecc_gp10b.h"

int gp10b_ecc_init(struct gk20a *g)
{
	int err = 0;

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(sm_lrf_ecc_single_err_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(sm_lrf_ecc_double_err_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(sm_shm_ecc_sec_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(sm_shm_ecc_sed_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(sm_shm_ecc_ded_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_ecc_total_sec_pipe0_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_ecc_total_ded_pipe0_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_unique_ecc_sec_pipe0_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_unique_ecc_ded_pipe0_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_ecc_total_sec_pipe1_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_ecc_total_ded_pipe1_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_unique_ecc_sec_pipe1_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_TPC(tex_unique_ecc_ded_pipe1_count);
	if (err != 0) {
		goto done;
	}

	err = NVGPU_ECC_COUNTER_INIT_PER_LTS(ecc_sec_count);
	if (err != 0) {
		goto done;
	}
	err = NVGPU_ECC_COUNTER_INIT_PER_LTS(ecc_ded_count);
	if (err != 0) {
		goto done;
	}

done:
	if (err != 0) {
		nvgpu_err(g, "ecc counter allocate failed, err=%d", err);
		nvgpu_ecc_free(g);
	}

	return err;
}
