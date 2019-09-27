/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/log.h>
#include <nvgpu/bitops.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/dma.h>
#include <nvgpu/io.h>
#include <nvgpu/hw_sim.h>
#include <nvgpu/sim.h>
#include <nvgpu/utils.h>
#include <nvgpu/bug.h>
#include <nvgpu/gk20a.h>

int nvgpu_alloc_sim_buffer(struct gk20a *g, struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_sys(g, PAGE_SIZE, mem);
}

void nvgpu_free_sim_buffer(struct gk20a *g, struct nvgpu_mem *mem)
{
	if (nvgpu_mem_is_valid(mem))
		nvgpu_dma_free(g, mem);

	memset(mem, 0, sizeof(*mem));
}

void nvgpu_free_sim_support(struct gk20a *g)
{
	nvgpu_free_sim_buffer(g, &g->sim->send_bfr);
	nvgpu_free_sim_buffer(g, &g->sim->recv_bfr);
	nvgpu_free_sim_buffer(g, &g->sim->msg_bfr);
}

void nvgpu_remove_sim_support(struct gk20a *g)
{
	if (g->sim)
		nvgpu_free_sim_support(g);
}

static inline u32 sim_msg_header_size(void)
{
	return 24;/*TBD: fix the header to gt this from NV_VGPU_MSG_HEADER*/
}

static inline u32 *sim_msg_bfr(struct gk20a *g, u32 byte_offset)
{
	u8 *cpu_va;

	cpu_va = (u8 *)g->sim->msg_bfr.cpu_va;

	return (u32 *)(cpu_va + byte_offset);
}

static inline u32 *sim_msg_hdr(struct gk20a *g, u32 byte_offset)
{
	return sim_msg_bfr(g, byte_offset); /*starts at 0*/
}

static inline u32 *sim_msg_param(struct gk20a *g, u32 byte_offset)
{
	/*starts after msg header/cmn*/
	return sim_msg_bfr(g, byte_offset + sim_msg_header_size());
}

static inline void sim_write_hdr(struct gk20a *g, u32 func, u32 size)
{
	/*memset(g->sim->msg_bfr.kvaddr,0,min(PAGE_SIZE,size));*/
	*sim_msg_hdr(g, sim_msg_signature_r()) = sim_msg_signature_valid_v();
	*sim_msg_hdr(g, sim_msg_result_r())    = sim_msg_result_rpc_pending_v();
	*sim_msg_hdr(g, sim_msg_spare_r())     = sim_msg_spare__init_v();
	*sim_msg_hdr(g, sim_msg_function_r())  = func;
	*sim_msg_hdr(g, sim_msg_length_r())    = size + sim_msg_header_size();
}

static inline u32 sim_escape_read_hdr_size(void)
{
	return 12; /*TBD: fix NV_VGPU_SIM_ESCAPE_READ_HEADER*/
}

static u32 *sim_send_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	u8 *cpu_va;

	cpu_va = (u8 *)g->sim->send_bfr.cpu_va;

	return (u32 *)(cpu_va + byte_offset);
}

static int rpc_send_message(struct gk20a *g)
{
	/* calculations done in units of u32s */
	u32 send_base = sim_send_put_pointer_v(g->sim->send_ring_put) * 2;
	u32 dma_offset = send_base + sim_dma_r()/sizeof(u32);
	u32 dma_hi_offset = send_base + sim_dma_hi_r()/sizeof(u32);

	*sim_send_ring_bfr(g, dma_offset*sizeof(u32)) =
		sim_dma_target_phys_pci_coherent_f() |
		sim_dma_status_valid_f() |
		sim_dma_size_4kb_f() |
		sim_dma_addr_lo_f(nvgpu_mem_get_addr(g, &g->sim->msg_bfr)
				>> PAGE_SHIFT);

	*sim_send_ring_bfr(g, dma_hi_offset*sizeof(u32)) =
		u64_hi32(nvgpu_mem_get_addr(g, &g->sim->msg_bfr));

	*sim_msg_hdr(g, sim_msg_sequence_r()) = g->sim->sequence_base++;

	g->sim->send_ring_put = (g->sim->send_ring_put + 2 * sizeof(u32))
		% PAGE_SIZE;

	/* Update the put pointer. This will trap into the host. */
	sim_writel(g->sim, sim_send_put_r(), g->sim->send_ring_put);

	return 0;
}

static inline u32 *sim_recv_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	u8 *cpu_va;

	cpu_va = (u8 *)g->sim->recv_bfr.cpu_va;

	return (u32 *)(cpu_va + byte_offset);
}

static int rpc_recv_poll(struct gk20a *g)
{
	u64 recv_phys_addr;

	/* XXX This read is not required (?) */
	/*pVGpu->recv_ring_get = VGPU_REG_RD32(pGpu, NV_VGPU_RECV_GET);*/

	/* Poll the recv ring get pointer in an infinite loop*/
	do {
		g->sim->recv_ring_put = sim_readl(g->sim, sim_recv_put_r());
	} while (g->sim->recv_ring_put == g->sim->recv_ring_get);

	/* process all replies */
	while (g->sim->recv_ring_put != g->sim->recv_ring_get) {
		/* these are in u32 offsets*/
		u32 dma_lo_offset =
			sim_recv_put_pointer_v(g->sim->recv_ring_get)*2 + 0;
		u32 dma_hi_offset = dma_lo_offset + 1;
		u32 recv_phys_addr_lo = sim_dma_addr_lo_v(
				*sim_recv_ring_bfr(g, dma_lo_offset*4));
		u32 recv_phys_addr_hi = sim_dma_hi_addr_v(
				*sim_recv_ring_bfr(g, dma_hi_offset*4));

		recv_phys_addr = (u64)recv_phys_addr_hi << 32 |
				 (u64)recv_phys_addr_lo << PAGE_SHIFT;

		if (recv_phys_addr !=
				nvgpu_mem_get_addr(g, &g->sim->msg_bfr)) {
			nvgpu_err(g, "%s Error in RPC reply",
				__func__);
			return -1;
		}

		/* Update GET pointer */
		g->sim->recv_ring_get = (g->sim->recv_ring_get + 2*sizeof(u32))
			% PAGE_SIZE;

		sim_writel(g->sim, sim_recv_get_r(), g->sim->recv_ring_get);

		g->sim->recv_ring_put = sim_readl(g->sim, sim_recv_put_r());
	}

	return 0;
}

static int issue_rpc_and_wait(struct gk20a *g)
{
	int err;

	err = rpc_send_message(g);
	if (err) {
		nvgpu_err(g, "%s failed rpc_send_message",
			__func__);
		return err;
	}

	err = rpc_recv_poll(g);
	if (err) {
		nvgpu_err(g, "%s failed rpc_recv_poll",
			__func__);
		return err;
	}

	/* Now check if RPC really succeeded */
	if (*sim_msg_hdr(g, sim_msg_result_r()) != sim_msg_result_success_v()) {
		nvgpu_err(g, "%s received failed status!",
			__func__);
		return -(*sim_msg_hdr(g, sim_msg_result_r()));
	}
	return 0;
}

static void nvgpu_sim_esc_readl(struct gk20a *g,
		char *path, u32 index, u32 *data)
{
	int err;
	size_t pathlen = strlen(path);
	u32 data_offset;

	sim_write_hdr(g, sim_msg_function_sim_escape_read_v(),
		      sim_escape_read_hdr_size());
	*sim_msg_param(g, 0) = index;
	*sim_msg_param(g, 4) = sizeof(u32);
	data_offset = roundup(0xc +  pathlen + 1, sizeof(u32));
	*sim_msg_param(g, 8) = data_offset;
	strcpy((char *)sim_msg_param(g, 0xc), path);

	err = issue_rpc_and_wait(g);

	if (err == 0) {
		memcpy(data, sim_msg_param(g, data_offset), sizeof(u32));
	} else {
		*data = 0xffffffff;
		WARN(1, "issue_rpc_and_wait failed err=%d", err);
	}
}

static void nvgpu_sim_init_late(struct gk20a *g)
{
	u64 phys;

	if (!g->sim)
		return;

	nvgpu_info(g, "sim init late");
	/*mark send ring invalid*/
	sim_writel(g->sim, sim_send_ring_r(), sim_send_ring_status_invalid_f());

	/*read get pointer and make equal to put*/
	g->sim->send_ring_put = sim_readl(g->sim, sim_send_get_r());
	sim_writel(g->sim, sim_send_put_r(), g->sim->send_ring_put);

	/*write send ring address and make it valid*/
	phys = nvgpu_mem_get_addr(g, &g->sim->send_bfr);
	sim_writel(g->sim, sim_send_ring_hi_r(),
		   sim_send_ring_hi_addr_f(u64_hi32(phys)));
	sim_writel(g->sim, sim_send_ring_r(),
		   sim_send_ring_status_valid_f() |
		   sim_send_ring_target_phys_pci_coherent_f() |
		   sim_send_ring_size_4kb_f() |
		   sim_send_ring_addr_lo_f(phys >> PAGE_SHIFT));

	/*repeat for recv ring (but swap put,get as roles are opposite) */
	sim_writel(g->sim, sim_recv_ring_r(), sim_recv_ring_status_invalid_f());

	/*read put pointer and make equal to get*/
	g->sim->recv_ring_get = sim_readl(g->sim, sim_recv_put_r());
	sim_writel(g->sim, sim_recv_get_r(), g->sim->recv_ring_get);

	/*write send ring address and make it valid*/
	phys = nvgpu_mem_get_addr(g, &g->sim->recv_bfr);
	sim_writel(g->sim, sim_recv_ring_hi_r(),
		   sim_recv_ring_hi_addr_f(u64_hi32(phys)));
	sim_writel(g->sim, sim_recv_ring_r(),
		   sim_recv_ring_status_valid_f() |
		   sim_recv_ring_target_phys_pci_coherent_f() |
		   sim_recv_ring_size_4kb_f() |
		   sim_recv_ring_addr_lo_f(phys >> PAGE_SHIFT));

	return;
}

int nvgpu_init_sim_support(struct gk20a *g)
{
	int err = -ENOMEM;

	if (!g->sim)
		return 0;

	/* allocate sim event/msg buffers */
	err = nvgpu_alloc_sim_buffer(g, &g->sim->send_bfr);
	err = err || nvgpu_alloc_sim_buffer(g, &g->sim->recv_bfr);
	err = err || nvgpu_alloc_sim_buffer(g, &g->sim->msg_bfr);

	if (err)
		goto fail;

	g->sim->sim_init_late = nvgpu_sim_init_late;
	g->sim->remove_support = nvgpu_remove_sim_support;
	g->sim->esc_readl = nvgpu_sim_esc_readl;
	return 0;

 fail:
	nvgpu_free_sim_support(g);
	return err;
}
