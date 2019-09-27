/*
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <nvgpu/types.h>
#include <linux/tegra_gr_comm.h>

#include "os/linux/os_linux.h"

int vgpu_ivc_init(struct gk20a *g, u32 elems,
		const size_t *queue_sizes, u32 queue_start, u32 num_queues)
{
	struct platform_device *pdev = to_platform_device(dev_from_gk20a(g));

	return tegra_gr_comm_init(pdev, elems, queue_sizes, queue_start,
				num_queues);
}

void vgpu_ivc_deinit(u32 queue_start, u32 num_queues)
{
	tegra_gr_comm_deinit(queue_start, num_queues);
}

void vgpu_ivc_release(void *handle)
{
	tegra_gr_comm_release(handle);
}

u32 vgpu_ivc_get_server_vmid(void)
{
	return tegra_gr_comm_get_server_vmid();
}

int vgpu_ivc_recv(u32 index, void **handle, void **data,
				size_t *size, u32 *sender)
{
	return tegra_gr_comm_recv(index, handle, data, size, sender);
}

int vgpu_ivc_send(u32 peer, u32 index, void *data, size_t size)
{
	return tegra_gr_comm_send(peer, index, data, size);
}

int vgpu_ivc_sendrecv(u32 peer, u32 index, void **handle,
				void **data, size_t *size)
{
	return tegra_gr_comm_sendrecv(peer, index, handle, data, size);
}

u32 vgpu_ivc_get_peer_self(void)
{
	return TEGRA_GR_COMM_ID_SELF;
}

void *vgpu_ivc_oob_get_ptr(u32 peer, u32 index, void **ptr,
					size_t *size)
{
	return tegra_gr_comm_oob_get_ptr(peer, index, ptr, size);
}

void vgpu_ivc_oob_put_ptr(void *handle)
{
	tegra_gr_comm_oob_put_ptr(handle);
}
