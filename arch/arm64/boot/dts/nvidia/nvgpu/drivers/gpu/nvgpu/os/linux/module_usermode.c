/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>

#include "os_linux.h"

/*
 * Locks out the driver from accessing GPU registers. This prevents access to
 * thse registers after the GPU has been clock or power gated. This should help
 * find annoying bugs where register reads and writes are silently dropped
 * after the GPU has been turned off. On older chips these reads and writes can
 * also lock the entire CPU up.
 */
void nvgpu_lockout_usermode_registers(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->usermode_regs = NULL;
}

/*
 * Undoes t19x_lockout_registers().
 */
void nvgpu_restore_usermode_registers(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->usermode_regs = l->usermode_regs_saved;
}

void nvgpu_remove_usermode_support(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (l->usermode_regs) {
		l->usermode_regs = NULL;
	}
}

void nvgpu_init_usermode_support(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->usermode_regs = l->regs + usermode_cfg0_r();
	l->usermode_regs_saved = l->usermode_regs;
}
