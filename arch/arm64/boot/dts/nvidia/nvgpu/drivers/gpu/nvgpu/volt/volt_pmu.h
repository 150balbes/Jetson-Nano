/*
* Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_VOLT_PMU_H
#define NVGPU_VOLT_PMU_H

u32 volt_pmu_send_load_cmd_to_pmu(struct gk20a *g);
u32 volt_set_voltage(struct gk20a *g, u32 logic_voltage_uv,
		u32 sram_voltage_uv);
u32 volt_get_voltage(struct gk20a *g, u32 volt_domain, u32 *voltage_uv);
int volt_set_noiseaware_vmin(struct gk20a *g, u32 logic_voltage_uv,
		u32 sram_voltage_uv);

u32 nvgpu_volt_set_voltage_gp10x(struct gk20a *g, u32 logic_voltage_uv,
	u32 sram_voltage_uv);
u32 nvgpu_volt_rail_get_voltage_gp10x(struct gk20a *g,
	u8 volt_domain, u32 *pvoltage_uv);
u32 nvgpu_volt_send_load_cmd_to_pmu_gp10x(struct gk20a *g);

u32 nvgpu_volt_set_voltage_gv10x(struct gk20a *g, u32 logic_voltage_uv,
	u32 sram_voltage_uv);
u32 nvgpu_volt_rail_get_voltage_gv10x(struct gk20a *g,
	u8 volt_domain, u32 *pvoltage_uv);
u32 nvgpu_volt_send_load_cmd_to_pmu_gv10x(struct gk20a *g);


#endif /* NVGPU_VOLT_PMU_H */
