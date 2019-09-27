/*
 * GV11B Cycle stats snapshots support
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CSS_GR_GV11B_H
#define CSS_GR_GV11B_H

int gv11b_css_hw_enable_snapshot(struct channel_gk20a *ch,
			struct gk20a_cs_snapshot_client *cs_client);
void gv11b_css_hw_disable_snapshot(struct gr_gk20a *gr);
int gv11b_css_hw_check_data_available(struct channel_gk20a *ch, u32 *pending,
			bool *hw_overflow);
void gv11b_css_hw_set_handled_snapshots(struct gk20a *g, u32 done);
bool gv11b_css_hw_get_overflow_status(struct gk20a *g);
u32 gv11b_css_hw_get_pending_snapshots(struct gk20a *g);


#endif /* CSS_GR_GV11B_H */
