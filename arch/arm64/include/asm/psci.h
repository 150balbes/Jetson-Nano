/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 ARM Limited
 */

#ifndef __ASM_PSCI_H
#define __ASM_PSCI_H

int psci_init(void);

int __invoke_psci_fn_hvc(u64 function_id, u64 arg0, u64 arg1, u64 arg2);
int __invoke_psci_fn_smc(u64 function_id, u64 arg0, u64 arg1, u64 arg2);

#endif /* __ASM_PSCI_H */
