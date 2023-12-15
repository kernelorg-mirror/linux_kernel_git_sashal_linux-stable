/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_CFI_H
#define _ASM_X86_CFI_H

/*
 * Clang Control Flow Integrity (CFI) support.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/cfi.h>
#include <asm/ibt.h>

#ifdef CONFIG_CFI_CLANG
enum bug_trap_type handle_cfi_failure(struct pt_regs *regs);
#else
static inline enum bug_trap_type handle_cfi_failure(struct pt_regs *regs)
{
	return BUG_TRAP_TYPE_NONE;
}
#endif /* CONFIG_CFI_CLANG */

#if HAS_KERNEL_IBT == 1
#define CFI_NOSEAL(x)	asm(IBT_NOSEAL(__stringify(x)))
#endif

#endif /* _ASM_X86_CFI_H */
