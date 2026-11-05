/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef RH850_ASM_H
#define RH850_ASM_H

/* GCC v850 puts __USER_LABEL_PREFIX__ ("_") in front of C symbols */
#define RH850_CAT2( a, b )    a ## b
#define RH850_CAT( a, b )     RH850_CAT2( a, b )
#define C_SYM( s )            RH850_CAT( __USER_LABEL_PREFIX__, s )

/* *INDENT-OFF* */
#ifdef __ASSEMBLER__
/* rh850_stsr RH850_SR_x, reg: RH850_SR_x is "regID, selID" */
	.macro	rh850_stsr id, sel, reg
	stsr	\id, \reg, \sel
	.endm
#endif
/* *INDENT-ON* */

#endif /* RH850_ASM_H */
