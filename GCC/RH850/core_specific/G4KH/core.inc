/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * G4KH reset-time register init, as asm macros. The core registers that
 * are undefined after reset (U2C HW UM 3.8.11, Table 3.199 p478).
 */
#include "core.h"

/* Memory error and register bank registers */
	.macro	rh850_core_clear_error_regs
	ldsr	r0, RH850_SR_MEA
	ldsr	r0, RH850_SR_MEI
	ldsr	r0, RH850_SR_RBIP
	.endm

/* FPU registers; PSW.CU0 must be set and in effect (p383) */
	.macro	rh850_core_init_fpu tmp
	ldsr	r0, RH850_SR_FPST
	ldsr	r0, RH850_SR_FPCC
	mov	hilo(RH850_FPSR_FS), \tmp
	ldsr	\tmp, RH850_SR_FPSR
	ldsr	r0, RH850_SR_FPEPC
	.endm

/* All MPU areas, selected one by one with MPIDX, and MPID0-7 */
	.macro	rh850_core_clear_mpu idx, end
	ldsr	r0, RH850_SR_MCA
	ldsr	r0, RH850_SR_MCS
	ldsr	r0, RH850_SR_MCR
	ldsr	r0, RH850_SR_MCI
	mov	r0, \idx
	movea	RH850_CORE_MPU_ENTRIES, r0, \end
1:	ldsr	\idx, RH850_SR_MPIDX
	ldsr	r0, RH850_SR_MPLA
	ldsr	r0, RH850_SR_MPUA
	ldsr	r0, RH850_SR_MPAT
	add	1, \idx
	cmp	\end, \idx
	bl	1b
	ldsr	r0, RH850_SR_MPIDX
	ldsr	r0, RH850_SR_MPID0
	ldsr	r0, RH850_SR_MPID1
	ldsr	r0, RH850_SR_MPID2
	ldsr	r0, RH850_SR_MPID3
	ldsr	r0, RH850_SR_MPID4
	ldsr	r0, RH850_SR_MPID5
	ldsr	r0, RH850_SR_MPID6
	ldsr	r0, RH850_SR_MPID7
	.endm

/* Instruction cache tag, data and error registers */
	.macro	rh850_core_clear_icache
	ldsr	r0, RH850_SR_ICTAGL
	ldsr	r0, RH850_SR_ICTAGH
	ldsr	r0, RH850_SR_ICDATL
	ldsr	r0, RH850_SR_ICDATH
	ldsr	r0, RH850_SR_ICERR
	.endm
