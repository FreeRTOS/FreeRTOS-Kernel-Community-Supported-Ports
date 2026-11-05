/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * FreeRTOS port for RH850 G4 cores, GCC: context switch and EIINT entry.
 * HW = U2C HW UM R01UH1018EJ0100.
 */

#include "rh850_asm.h"
#include "rh850_sysreg.h"
#include "core.h"
#include "FreeRTOSConfig.h"
#include "portframe.h"

/*
 * FPSR and FPEPC of the frame at sp. While FPEC.FPIVD = 1 an FPI is pending
 * (HW 3.2.6.1 (11) p361, Table 3.54 p250): FPEPC (Table 3.50 p247) and the
 * FPSR cause bits (Table 3.49 p245) are kept for it.
 */
	.macro	port_fpu_restore fpsr, fpepc
	ld.w	\fpsr[sp], r10
#if RH850_CORE_HAS_FPEC
	rh850_stsr	RH850_SR_FPEC, r11
	andi	RH850_FPEC_FPIVD, r11, r11
	bz	8f
	rh850_stsr	RH850_SR_FPSR, r11
	andi	RH850_FPSR_XC_MASK, r11, r11
	andi	RH850_FPSR_XC_MASK, r10, r12
	xor	r12, r10
	or	r11, r10
	ldsr	r10, RH850_SR_FPSR
	br	9f
8:
#endif
	ldsr	r10, RH850_SR_FPSR
	ld.w	\fpepc[sp], r10
	ldsr	r10, RH850_SR_FPEPC
9:
	.endm

/* Full task frame on the task stack; sp goes to pxCurrentTCB */
	.macro	port_save
#if portUSE_FPU && RH850_CORE_HAS_SYNCE
	/* FPU results and FPI reports of the task are complete before FPSR is read */
	synce
#endif
	addi	-portFRAME_SIZE, sp, sp
	st.w	r1, portFRAME_R1[sp]
	st.w	r2, portFRAME_R2[sp]
	.irp	n, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, \
		19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	st.w	r\n, portFRAME_R(\n)[sp]
	.endr
	rh850_stsr	RH850_SR_EIPC, r10
	st.w	r10, portFRAME_EIPC[sp]
	rh850_stsr	RH850_SR_EIPSW, r10
	st.w	r10, portFRAME_EIPSW[sp]
#if portUSE_FPU
	rh850_stsr	RH850_SR_FPSR, r10
	st.w	r10, portFRAME_FPSR[sp]
	rh850_stsr	RH850_SR_FPEPC, r10
	st.w	r10, portFRAME_FPEPC[sp]
#endif
	rh850_stsr	RH850_CORE_MASK_SR, r10
	st.w	r10, portFRAME_MASK[sp]
	mov	hilo(C_SYM(ulPortCriticalNesting)), r10
	ld.w	0[r10], r10
	st.w	r10, portFRAME_CRITICAL[sp]
	mov	hilo(C_SYM(pxCurrentTCB)), r10
	ld.w	0[r10], r10
	st.w	sp, 0[r10]
	.endm

/* Frame of pxCurrentTCB, including its mask, then EIRET */
	.macro	port_restore
	mov	hilo(C_SYM(pxCurrentTCB)), r10
	ld.w	0[r10], r10
	ld.w	0[r10], sp
	ld.w	portFRAME_CRITICAL[sp], r10
	mov	hilo(C_SYM(ulPortCriticalNesting)), r11
	st.w	r10, 0[r11]
	ld.w	portFRAME_MASK[sp], r10
	ldsr	r10, RH850_CORE_MASK_SR
#if portUSE_FPU
	port_fpu_restore	portFRAME_FPSR, portFRAME_FPEPC
#endif
	ld.w	portFRAME_EIPSW[sp], r10
	ldsr	r10, RH850_SR_EIPSW
	ld.w	portFRAME_EIPC[sp], r10
	ldsr	r10, RH850_SR_EIPC
	ld.w	portFRAME_R1[sp], r1
	ld.w	portFRAME_R2[sp], r2
	.irp	n, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, \
		19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	ld.w	portFRAME_R(\n)[sp], r\n
	.endr
	addi	portFRAME_SIZE, sp, sp
	eiret
	.endm

/* Nested interrupt: return state and GCC call-used registers (ISR stack) */
	.macro	isr_save
#if portUSE_FPU && RH850_CORE_HAS_SYNCE
	synce
#endif
	addi	-portISR_SIZE, sp, sp
	st.w	r1, portISR_R1[sp]
	st.w	r2, portISR_R2[sp]
	st.w	r5, portISR_R5[sp]
	.irp	n, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
	st.w	r\n, portISR_R(\n)[sp]
	.endr
	st.w	r30, portISR_R30[sp]
	st.w	r31, portISR_R31[sp]
	rh850_stsr	RH850_SR_EIPC, r10
	st.w	r10, portISR_EIPC[sp]
	rh850_stsr	RH850_SR_EIPSW, r10
	st.w	r10, portISR_EIPSW[sp]
#if portUSE_FPU
	rh850_stsr	RH850_SR_FPSR, r10
	st.w	r10, portISR_FPSR[sp]
	rh850_stsr	RH850_SR_FPEPC, r10
	st.w	r10, portISR_FPEPC[sp]
#endif
	.endm

	.macro	isr_restore
#if portUSE_FPU
	port_fpu_restore	portISR_FPSR, portISR_FPEPC
#endif
	ld.w	portISR_EIPSW[sp], r10
	ldsr	r10, RH850_SR_EIPSW
	ld.w	portISR_EIPC[sp], r10
	ldsr	r10, RH850_SR_EIPC
	ld.w	portISR_R1[sp], r1
	ld.w	portISR_R2[sp], r2
	ld.w	portISR_R5[sp], r5
	.irp	n, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
	ld.w	portISR_R(\n)[sp], r\n
	.endr
	ld.w	portISR_R30[sp], r30
	ld.w	portISR_R31[sp], r31
	addi	portISR_SIZE, sp, sp
	eiret
	.endm

	.section .text.port, "ax"

	/* TRAP0 (040H), taskYIELD: PSW.ID = 1 for the whole switch */
	.global	rh850_trap0_entry
rh850_trap0_entry:
	port_save
	jarl	C_SYM(vTaskSwitchContext), lp
	port_restore

	/*
	 * EIINT sets PSW.ID (HW Table 3.95 p288). EIPC/EIPSW are saved before EI,
	 * since a nested interrupt overwrites them (HW 3.2.4.1 (7) p298).
	 */
	.global	rh850_eiint_entry
rh850_eiint_entry:
	/* Two scratch registers on the current stack to test the depth */
	addi	-8, sp, sp
	st.w	r10, 0[sp]
	st.w	r11, 4[sp]
	mov	hilo(C_SYM(ulPortInterruptNesting)), r10
	ld.w	0[r10], r11
	cmp	r0, r11
	ld.w	0[sp], r10
	ld.w	4[sp], r11
	/* movea keeps the flags of cmp */
	movea	8, sp, sp
	bne	1f

	/* Outermost: task frame on the task stack, then the ISR stack */
	port_save
	mov	hilo(C_SYM(pxPortISRStackTop)), r10
	ld.w	0[r10], sp
	br	2f

	/* Nested: already on the ISR stack */
1:	isr_save

2:	mov	hilo(C_SYM(ulPortInterruptNesting)), r10
	ld.w	0[r10], r11
	add	1, r11
	st.w	r11, 0[r10]
	/* Read EIIC before EI: a nested interrupt overwrites it */
	rh850_stsr	RH850_SR_EIIC, r6
	ei
	jarl	C_SYM(vApplicationIRQHandler), lp
	di
	mov	hilo(C_SYM(ulPortInterruptNesting)), r10
	ld.w	0[r10], r11
	add	-1, r11
	st.w	r11, 0[r10]
	cmp	r0, r11
	bne	3f

	/* Outermost exit: switch if asked, then EIRET with EP = 0 clears ISPR (HW 3.2.4.3 p302-303) */
#if portUSE_FPU && RH850_CORE_HAS_FPEC
	/*
	 * A pending FPI of this task is taken after EIRET, so keep the task and
	 * switch at a later exit (HW 3.2.6.1 (11) p361, FPEC p250).
	 */
	rh850_stsr	RH850_SR_FPEC, r11
	andi	RH850_FPEC_FPIVD, r11, r11
	bne	4f
#endif
	mov	hilo(C_SYM(ulPortYieldPending)), r10
	ld.w	0[r10], r11
	cmp	r0, r11
	be	4f
	st.w	r0, 0[r10]
	jarl	C_SYM(vTaskSwitchContext), lp
4:	port_restore

3:	isr_restore

	/*
	 * Start the first task with ID = 1. EP = 1 so that this EIRET does not
	 * end an interrupt in the INTC (HW 3.2.4.3 p302-303).
	 */
	.global	C_SYM(vPortStartFirstTask)
C_SYM(vPortStartFirstTask):
	di
	rh850_stsr	RH850_SR_PSW, r10
	ori	RH850_PSW_EP, r10, r10
	ldsr	r10, RH850_SR_PSW
	port_restore

#if (configUSE_TICKLESS_IDLE == 1)
	/* PSW.ID for the tickless sleep in port.c (kept in this file) */
	.global	C_SYM(vPortIrqDisable)
C_SYM(vPortIrqDisable):
	di
	jmp	[lp]

	.global	C_SYM(vPortIrqEnable)
C_SYM(vPortIrqEnable):
	ei
	jmp	[lp]
#endif
