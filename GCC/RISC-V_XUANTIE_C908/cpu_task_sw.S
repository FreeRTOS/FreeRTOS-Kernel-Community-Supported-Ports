 /*
 * Copyright (C) 2017-2024 Alibaba Group Holding Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <riscv_csr.h>
#include <FreeRTOSConfig.h>

.global cpu_is_irq_enable
.global cpu_intrpt_save
.global cpu_intrpt_restore
.global vPortStartTask
.global MachineSoftwareIRQ_Handler

#if configNUMBER_OF_CORES > 1
.extern pxCurrentTCBs
#else
.extern pxCurrentTCB
#endif
.extern vTaskSwitchContext
.extern g_top_irqstack

.equ RISCV_MSTATUS_MIE,        (1<<3)       /*machine-level interrupt bit*/
.equ RISCV_SSTATUS_SIE,        (1<<1)       /*supervisor-level interrupt bit*/

.type   cpu_intrpt_save, %function
cpu_intrpt_save:
    csrrci a0, MODE_PREFIX(status), RISCV_MSTATUS_MIE
    ret
    .size   cpu_intrpt_save, . - cpu_intrpt_save

.type   cpu_is_irq_enable, %function
cpu_is_irq_enable:
    csrr a0, MODE_PREFIX(status)
    andi a0, a0, RISCV_MSTATUS_MIE
    ret
    .size   cpu_is_irq_enable, . - cpu_is_irq_enable

.type   cpu_intrpt_restore, %function
cpu_intrpt_restore:
    csrw MODE_PREFIX(status), a0
    ret
    .size   cpu_intrpt_restore, . - cpu_intrpt_restore

.macro FREERTOS_RESTORE_MSTATUS
    li       t1, 0

    #ifdef CONFIG_RISC_VECTOR_ENABLED
        csrr     t0, vlenb
        slli     t0, t0, 5
        add      t1, t1, t0
        addi     t1, t1, 40
    #endif  /* CONFIG_RISC_VECTOR_ENABLED */

    #if CONFIG_RISCV_FPU_ENABLED
        addi     t1, t1, (256 + 8)
    #endif /* CONFIG_RISCV_FPU_ENABLED */

    addi     t1, t1, (256 - 8)
    add      t1, sp, t1
    ld       t3, (0)(t1)
    csrw     mstatus, t3
.endm

/********************************************************************
 * Functions: vPortYield
 *
 ********************************************************************/
.global vPortYield
.type   vPortYield, %function
vPortYield:
    /* push regs */
    addi    sp, sp, -16
    sd      t0, 0(sp)
    sd      t2, 8(sp)
    /*trigger irq*/
    csrr    t2, mhartid
    slli    t2, t2, 2
    li      t0, CLINT_BASE
    add     t0, t0, t2
    li      t2, 0x1
    sw      t2, 0(t0)
    /* pop regs */
    ld      t0, 0(sp)
    ld      t2, 8(sp)
    addi    sp, sp, 16
    ret

.align 8
vPortStartTask:
    j       __vPortFirstTask

.align 8
MachineSoftwareIRQ_Handler:
    addi    sp, sp, -(128+128)
    sd      x1,  (0  +0  )(sp)
    sd      x3,  (4  +4  )(sp)
    sd      x4,  (8  +8  )(sp)
    sd      x5,  (12 +12 )(sp)
    sd      x6,  (16 +16 )(sp)
    sd      x7,  (20 +20 )(sp)
    sd      x8,  (24 +24 )(sp)
    sd      x9,  (28 +28 )(sp)
    sd      x10, (32 +32 )(sp)
    sd      x11, (36 +36 )(sp)
    sd      x12, (40 +40 )(sp)
    sd      x13, (44 +44 )(sp)
    sd      x14, (48 +48 )(sp)
    sd      x15, (52 +52 )(sp)
    sd      x16, (56 +56 )(sp)
    sd      x17, (60 +60 )(sp)
    sd      x18, (64 +64 )(sp)
    sd      x19, (68 +68 )(sp)
    sd      x20, (72 +72 )(sp)
    sd      x21, (76 +76 )(sp)
    sd      x22, (80 +80 )(sp)
    sd      x23, (84 +84 )(sp)
    sd      x24, (88 +88 )(sp)
    sd      x25, (92 +92 )(sp)
    sd      x26, (96 +96 )(sp)
    sd      x27, (100+100)(sp)
    sd      x28, (104+104)(sp)
    sd      x29, (108+108)(sp)
    sd      x30, (112+112)(sp)
    sd      x31, (116+116)(sp)

    csrr    t0, MODE_PREFIX(epc)
    sd      t0, (120+120)(sp)
    csrr    t3, MODE_PREFIX(status)
    sd      t3, (124+124)(sp)

    #if CONFIG_RISCV_FPU_ENABLED
        li       t1, SR_FS_DIRTY
        and      t4, t3, t1
        bne      t4, t1, 1f

        addi     sp, sp, -(4+4)
        frcsr    t0
        sd       t0, (0  +0  )(sp)

        addi     sp, sp, -(128+128)
        fsd      f31, (0  +0  )(sp)
        fsd      f30, (4  +4  )(sp)
        fsd      f29, (8  +8  )(sp)
        fsd      f28, (12 +12 )(sp)
        fsd      f27, (16 +16 )(sp)
        fsd      f26, (20 +20 )(sp)
        fsd      f25, (24 +24 )(sp)
        fsd      f24, (28 +28 )(sp)
        fsd      f23, (32 +32 )(sp)
        fsd      f22, (36 +36 )(sp)
        fsd      f21, (40 +40 )(sp)
        fsd      f20, (44 +44 )(sp)
        fsd      f19, (48 +48 )(sp)
        fsd      f18, (52 +52 )(sp)
        fsd      f17, (56 +56 )(sp)
        fsd      f16, (60 +60 )(sp)
        fsd      f15, (64 +64 )(sp)
        fsd      f14, (68 +68 )(sp)
        fsd      f13, (72 +72 )(sp)
        fsd      f12, (76 +76 )(sp)
        fsd      f11, (80 +80 )(sp)
        fsd      f10, (84 +84 )(sp)
        fsd      f9,  (88 +88 )(sp)
        fsd      f8,  (92 +92 )(sp)
        fsd      f7,  (96 +96 )(sp)
        fsd      f6,  (100+100)(sp)
        fsd      f5,  (104+104)(sp)
        fsd      f4,  (108+108)(sp)
        fsd      f3,  (112+112)(sp)
        fsd      f2,  (116+116)(sp)
        fsd      f1,  (120+120)(sp)
        fsd      f0,  (124+124)(sp)

        j        2f
        1:
            addi     sp, sp, -264
        2:
    #endif /* CONFIG_RISCV_VECTOR_ENABLED */

    #if CONFIG_RISCV_VECTOR_ENABLED
        li      t1, SR_VS_DIRTY
        and     t4, t3, t1
        bne     t4, t1, 3f
        addi    sp, sp, -(20+20)
        csrr    t0, vl
        sd      t0,  (0  +0  )(sp)
        csrr    t0, vtype
        sd      t0,  (4  +4  )(sp)
        csrr    t0, vstart
        sd      t0,  (8  +8  )(sp)
        csrr    t0, vxsat
        sd      t0,  (12 +12 )(sp)
        csrr    t0, vxrm
        sd      t0,  (16 +16 )(sp)

        csrr     t0, vlenb
        slli     t0, t0, 3
        slli     t1, t0, 2
        sub      sp, sp, t1
        #if (__riscv_v == 7000)
            vsetvli  zero, zero, e8, m8
            vsb.v    v0, (sp)
            add      sp, sp, t0
            vsb.v    v8, (sp)
            add      sp, sp, t0
            vsb.v    v16, (sp)
            add      sp, sp, t0
            vsb.v    v24, (sp)
        #elif (__riscv_v == 1000000)
            vsetvli  zero, zero, e8, m8, ta, ma
            vs8r.v   v0, (sp)
            add      sp, sp, t0
            vs8r.v   v8, (sp)
            add      sp, sp, t0
            vs8r.v   v16, (sp)
            add      sp, sp, t0
            vs8r.v   v24, (sp)
        #endif
        sub      t0, t1, t0
        sub      sp, sp, t0

        j        4f
        3:
        addi     sp, sp, -40
        csrr     t0, vlenb
        slli     t0, t0, 5
        sub      sp, sp, t0
        4:
    #endif /*CONFIG_RISCV_VECTOR_ENABLED */

    #if configNUMBER_OF_CORES > 1
        la       a1, pxCurrentTCBs
        csrr     a0, mhartid

        li       t1, (__riscv_xlen / 8)
        mul      t0, a0, t1
        add      a1, a1, t0
        ld       a1, (a1)
        sd       sp, (a1)

        jal      xPortTaskSwitch
    #else
        la      a1, pxCurrentTCB
        ld      a1, (a1)
        sd      sp, (a1)

        jal     vTaskSwitchContext
    #endif /* configNUMBER_OF_CORES > 1 */

    __vPortFirstTask:
    #if configNUMBER_OF_CORES > 1
        la      a1, pxCurrentTCBs
        csrr    t0, mhartid
        li      t1, (__riscv_xlen / 8)
        mul     t0, t0, t1
        add     a1, a1, t0
    #else  /* configNUMBER_OF_CORES > 1 */
        la      a1, pxCurrentTCB
    #endif   /* configNUMBER_OF_CORES > 1 */
    ld      a1, (a1)
    ld      sp, (a1)

    FREERTOS_RESTORE_MSTATUS

    /* when using SMP, pending bit shoudn't be cleared here */
    #if configNUMBER_OF_CORES == 1
        /*clear software interrupt*/
        li      t0, CLINT_BASE
        csrr    t2, mhartid
        slli    t2, t2, 2
        add     t0, t0, t2
        li      t2, 0x0
        sw      t2, 0(t0)
    #endif

    #if CONFIG_RISCV_VECTOR_ENABLED
        li       t1, SR_VS_DIRTY
        and      t4, t3, t1
        bne      t4, t1, 13f

        csrr     t0, vlenb
        slli     t0, t0, 3
        #if (__riscv_v == 7000)
            vsetvli  zero, zero, e8, m8
            vlb.v    v0, (sp)
            add      sp, sp, t0
            vlb.v    v8, (sp)
            add      sp, sp, t0
            vlb.v    v16, (sp)
            add      sp, sp, t0
            vlb.v    v24, (sp)
            add      sp, sp, t0
        #elif (__riscv_v == 1000000)
            vsetvli  zero, zero, e8, m8, ta, ma
            vl8r.v   v0, (sp)
            add      sp, sp, t0
            vl8r.v   v8, (sp)
            add      sp, sp, t0
            vl8r.v   v16, (sp)
            add      sp, sp, t0
            vl8r.v   v24, (sp)
            add      sp, sp, t0
        #endif
        lwu     t0, (0 +0)(sp)
        lwu     t1, (4 +4)(sp)
        lwu     t2, (8 +8)(sp)
        vsetvl  zero, t0, t1
        csrw    vstart, t2
        lwu     t2, (12 +12)(sp)
        csrw    vxsat, t2
        lwu     t2, (16 +16)(sp)
        csrw    vxrm, t2
        addi    sp, sp, (20+20)
        j       14f
        13:
            /* don't restore, move sp only */
            addi    sp, sp, (20+20)
            csrr    t0, vlenb
            slli    t0, t0, 5
            add     sp, sp, t0
        14:
    #endif /* CONFIG_RISCV_VECTOR_ENABLED */

    #ifdef CONFIG_RISCV_FPU_ENABLED
        li       t1, SR_FS_DIRTY
        and      t4, t3, t1
        bne      t4, t1, 15f
        fld      f31,( 0 + 0 )(sp)
        fld      f30,( 4 + 4 )(sp)
        fld      f29,( 8 + 8 )(sp)
        fld      f28,( 12+ 12)(sp)
        fld      f27,( 16+ 16)(sp)
        fld      f26,( 20+ 20)(sp)
        fld      f25,( 24+ 24)(sp)
        fld      f24,( 28+ 28)(sp)
        fld      f23,( 32+ 32)(sp)
        fld      f22,( 36+ 36)(sp)
        fld      f21,( 40+ 40)(sp)
        fld      f20,( 44+ 44)(sp)
        fld      f19,( 48+ 48)(sp)
        fld      f18,( 52+ 52)(sp)
        fld      f17,( 56+ 56)(sp)
        fld      f16,( 60+ 60)(sp)
        fld      f15,( 64+ 64)(sp)
        fld      f14,( 68+ 68)(sp)
        fld      f13,( 72+ 72)(sp)
        fld      f12,( 76+ 76)(sp)
        fld      f11,( 80+ 80)(sp)
        fld      f10,( 84+ 84)(sp)
        fld      f9, ( 88+ 88)(sp)
        fld      f8, ( 92+ 92)(sp)
        fld      f7, ( 96+ 96)(sp)
        fld      f6, (100+100)(sp)
        fld      f5, (104+104)(sp)
        fld      f4, (108+108)(sp)
        fld      f3, (112+112)(sp)
        fld      f2, (116+116)(sp)
        fld      f1, (120+120)(sp)
        fld      f0, (124+124)(sp)
        addi     sp, sp, (128+128)

        ld       t0, (0 +0)(sp)
        fscsr    t0
        addi     sp, sp, (4+4)

        j        16f
        15:
            addi     sp, sp, 264
        16:
    #endif

    ld      t0, (120 +120)(sp)
    csrw    MODE_PREFIX(epc), t0

    ld     x1,  (0  +0  )(sp)
    ld     x3,  (4  +4  )(sp)
    ld     x4,  (8  +8  )(sp)
    ld     x5,  (12 +12 )(sp)
    ld     x6,  (16 +16 )(sp)
    ld     x7,  (20 +20 )(sp)
    ld     x8,  (24 +24 )(sp)
    ld     x9,  (28 +28 )(sp)
    ld     x10, (32 +32 )(sp)
    ld     x11, (36 +36 )(sp)
    ld     x12, (40 +40 )(sp)
    ld     x13, (44 +44 )(sp)
    ld     x14, (48 +48 )(sp)
    ld     x15, (52 +52 )(sp)
    ld     x16, (56 +56 )(sp)
    ld     x17, (60 +60 )(sp)
    ld     x18, (64 +64 )(sp)
    ld     x19, (68 +68 )(sp)
    ld     x20, (72 +72 )(sp)
    ld     x21, (76 +76 )(sp)
    ld     x22, (80 +80 )(sp)
    ld     x23, (84 +84 )(sp)
    ld     x24, (88 +88 )(sp)
    ld     x25, (92 +92 )(sp)
    ld     x26, (96 +96 )(sp)
    ld     x27, (100+100)(sp)
    ld     x28, (104+104)(sp)
    ld     x29, (108+108)(sp)
    ld     x30, (112+112)(sp)
    ld     x31, (116+116)(sp)
    addi    sp, sp, (128+128)

    MODE_PREFIX(ret)

   .size   MachineSoftwareIRQ_Handler, . - MachineSoftwareIRQ_Handler
/*-----------------------------------------------------------*/

/*
 * Unlike other ports pxPortInitialiseStack() is written in assembly code as it
 * needs access to the portasmADDITIONAL_CONTEXT_SIZE constant.  The prototype
 * for the function is as per the other ports:
 * StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters );
 *
 * As per the standard RISC-V ABI pxTopcOfStack is passed in in a0, pxCode in
 * a1, and pvParameters in a2.  The new top of stack is passed out in a0.
 *
 * RISC-V maps registers to ABI names as follows (X1 to X31 integer registers
 * for the 'I' profile, X1 to X15 for the 'E' profile, currently I assumed).
 *
 * Register		ABI Name	Description						Saver
 * x0			zero		Hard-wired zero					-
 * x1			ra			Return address					Caller
 * x2			sp			Stack pointer					Callee
 * x3			gp			Global pointer					-
 * x4			tp			Thread pointer					-
 * x5-7			t0-2		Temporaries						Caller
 * x8			s0/fp		Saved register/Frame pointer	Callee
 * x9			s1			Saved register					Callee
 * x10-11		a0-1		Function Arguments/return values Caller
 * x12-17		a2-7		Function arguments				Caller
 * x18-27		s2-11		Saved registers					Callee
 * x28-31		t3-6		Temporaries						Caller
 *
 * The RISC-V context is saved t FreeRTOS tasks in the following stack frame,
 * where the global and thread pointers are currently assumed to be constant so
 * are not saved:
 *
 * mstatus
 * x31
 * x30
 * x29
 * x28
 * x27
 * x26
 * x25
 * x24
 * x23
 * x22
 * x21
 * x20
 * x19
 * x18
 * x17
 * x16
 * x15
 * x14
 * x13
 * x12
 * x11
 * pvParameters
 * x9
 * x8
 * x7
 * x6
 * x5
 * portTASK_RETURN_ADDRESS
 * [chip specific registers go here]
 * pxCode
 */

/*-----------------------------------------------------------*/
