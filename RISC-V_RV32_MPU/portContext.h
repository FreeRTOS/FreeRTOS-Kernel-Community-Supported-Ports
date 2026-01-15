/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef PORTCONTEXT_H
#define PORTCONTEXT_H

#ifndef configENABLE_FPU
    #define configENABLE_FPU 0
#endif

#ifndef configENABLE_VPU
    #define configENABLE_VPU 0
#endif

#define store_x          sw
#define load_x           lw
#define portWORD_SIZE    4

#include "freertos_risc_v_chip_specific_extensions.h"

/* Only the standard core registers are stored by default.  Any additional
 * registers must be saved by the portasmSAVE_ADDITIONAL_REGISTERS and
 * portasmRESTORE_ADDITIONAL_REGISTERS macros - which can be defined in a chip
 * specific version of freertos_risc_v_chip_specific_extensions.h.  See the
 * notes at the top of portASM.S file. */
#define portCONTEXT_SIZE               ( 34 * portWORD_SIZE )

#if ( configENABLE_FPU == 1 )
    /* Bit [14:13] in the mstatus encode the status of FPU state which is one of
     * the following values:
     * 1. Value: 0, Meaning: Off.
     * 2. Value: 1, Meaning: Initial.
     * 3. Value: 2, Meaning: Clean.
     * 4. Value: 3, Meaning: Dirty.
     */
    #define MSTATUS_FS_MASK                 0x6000
    #define MSTATUS_FS_INITIAL              0x2000
    #define MSTATUS_FS_CLEAN                0x4000
    #define MSTATUS_FS_DIRTY                0x6000
    #define MSTATUS_FS_OFFSET               13

    #ifdef __riscv_fdiv
        #if __riscv_flen == 32
            #define load_f                  flw
            #define store_f                 fsw
        #elif __riscv_flen == 64
            #define load_f                  fld
            #define store_f                 fsd
        #else
            #error Assembler did not define __riscv_flen
        #endif

        #define portFPU_REG_SIZE                ( __riscv_flen / 8 )
        #define portFPU_REG_COUNT               33 /* 32 Floating point registers plus one CSR. */
        #define portFPU_REG_OFFSET( regIndex )  ( ( 2 * portWORD_SIZE ) + ( regIndex * portFPU_REG_SIZE ) )
        #define portFPU_CONTEXT_SIZE            ( portFPU_REG_SIZE * portFPU_REG_COUNT )
    #else
        #error configENABLE_FPU must not be set to 1 if the hardware does not have FPU
    #endif
#endif

#if ( configENABLE_VPU == 1 )
    /* Bit [10:9] in the mstatus encode the status of VPU state which is one of
     * the following values:
     * 1. Value: 0, Meaning: Off.
     * 2. Value: 1, Meaning: Initial.
     * 3. Value: 2, Meaning: Clean.
     * 4. Value: 3, Meaning: Dirty.
     */
    #define MSTATUS_VS_MASK                 0x600
    #define MSTATUS_VS_INITIAL              0x200
    #define MSTATUS_VS_CLEAN                0x400
    #define MSTATUS_VS_DIRTY                0x600
    #define MSTATUS_VS_OFFSET               9

    #ifndef __riscv_vector
        #error configENABLE_VPU must not be set to 1 if the hardware does not have VPU
    #endif
#endif
/*-----------------------------------------------------------*/

.extern pxCurrentTCB
.extern xISRStackTop
.extern vPortRestorePMPSettings
/*-----------------------------------------------------------*/
    .macro portcontextM_MODE_ENTER
/* Switch to kernel stack if we are from U-mode */

/*
 * We choose to scratch tp here because we can restore it:
 * - tp <- 1 if we were in U-mode
 * - tp <- 0 if we were in M-mode
 */
csrr    tp, mstatus
srli    tp, tp, 11
andi    tp, tp, 3 /* tp = MPP (11=M, 00=U) */
bnez    tp, 1f

/* From U-mode */
load_x tp, pxCurrentTCB         /* Load pxCurrentTCB. */
store_x sp, 0 ( tp )            /* Write Task Stack to the first TCB member */
load_x sp, portWORD_SIZE ( tp ) /* Read Kernel Stack from the second TCB member. */
li tp, 1
j 2f
1:
/* From M-mode */
li tp, 0
2:
    .endm

    .macro portcontextM_MODE_EXIT
/* Switch to task stack if we are returning to U-mode */

/*
 * We choose to scratch tp here because we can restore it:
 * - tp <- 1 if we will eret to U-mode
 * - tp <- 0 if we will eret to M-mode
 */
csrr    tp, mstatus
srli    tp, tp, 11
andi    tp, tp, 3 /* tp = MPP (11=M, 00=U) */
bnez    tp, 1f

/* To U-mode */
load_x tp, pxCurrentTCB             /* Load pxCurrentTCB. */
store_x sp, portWORD_SIZE ( tp )    /* Write Kenrel Stack to the second TCB member */
load_x sp, 0 ( tp )                 /* Read Task Stack from the first TCB member. */
li tp, 1
j 2f
1:
/* To M-mode */
li tp, 0
2:
    .endm

    .macro portcontextLOAD_KERNEL_SP_FROM_TCB
load_x sp, pxCurrentTCB             /* Load pxCurrentTCB. */
load_x sp, portWORD_SIZE ( sp )     /* Read Kernel Stack from the second TCB member. */
    .endm
/*-----------------------------------------------------------*/

    .macro portcontexSAVE_FPU_CONTEXT
addi sp, sp, -( portFPU_CONTEXT_SIZE )
/* Store the FPU registers. */
store_f f0,  portFPU_REG_OFFSET( 0  )( sp )
store_f f1,  portFPU_REG_OFFSET( 1  )( sp )
store_f f2,  portFPU_REG_OFFSET( 2  )( sp )
store_f f3,  portFPU_REG_OFFSET( 3  )( sp )
store_f f4,  portFPU_REG_OFFSET( 4  )( sp )
store_f f5,  portFPU_REG_OFFSET( 5  )( sp )
store_f f6,  portFPU_REG_OFFSET( 6  )( sp )
store_f f7,  portFPU_REG_OFFSET( 7  )( sp )
store_f f8,  portFPU_REG_OFFSET( 8  )( sp )
store_f f9,  portFPU_REG_OFFSET( 9  )( sp )
store_f f10, portFPU_REG_OFFSET( 10 )( sp )
store_f f11, portFPU_REG_OFFSET( 11 )( sp )
store_f f12, portFPU_REG_OFFSET( 12 )( sp )
store_f f13, portFPU_REG_OFFSET( 13 )( sp )
store_f f14, portFPU_REG_OFFSET( 14 )( sp )
store_f f15, portFPU_REG_OFFSET( 15 )( sp )
store_f f16, portFPU_REG_OFFSET( 16 )( sp )
store_f f17, portFPU_REG_OFFSET( 17 )( sp )
store_f f18, portFPU_REG_OFFSET( 18 )( sp )
store_f f19, portFPU_REG_OFFSET( 19 )( sp )
store_f f20, portFPU_REG_OFFSET( 20 )( sp )
store_f f21, portFPU_REG_OFFSET( 21 )( sp )
store_f f22, portFPU_REG_OFFSET( 22 )( sp )
store_f f23, portFPU_REG_OFFSET( 23 )( sp )
store_f f24, portFPU_REG_OFFSET( 24 )( sp )
store_f f25, portFPU_REG_OFFSET( 25 )( sp )
store_f f26, portFPU_REG_OFFSET( 26 )( sp )
store_f f27, portFPU_REG_OFFSET( 27 )( sp )
store_f f28, portFPU_REG_OFFSET( 28 )( sp )
store_f f29, portFPU_REG_OFFSET( 29 )( sp )
store_f f30, portFPU_REG_OFFSET( 30 )( sp )
store_f f31, portFPU_REG_OFFSET( 31 )( sp )
csrr t0, fcsr
store_x t0,  portFPU_REG_OFFSET( 32 )( sp )
    .endm
/*-----------------------------------------------------------*/

    .macro portcontextRESTORE_FPU_CONTEXT
/* Restore the FPU registers. */
load_f f0,  portFPU_REG_OFFSET( 0  )( sp )
load_f f1,  portFPU_REG_OFFSET( 1  )( sp )
load_f f2,  portFPU_REG_OFFSET( 2  )( sp )
load_f f3,  portFPU_REG_OFFSET( 3  )( sp )
load_f f4,  portFPU_REG_OFFSET( 4  )( sp )
load_f f5,  portFPU_REG_OFFSET( 5  )( sp )
load_f f6,  portFPU_REG_OFFSET( 6  )( sp )
load_f f7,  portFPU_REG_OFFSET( 7  )( sp )
load_f f8,  portFPU_REG_OFFSET( 8  )( sp )
load_f f9,  portFPU_REG_OFFSET( 9  )( sp )
load_f f10, portFPU_REG_OFFSET( 10 )( sp )
load_f f11, portFPU_REG_OFFSET( 11 )( sp )
load_f f12, portFPU_REG_OFFSET( 12 )( sp )
load_f f13, portFPU_REG_OFFSET( 13 )( sp )
load_f f14, portFPU_REG_OFFSET( 14 )( sp )
load_f f15, portFPU_REG_OFFSET( 15 )( sp )
load_f f16, portFPU_REG_OFFSET( 16 )( sp )
load_f f17, portFPU_REG_OFFSET( 17 )( sp )
load_f f18, portFPU_REG_OFFSET( 18 )( sp )
load_f f19, portFPU_REG_OFFSET( 19 )( sp )
load_f f20, portFPU_REG_OFFSET( 20 )( sp )
load_f f21, portFPU_REG_OFFSET( 21 )( sp )
load_f f22, portFPU_REG_OFFSET( 22 )( sp )
load_f f23, portFPU_REG_OFFSET( 23 )( sp )
load_f f24, portFPU_REG_OFFSET( 24 )( sp )
load_f f25, portFPU_REG_OFFSET( 25 )( sp )
load_f f26, portFPU_REG_OFFSET( 26 )( sp )
load_f f27, portFPU_REG_OFFSET( 27 )( sp )
load_f f28, portFPU_REG_OFFSET( 28 )( sp )
load_f f29, portFPU_REG_OFFSET( 29 )( sp )
load_f f30, portFPU_REG_OFFSET( 30 )( sp )
load_f f31, portFPU_REG_OFFSET( 31 )( sp )
load_x t0,  portFPU_REG_OFFSET( 32 )( sp )
csrw fcsr, t0
addi sp, sp, ( portFPU_CONTEXT_SIZE )
    .endm
/*-----------------------------------------------------------*/

    .macro portcontexSAVE_VPU_CONTEXT
csrr t0, vlenb /* t0 = vlenb. vlenb is the length of each vector register in bytes. */
slli t0, t0, 3 /* t0 = vlenb * 8. t0 now contains the space required to store 8 vector registers. */
neg  t0, t0

/* Store the vector registers in group of 8. */
add     sp, sp, t0
vs8r.v  v0, (sp)    /* Store v0-v7. */
add     sp, sp, t0
vs8r.v  v8, (sp)    /* Store v8-v15. */
add     sp, sp, t0
vs8r.v  v16, (sp)   /* Store v16-v23. */
add     sp, sp, t0
vs8r.v  v24, (sp)   /* Store v24-v31. */

/* Store the VPU CSRs. */
addi    sp, sp, -( 4 * portWORD_SIZE )
csrr    t0, vstart
store_x t0, 0 * portWORD_SIZE( sp )
csrr    t0, vcsr
store_x t0, 1 * portWORD_SIZE( sp )
csrr    t0, vl
store_x t0, 2 * portWORD_SIZE( sp )
csrr    t0, vtype
store_x t0, 3 * portWORD_SIZE( sp )
    .endm
/*-----------------------------------------------------------*/

    .macro portcontextRESTORE_VPU_CONTEXT
/* Restore the VPU CSRs. */
load_x  t0, 0  * portWORD_SIZE( sp )
csrw    vstart, t0
load_x  t0, 1 * portWORD_SIZE( sp )
csrw    vcsr, t0
load_x  t0, 2 * portWORD_SIZE( sp )
load_x  t1, 3 * portWORD_SIZE( sp )
vsetvl  x0, t0, t1 /* vlen and vtype can only be updated by using vset*vl* instructions. */
addi    sp, sp, ( 4 * portWORD_SIZE )

csrr t0, vlenb /* t0 = vlenb. vlenb is the length of each vector register in bytes. */
slli t0, t0, 3 /* t0 = vlenb * 8. t0 now contains the space required to store 8 vector registers. */

/* Restore the vector registers. */
vl8r.v  v24, (sp)
add     sp, sp, t0
vl8r.v  v16, (sp)
add     sp, sp, t0
vl8r.v  v8, (sp)
add     sp, sp, t0
vl8r.v  v0, (sp)
add     sp, sp, t0
    .endm
/*-----------------------------------------------------------*/

   .macro portcontextSAVE_CONTEXT_TO_SP
#if( configENABLE_FPU == 1 )

    /* temporary save t0, t1 */
    store_x t0, -1 * portWORD_SIZE( sp )
    store_x t1, -2 * portWORD_SIZE( sp )

    csrr t0, mstatus
    srl t0, t0, MSTATUS_FS_OFFSET
    andi t0, t0, 3
    addi t1, x0, 3
    bne t0, t1, 1f /* If FPU status is not dirty, do not save FPU registers. */

    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
    portcontexSAVE_FPU_CONTEXT
    j 2f
1:
    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
2:
#endif

#if( configENABLE_VPU == 1 )

    /* temporary save t0, t1 */
    store_x t0, -1 * portWORD_SIZE( sp )
    store_x t1, -2 * portWORD_SIZE( sp )

    csrr t0, mstatus
    srl t0, t0, MSTATUS_VS_OFFSET
    andi t0, t0, 3
    addi t1, x0, 3
    bne t0, t1, 3f /* If VPU status is not dirty, do not save FPU registers. */

    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
    portcontexSAVE_VPU_CONTEXT
    j 4f
3:
    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
4:
#endif

portasmSAVE_ADDITIONAL_REGISTERS /* Defined in freertos_risc_v_chip_specific_extensions.h to save any registers unique to the RISC-V implementation. */

addi sp, sp, -portCONTEXT_SIZE
store_x x1,  3  * portWORD_SIZE( sp )
//store_x x2,  4  * portWORD_SIZE( sp ) /* Don't do sp */
//store_x x3,  5  * portWORD_SIZE( sp ) /* Don't do gp */
store_x x4,  6  * portWORD_SIZE( sp )
store_x x5,  7  * portWORD_SIZE( sp )
store_x x6,  8  * portWORD_SIZE( sp )
store_x x7,  9  * portWORD_SIZE( sp )
store_x x8,  10  * portWORD_SIZE( sp )
store_x x9,  11  * portWORD_SIZE( sp )
store_x x10, 12  * portWORD_SIZE( sp )
store_x x11, 13  * portWORD_SIZE( sp )
store_x x12, 14 * portWORD_SIZE( sp )
store_x x13, 15 * portWORD_SIZE( sp )
store_x x14, 16 * portWORD_SIZE( sp )
store_x x15, 17 * portWORD_SIZE( sp )
store_x x16, 18 * portWORD_SIZE( sp )
store_x x17, 19 * portWORD_SIZE( sp )
store_x x18, 20 * portWORD_SIZE( sp )
store_x x19, 21 * portWORD_SIZE( sp )
store_x x20, 22 * portWORD_SIZE( sp )
store_x x21, 23 * portWORD_SIZE( sp )
store_x x22, 24 * portWORD_SIZE( sp )
store_x x23, 25 * portWORD_SIZE( sp )
store_x x24, 26 * portWORD_SIZE( sp )
store_x x25, 27 * portWORD_SIZE( sp )
store_x x26, 28 * portWORD_SIZE( sp )
store_x x27, 29 * portWORD_SIZE( sp )
store_x x28, 30 * portWORD_SIZE( sp )
store_x x29, 31 * portWORD_SIZE( sp )
store_x x30, 32 * portWORD_SIZE( sp )
store_x x31, 33 * portWORD_SIZE( sp )

csrr t0, mscratch
store_x t0, 2 * portWORD_SIZE( sp )

csrr t0, mstatus
store_x t0, 1 * portWORD_SIZE( sp )

#if( configENABLE_FPU == 1 )
    /* Mark the FPU as clean, if it was dirty and we saved FPU registers. */
    srl t1, t0, MSTATUS_FS_OFFSET
    andi t1, t1, 3
    addi t2, x0, 3
    bne t1, t2, 3f

    li t1, ~MSTATUS_FS_MASK
    and t0, t0, t1
    li t1, MSTATUS_FS_CLEAN
    or t0, t0, t1
    csrw mstatus, t0
3:
#endif

#if( configENABLE_VPU == 1 )
    /* Mark the VPU as clean, if it was dirty and we saved VPU registers. */
    srl t1, t0, MSTATUS_VS_OFFSET
    andi t1, t1, 3
    addi t2, x0, 3
    bne t1, t2, 4f

    li t1, ~MSTATUS_VS_MASK
    and t0, t0, t1
    li t1, MSTATUS_VS_CLEAN
    or t0, t0, t1
    csrw mstatus, t0
4:
#endif

load_x t0, pxCurrentTCB          /* Load pxCurrentTCB. */
store_x sp, portWORD_SIZE ( t0 ) /* Write Kernel Stack to the second TCB member. */

   .endm
/*-----------------------------------------------------------*/

   .macro portcontextSAVE_EXCEPTION_CONTEXT
portcontextM_MODE_ENTER
portcontextSAVE_CONTEXT_TO_SP
li tp, 0 /* indicate that we are in M-mode now */
csrr a0, mcause
csrr a1, mepc
addi a1, a1, 4          /* Synchronous so update exception return address to the instruction after the instruction that generated the exception. */
store_x a1, 0 ( sp )    /* Save updated exception return address. */
   .endm
/*-----------------------------------------------------------*/

   .macro portcontextSAVE_INTERRUPT_CONTEXT
portcontextM_MODE_ENTER
portcontextSAVE_CONTEXT_TO_SP
li tp, 0 /* indicate that we are in M-mode now */
csrr a0, mcause
csrr a1, mepc
store_x a1, 0 ( sp )    /* Asynchronous interrupt so save unmodified exception return address. */
load_x sp, xISRStackTop /* Switch to ISR stack. */
   .endm
/*-----------------------------------------------------------*/

   .macro portcontextRESTORE_CONTEXT_FROM_SP

/* Load mepc with the address of the instruction in the task to run next. */
load_x t0, 0 ( sp )
csrw mepc, t0

/* Restore mstatus register. */
load_x t0, 1 * portWORD_SIZE( sp )
csrw mstatus, t0

/* Restore mscratch register. */
load_x t0, 2 * portWORD_SIZE( sp )
csrw mscratch, t0

load_x x1,  3  * portWORD_SIZE( sp )
//load_x x2,  4  * portWORD_SIZE( sp ) /* Don't do sp */
//load_x x3,  5  * portWORD_SIZE( sp ) /* Don't do gp */
load_x x4,  6  * portWORD_SIZE( sp )
load_x x5,  7  * portWORD_SIZE( sp )
load_x x6,  8  * portWORD_SIZE( sp )
load_x x7,  9  * portWORD_SIZE( sp )
load_x x8,  10  * portWORD_SIZE( sp )
load_x x9,  11  * portWORD_SIZE( sp )
load_x x10, 12  * portWORD_SIZE( sp )
load_x x11, 13  * portWORD_SIZE( sp )
load_x x12, 14 * portWORD_SIZE( sp )
load_x x13, 15 * portWORD_SIZE( sp )
load_x x14, 16 * portWORD_SIZE( sp )
load_x x15, 17 * portWORD_SIZE( sp )
load_x x16, 18 * portWORD_SIZE( sp )
load_x x17, 19 * portWORD_SIZE( sp )
load_x x18, 20 * portWORD_SIZE( sp )
load_x x19, 21 * portWORD_SIZE( sp )
load_x x20, 22 * portWORD_SIZE( sp )
load_x x21, 23 * portWORD_SIZE( sp )
load_x x22, 24 * portWORD_SIZE( sp )
load_x x23, 25 * portWORD_SIZE( sp )
load_x x24, 26 * portWORD_SIZE( sp )
load_x x25, 27 * portWORD_SIZE( sp )
load_x x26, 28 * portWORD_SIZE( sp )
load_x x27, 29 * portWORD_SIZE( sp )
load_x x28, 30 * portWORD_SIZE( sp )
load_x x29, 31 * portWORD_SIZE( sp )
load_x x30, 32 * portWORD_SIZE( sp )
load_x x31, 33 * portWORD_SIZE( sp )
addi sp, sp, portCONTEXT_SIZE

/* Defined in freertos_risc_v_chip_specific_extensions.h to restore any registers unique to the RISC-V implementation. */
portasmRESTORE_ADDITIONAL_REGISTERS

#if( configENABLE_VPU == 1 )

    /* temporary save t0, t1 */
    store_x t0, -1 * portWORD_SIZE( sp )
    store_x t1, -2 * portWORD_SIZE( sp )

    csrr t0, mstatus
    srl t0, t0, MSTATUS_VS_OFFSET
    andi t0, t0, 3
    addi t1, x0, 3
    bne t0, t1, 5f /* If VPU status is not dirty, do not restore VPU registers. */

    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
    portcontextRESTORE_VPU_CONTEXT
    j 6f
5:
    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
6:
#endif /* ifdef portasmSTORE_VPU_CONTEXT */

#if( configENABLE_FPU == 1 )

    /* temporary save t0, t1 */
    store_x t0, -1 * portWORD_SIZE( sp )
    store_x t1, -2 * portWORD_SIZE( sp )

    csrr t0, mstatus
    srl t1, t0, MSTATUS_FS_OFFSET
    andi t1, t1, 3
    addi t2, x0, 3
    bne t0, t1, 7f /* If FPU status is not dirty, do not restore FPU registers. */

    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
    portcontextRESTORE_FPU_CONTEXT
    j 8f
7:
    /* restore t0, t1 */
    load_x t0, -1 * portWORD_SIZE( sp )
    load_x t1, -2 * portWORD_SIZE( sp )
8:
#endif /* ifdef portasmSTORE_FPU_CONTEXT */
   .endm
/*-----------------------------------------------------------*/

    .macro portcontextRESTORE_EXCEPTION_CONTEXT
call vPortRestorePMPSettings        /* Do this before switching to Task Stack */
portcontextRESTORE_CONTEXT_FROM_SP
portcontextM_MODE_EXIT
    .endm

    .macro portcontextRESTORE_INTERRUPT_CONTEXT
/* we are using IRQ stack now, while the context is stored on kernel stack */
portcontextLOAD_KERNEL_SP_FROM_TCB
call vPortRestorePMPSettings        /* Do this before switching to Task Stack */
portcontextRESTORE_CONTEXT_FROM_SP
portcontextM_MODE_EXIT
    .endm

#endif /* PORTCONTEXT_H */
