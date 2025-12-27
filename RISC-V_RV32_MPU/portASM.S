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

/*
 * The FreeRTOS kernel's RISC-V port is split between the the code that is
 * common across all currently supported RISC-V chips (implementations of the
 * RISC-V ISA), and code which tailors the port to a specific RISC-V chip:
 *
 * + The code that is common to all RISC-V chips is implemented in
 *   FreeRTOS\Source\portable\GCC\RISC-V\portASM.S.  There is only one
 *   portASM.S file because the same file is used no matter which RISC-V chip is
 *   in use.
 *
 * + The code that tailors the kernel's RISC-V port to a specific RISC-V
 *   chip is implemented in freertos_risc_v_chip_specific_extensions.h.  There
 *   is one freertos_risc_v_chip_specific_extensions.h that can be used with any
 *   RISC-V chip that both includes a standard CLINT and does not add to the
 *   base set of RISC-V registers.  There are additional
 *   freertos_risc_v_chip_specific_extensions.h files for RISC-V implementations
 *   that do not include a standard CLINT or do add to the base set of RISC-V
 *   registers.
 *
 * CARE MUST BE TAKEN TO INCLDUE THE CORRECT
 * freertos_risc_v_chip_specific_extensions.h HEADER FILE FOR THE CHIP
 * IN USE.  To include the correct freertos_risc_v_chip_specific_extensions.h
 * header file ensure the path to the correct header file is in the assembler's
 * include path.
 *
 * This freertos_risc_v_chip_specific_extensions.h is for use on RISC-V chips
 * that include a standard CLINT and do not add to the base set of RISC-V
 * registers.
 *
 */

#include "portContext.h"

.global xPortStartFirstTask
.global pxPortInitialiseStackInternal
.global freertos_risc_v_exception_handler
.global freertos_risc_v_interrupt_handler
.global freertos_risc_v_mtimer_interrupt_handler
.global freertos_risc_v_msip_interrupt_handler

.extern vTaskSwitchContext
.extern xTaskIncrementTick
.extern pullMachineTimerCompareRegister
.extern pullNextTime
.extern uxTimerIncrementsForOneTick /* size_t type so 32-bit on 32-bit core and 64-bits on 64-bit core. */
.extern xTaskReturnAddress

.weak freertos_risc_v_application_exception_handler
.weak freertos_risc_v_application_interrupt_handler
/*-----------------------------------------------------------*/

.macro portUPDATE_MTIMER_COMPARE_REGISTER
    load_x a0, pullMachineTimerCompareRegister  /* Load address of compare register into a0. */
    load_x a1, pullNextTime                     /* Load the address of ullNextTime into a1. */

    /* Update the 64-bit mtimer compare match value in two 32-bit writes. */
    li a4, -1
    lw a2, 0(a1)                /* Load the low word of ullNextTime into a2. */
    lw a3, 4(a1)                /* Load the high word of ullNextTime into a3. */
    sw a4, 0(a0)                /* Low word no smaller than old value to start with - will be overwritten below. */
    sw a3, 4(a0)                /* Store high word of ullNextTime into compare register.  No smaller than new value. */
    sw a2, 0(a0)                /* Store low word of ullNextTime into compare register. */
    lw t0, uxTimerIncrementsForOneTick  /* Load the value of ullTimerIncrementForOneTick into t0 (could this be optimized by storing in an array next to pullNextTime?). */
    add a4, t0, a2              /* Add the low word of ullNextTime to the timer increments for one tick (assumes timer increment for one tick fits in 32-bits). */
    sltu t1, a4, a2             /* See if the sum of low words overflowed (what about the zero case?). */
    add t2, a3, t1              /* Add overflow to high word of ullNextTime. */
    sw a4, 0(a1)                /* Store new low word of ullNextTime. */
    sw t2, 4(a1)                /* Store new high word of ullNextTime. */
    .endm
/*-----------------------------------------------------------*/

/*
 * StackType_t *pxPortInitialiseStackInternal( StackType_t *pxTopOfKernelStack,
 *                                             TaskFunction_t pxCode,
 *                                             void *pvParameters,
 *                                             BaseType_t xIsPrivileged
 *                                             StackType_t *pxTopOfTaskStack );
 *
 * As per the standard RISC-V ABI pxTopOfStack is passed in in a0, pxCode in
 * a1, and pvParameters in a2.  The new top of stack is passed out in a0.
 *
 * The RISC-V context is saved to FreeRTOS tasks in the following stack frame,
 * where the global and thread pointers are currently assumed to be constant so
 * are not saved:
 *
 * [FPU registers (when enabled/available) go here]
 * [VPU registers (when enabled/available) go here]
 * [chip specific registers go here]
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
 * x4 (xTaskIsRestricted)
 * x3
 * x2
 * portTASK_RETURN_ADDRESS
 * mscratch (pxKernelStackTop, for M-mode task, this is always 0)
 * mstatus
 * pxCode
 */
pxPortInitialiseStackInternal:

    addi t0, x0, portasmADDITIONAL_CONTEXT_SIZE /* The number of chip specific additional registers. */
chip_specific_stack_frame:              /* First add any chip specific registers to the stack frame being created. */
    beq t0, x0, 1f                      /* No more chip specific registers to save. */
    addi a0, a0, -portWORD_SIZE         /* Make space for chip specific register. */
    store_x x0, 0(a0)                   /* Give the chip specific register an initial value of zero. */
    addi t0, t0, -1                     /* Decrement the count of chip specific registers remaining. */
    j chip_specific_stack_frame         /* Until no more chip specific registers. */
1:

    addi a0, a0, -(22 * portWORD_SIZE)  /* Space for registers x10-x31. */
    store_x a2, 0(a0)                   /* Task parameters (pvParameters parameter) goes into register x10/a0 on the stack. */

    addi a0, a0, -(9 * portWORD_SIZE)   /* Space for registers x2-x9 + taskReturnAddress (register x1). */

    beqz a3, 1f /* a3 = xTaskIsPrivileged */
    /* M-mode Task */
    li t0, 0
    j 2f
1:  /* U-mode Task */
    li t0, 1
2:
    store_x t0, 3 * portWORD_SIZE(a0)   /* xTaskIsRestricted onto the stack */

    load_x t0, xTaskReturnAddress
    store_x t0, 0(a0)                   /* Return address onto the stack. */

    csrr t0, mstatus                    /* Obtain current mstatus value. */
    andi t0, t0, ~0x8                   /* Ensure interrupts are disabled when the stack is restored within an ISR.  Required when a task is created after the scheduler has been started, otherwise interrupts would be disabled anyway. */
    li      t1, (3 << 11)               /* Clear MPP field */
    not     t1, t1
    and     t0, t0, t1

    ori     t0, t0, (1 << 7)            /* Set MPIE field */

    beqz    a3, 1f
    /* M-mode Task */
    li      t1, (3 << 11)
    or      t0, t0, t1                  /* Set MPIE and MPP bits in mstatus value. */
1:  /* U-mode Task */
    /* MPP=0, so do nothing */

#if( configENABLE_FPU == 1 )
    /* Mark the FPU as clean in the mstatus value. */
    li t1, ~MSTATUS_FS_MASK
    and t0, t0, t1
    li t1, MSTATUS_FS_CLEAN
    or t0, t0, t1
#endif

#if( configENABLE_VPU == 1 )
    /* Mark the VPU as clean in the mstatus value. */
    li t1, ~MSTATUS_VS_MASK
    and t0, t0, t1
    li t1, MSTATUS_VS_CLEAN
    or t0, t0, t1
#endif

    addi a0, a0, -3 * portWORD_SIZE
    store_x x0, 2 * portWORD_SIZE(a0)   /* mscratch onto the stack */
    store_x t0, 1 * portWORD_SIZE(a0)   /* mstatus onto the stack. */
    store_x a1, 0(a0)                   /* mret value (pxCode parameter) onto the stack. */
    ret
/*-----------------------------------------------------------*/

xPortStartFirstTask:
    portcontextLOAD_KERNEL_SP_FROM_TCB
    portcontextRESTORE_CONTEXT_FROM_SP
    portcontextM_MODE_EXIT
    mret

/*-----------------------------------------------------------*/

freertos_risc_v_application_exception_handler:
    csrr t0, mcause     /* For viewing in the debugger only. */
    csrr t1, mepc       /* For viewing in the debugger only */
    csrr t2, mstatus    /* For viewing in the debugger only */
    j .
/*-----------------------------------------------------------*/

freertos_risc_v_application_interrupt_handler:
    csrr t0, mcause     /* For viewing in the debugger only. */
    csrr t1, mepc       /* For viewing in the debugger only */
    csrr t2, mstatus    /* For viewing in the debugger only */
    j .
/*-----------------------------------------------------------*/

.section .text.freertos_risc_v_exception_handler
freertos_risc_v_exception_handler:
    portcontextSAVE_EXCEPTION_CONTEXT
    /* a0 now contains mcause. */
    li t0, 8                        /* 8 == ecall from U-mode */
    bne a0, t0, other_exception     /* Not an U-mode ecall, so some other exception. */

    /*
     * a0: old sp, pointing to saved task's context
     */
    mv a0, sp
    call vPortEcallHandler
    j exception_exit

other_exception:
    call freertos_risc_v_application_exception_handler
exception_exit:
    portcontextRESTORE_EXCEPTION_CONTEXT
    mret
/*-----------------------------------------------------------*/

.section .text.freertos_risc_v_interrupt_handler
freertos_risc_v_interrupt_handler:
    portcontextSAVE_INTERRUPT_CONTEXT
    call freertos_risc_v_application_interrupt_handler
    portcontextRESTORE_INTERRUPT_CONTEXT
    mret
/*-----------------------------------------------------------*/

.section .text.freertos_risc_v_mtimer_interrupt_handler
freertos_risc_v_mtimer_interrupt_handler:
    portcontextSAVE_INTERRUPT_CONTEXT
    portUPDATE_MTIMER_COMPARE_REGISTER
    call xTaskIncrementTick
    beqz a0, exit_without_context_switch    /* Don't switch context if incrementing tick didn't unblock a task. */
    call vTaskSwitchContext
exit_without_context_switch:
    portcontextRESTORE_INTERRUPT_CONTEXT
    mret
/*-----------------------------------------------------------*/

.section .text.freertos_risc_v_msip_interrupt_handler
freertos_risc_v_msip_interrupt_handler:
    portcontextSAVE_INTERRUPT_CONTEXT
    call vPortMSIPHandler
    portcontextRESTORE_INTERRUPT_CONTEXT
    mret
/*-----------------------------------------------------------*/
