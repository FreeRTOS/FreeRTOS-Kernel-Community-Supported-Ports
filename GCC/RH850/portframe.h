/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef PORTFRAME_H
#define PORTFRAME_H

/*
 * FPU slots are used only with an FPU and configENABLE_FPU: with CU0 = 0 an
 * FPSR/FPEPC access raises UCPOP (U2C HW UM p185). Needs core.h first.
 */
#if RH850_CORE_HAS_FPU && ( configENABLE_FPU == 1 )
    #define portUSE_FPU    1
#else
    #define portUSE_FPU    0
#endif

/*
 * Task frame, byte offsets from the saved sp (lowest address first).
 * Used by port.c and portasm.S. sp itself (r3) is kept in the TCB.
 */
#define portFRAME_CRITICAL    0
#define portFRAME_FPEPC       4
#define portFRAME_FPSR        8
#define portFRAME_EIPSW       12
#define portFRAME_EIPC        16
#define portFRAME_R1          20
#define portFRAME_R2          24
/* r4 to r31 */
#define portFRAME_R( n )    ( 28 + ( ( n ) - 4 ) * 4 )
/* Interrupt mask (PLMR) of the task, so it follows the task */
#define portFRAME_MASK    140
/* 8-byte multiple keeps sp aligned */
#define portFRAME_SIZE    144

/*
 * Frame of a nested interrupt on the ISR stack: the return state and the
 * registers a GCC call may change (r1, r2, r5-r19, r30, r31).
 */
#define portISR_EIPC      0
#define portISR_EIPSW     4
#define portISR_FPSR      8
#define portISR_FPEPC     12
#define portISR_R1        16
#define portISR_R2        20
#define portISR_R5        24
/* r6 to r19 */
#define portISR_R( n )    ( 28 + ( ( n ) - 6 ) * 4 )
#define portISR_R30       84
#define portISR_R31       88
#define portISR_SIZE      96

#endif /* PORTFRAME_H */
