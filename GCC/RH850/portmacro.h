/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef PORTMACRO_H
    #define PORTMACRO_H

/* FreeRTOS port for RH850 G4 cores, GCC */

    #include <stdint.h>
    #include "rh850_sysreg.h"
    #include "core.h"
    #include "rh850_cpu.h"
    #include "portframe.h"

    #ifdef __cplusplus
    extern "C" {
    #endif

    #define portCHAR          char
    #define portFLOAT         float
    #define portDOUBLE        double
    #define portLONG          long
    #define portSHORT         short
    #define portSTACK_TYPE    uint32_t
    #define portBASE_TYPE     long

    typedef portSTACK_TYPE   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;

    #if ( configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_32_BITS )
        typedef uint32_t     TickType_t;
        #define portMAX_DELAY              ( ( TickType_t ) 0xffffffffUL )
/* 32-bit loads and stores are atomic */
        #define portTICK_TYPE_IS_ATOMIC    1
    #else
        #error "Only 32-bit ticks are supported"
    #endif

    #define portSTACK_GROWTH      ( -1 )
    #define portTICK_PERIOD_MS    ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
    #define portBYTE_ALIGNMENT    8

/*
 * SYNCE before TRAP0, with PSW.ID = 0, so a pending FPU exception hits this
 * task, not the next one (G4KH SW UM 2.1.2.1 p7; U2C HW UM 3.2.4.7 p322).
 */
    #if portUSE_FPU && RH850_CORE_HAS_SYNCE
        #define portYIELD()    __asm__ volatile ( "synce\n\ttrap 0" ::: "memory" )
    #else
        #define portYIELD()    __asm__ volatile ( "trap 0" ::: "memory" )
    #endif

/* An ISR only marks the switch; the EIINT entry does it on the way out */
    extern volatile uint32_t ulPortYieldPending;
    #define portEND_SWITCHING_ISR( x ) \
    do {                               \
        if( ( x ) != pdFALSE ) {       \
            ulPortYieldPending = 1;    \
        }                              \
    } while( 0 )
    #define portYIELD_FROM_ISR( x )    portEND_SWITCHING_ISR( x )

/*
 * PLMR masks priority numbers >= configMAX_SYSCALL_INTERRUPT_PRIORITY; lower
 * numbers are not masked and must not call the API (U2C HW UM p242).
 */
    static inline UBaseType_t ulPortSetInterruptMask( void )
    {
        UBaseType_t old = RH850_STSR( RH850_CORE_MASK_SR );

        RH850_LDSR( RH850_CORE_MASK( configMAX_SYSCALL_INTERRUPT_PRIORITY ), RH850_CORE_MASK_SR );
        return old;
    }

    static inline void vPortClearInterruptMask( UBaseType_t mask )
    {
        RH850_LDSR( mask, RH850_CORE_MASK_SR );
    }

    #define portDISABLE_INTERRUPTS()                  ( void ) ulPortSetInterruptMask()
    #define portENABLE_INTERRUPTS()                   vPortClearInterruptMask( RH850_CORE_MASK_NONE )
    #define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortSetInterruptMask()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortClearInterruptMask( x )

    BaseType_t xPortIsInsideInterrupt( void );
    #if ( configASSERT_DEFINED == 1 )
        void vPortValidateInterruptPriority( void );
        #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
    #endif

/* ISR stack (port.c) and its unused words, for tests */
    extern StackType_t xPortISRStack[];
    extern volatile uint32_t ulPortInterruptNesting;
    UBaseType_t uxPortGetISRStackHighWaterMark( void );

    void vPortEnterCritical( void );
    void vPortExitCritical( void );
    #define portENTER_CRITICAL()                                  vPortEnterCritical()
    #define portEXIT_CRITICAL()                                   vPortExitCritical()

    #define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
    #define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )

    #define portNOP()                                             __asm__ volatile ( "nop" )
    #define portMEMORY_BARRIER()                                  __asm__ volatile ( "" ::: "memory" )

/* Provided by the board: start the tick interrupt, call vPortTickHandler */
    void vPortSetupTimerInterrupt( void );
    void vPortTickHandler( void );

    #if ( configUSE_TICKLESS_IDLE == 1 )

/*
 * Tickless idle needs a free-running up-counter from the board; its compare
 * must raise the tick request itself when the counter is already past it.
 */
        void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
        #define portSUPPRESS_TICKS_AND_SLEEP( x )    vPortSuppressTicksAndSleep( x )
        uint32_t ulPortTickTimerCount( void );
        void vPortTickTimerSetCompare( uint32_t ulCompare );
        void vPortTickTimerClearPending( void );
        extern const uint32_t ulPortTickTimerCyclesPerTick;
        void vPortIrqDisable( void );
        void vPortIrqEnable( void );
    #endif /* if ( configUSE_TICKLESS_IDLE == 1 ) */
/* Provided by the application: handle the EIINT with this EIIC value */
    void vApplicationIRQHandler( uint32_t ulEIIC );

    #ifdef __cplusplus
}
    #endif

#endif /* PORTMACRO_H */
