/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: MIT
 */

/* FreeRTOS port for RH850 G4 cores: the C part */

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "portframe.h"

_Static_assert( portFRAME_MASK + 4 <= portFRAME_SIZE, "frame too small" );
_Static_assert( ( portFRAME_SIZE % portBYTE_ALIGNMENT ) == 0, "frame size" );
_Static_assert( portISR_R31 + 4 <= portISR_SIZE, "ISR frame too small" );
_Static_assert( ( portISR_SIZE % portBYTE_ALIGNMENT ) == 0, "ISR frame size" );
_Static_assert( configKERNEL_INTERRUPT_PRIORITY >= configMAX_SYSCALL_INTERRUPT_PRIORITY,
                "the kernel interrupt must be maskable" );
_Static_assert( configMAX_SYSCALL_INTERRUPT_PRIORITY > 0,
                "a mask of 0 would mask every interrupt" );

/* Not zero before the scheduler runs, so critical sections stay masked */
#define portINITIAL_CRITICAL_NESTING    0xaaaaaaaaUL
#define portISR_STACK_FILL              0xa5a5a5a5UL

/* Nesting of the running task; portasm.S saves it in each task frame */
volatile uint32_t ulPortCriticalNesting = portINITIAL_CRITICAL_NESTING;
/* Set by ISRs; the outermost EIINT exit switches tasks */
volatile uint32_t ulPortYieldPending;
/* EIINT nesting depth, kept by portasm.S */
volatile uint32_t ulPortInterruptNesting;
#if ( configUSE_TICKLESS_IDLE == 1 )
/* Tick timer counter value of the last counted tick */
    static uint32_t ulLastTick;
#endif

/* ISR stack; the outermost EIINT entry moves sp to its top */
StackType_t xPortISRStack[ configISR_STACK_SIZE_WORDS ] __attribute__( ( aligned( portBYTE_ALIGNMENT ) ) );
StackType_t * const pxPortISRStackTop = &xPortISRStack[ configISR_STACK_SIZE_WORDS ];

void vPortStartFirstTask( void ) __attribute__( ( noreturn ) );

static void prvTaskExitError( void )
{
    /* A task function returned */
    configASSERT( 0 );

    for( ; ; )
    {
    }
}

StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
    uintptr_t end = ( uintptr_t ) ( pxTopOfStack + 1 ) & ~( uintptr_t ) ( portBYTE_ALIGNMENT - 1 );
    uint8_t * frame = ( uint8_t * ) ( end - portFRAME_SIZE );

    memset( frame, 0, portFRAME_SIZE );
    *( uint32_t * ) ( frame + portFRAME_EIPC ) = ( uint32_t ) pxCode;
    *( uint32_t * ) ( frame + portFRAME_R( 6 ) ) = ( uint32_t ) pvParameters;
    *( uint32_t * ) ( frame + portFRAME_R( 31 ) ) = ( uint32_t ) prvTaskExitError;
    /* EBASE vectors, interrupts on (ID = 0); CU0 only if tasks use the FPU */
    *( uint32_t * ) ( frame + portFRAME_EIPSW ) = RH850_PSW_EBV
                                                  #if portUSE_FPU
                                                      | RH850_PSW_CU0
                                                  #endif
    ;
    #if portUSE_FPU
        *( uint32_t * ) ( frame + portFRAME_FPSR ) = RH850_FPSR_FS;
    #endif
    *( uint32_t * ) ( frame + portFRAME_CRITICAL ) = 0;
    *( uint32_t * ) ( frame + portFRAME_MASK ) = RH850_CORE_MASK_NONE;

    return ( StackType_t * ) frame;
}

BaseType_t xPortStartScheduler( void )
{
    for( unsigned int i = 0; i < configISR_STACK_SIZE_WORDS; i++ )
    {
        xPortISRStack[ i ] = portISR_STACK_FILL;
    }

    vPortSetupTimerInterrupt();
    #if ( configUSE_TICKLESS_IDLE == 1 )
        ulLastTick = ulPortTickTimerCount();
        vPortTickTimerSetCompare( ulLastTick + ulPortTickTimerCyclesPerTick );
    #endif
    vPortStartFirstTask();
}

void vPortEndScheduler( void )
{
    /* Not supported */
    configASSERT( 0 );
}

void vPortEnterCritical( void )
{
    ( void ) ulPortSetInterruptMask();
    ulPortCriticalNesting++;
}

void vPortExitCritical( void )
{
    configASSERT( ulPortCriticalNesting != 0U );
    ulPortCriticalNesting--;

    if( ulPortCriticalNesting == 0U )
    {
        vPortClearInterruptMask( RH850_CORE_MASK_NONE );
    }
}

#if ( configUSE_TICKLESS_IDLE == 1 )
/* Longest sleep, so that counter differences stay below 2^31 */
    #define portMAX_SLEEP_TICKS    ( 0x7fffffffUL / ulPortTickTimerCyclesPerTick )

/* Count every full tick period since the last one; 0 for a late duplicate */
    static void prvCountTicks( void )
    {
        const uint32_t cycles = ulPortTickTimerCyclesPerTick;
        uint32_t ticks = ( ulPortTickTimerCount() - ulLastTick ) / cycles;

        while( ticks-- != 0U )
        {
            ulLastTick += cycles;

            if( xTaskIncrementTick() != pdFALSE )
            {
                ulPortYieldPending = 1;
            }
        }
    }

    void vPortTickHandler( void )
    {
        UBaseType_t mask = ulPortSetInterruptMask();

        prvCountTicks();
        vPortTickTimerSetCompare( ulLastTick + ulPortTickTimerCyclesPerTick );
        vPortClearInterruptMask( mask );
    }

/*
 * Called by the idle task with the scheduler suspended. PSW.ID = 1 holds
 * every interrupt, but an EIINT request still ends HALT (G4MH SW UM p80).
 */
    void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
        const uint32_t cycles = ulPortTickTimerCyclesPerTick;
        TickType_t xModifiableIdleTime;
        uint32_t ticks;

        if( xExpectedIdleTime > portMAX_SLEEP_TICKS )
        {
            xExpectedIdleTime = portMAX_SLEEP_TICKS;
        }

        vPortIrqDisable();

        if( eTaskConfirmSleepModeStatus() == eAbortSleep )
        {
            vPortIrqEnable();
            return;
        }

        vPortTickTimerSetCompare( ulLastTick + xExpectedIdleTime * cycles );
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );

        if( xModifiableIdleTime > 0U )
        {
            rh850_halt();
        }

        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        /* Step the full ticks slept; the rest is counted by the tick ISR */
        ticks = ( ulPortTickTimerCount() - ulLastTick ) / cycles;

        if( ticks > xExpectedIdleTime )
        {
            ticks = xExpectedIdleTime;
        }

        if( ticks != 0U )
        {
            ulLastTick += ticks * cycles;
            vTaskStepTick( ticks );
        }

        /* A compare that fired during the sleep is counted above */
        vPortTickTimerClearPending();
        vPortTickTimerSetCompare( ulLastTick + cycles );
        vPortIrqEnable();
    }
#else /* if ( configUSE_TICKLESS_IDLE == 1 ) */
    void vPortTickHandler( void )
    {
        UBaseType_t mask = ulPortSetInterruptMask();

        if( xTaskIncrementTick() != pdFALSE )
        {
            ulPortYieldPending = 1;
        }

        vPortClearInterruptMask( mask );
    }
#endif /* if ( configUSE_TICKLESS_IDLE == 1 ) */

BaseType_t xPortIsInsideInterrupt( void )
{
    return ( ulPortInterruptNesting != 0U ) ? pdTRUE : pdFALSE;
}

#if ( configASSERT_DEFINED == 1 )

/*
 * FromISR APIs need priority number >= the mask value. The lowest set ISPR
 * bit is the running priority (U2C HW UM Table 3.44 p237).
 */
    void vPortValidateInterruptPriority( void )
    {
        uint32_t ispr = RH850_STSR( RH850_SR_ISPR );

        if( ispr != 0U )
        {
            configASSERT( ( uint32_t ) __builtin_ctz( ispr ) >= configMAX_SYSCALL_INTERRUPT_PRIORITY );
        }
    }
#endif /* if ( configASSERT_DEFINED == 1 ) */

UBaseType_t uxPortGetISRStackHighWaterMark( void )
{
    UBaseType_t n = 0;

    while( ( n < configISR_STACK_SIZE_WORDS ) && ( xPortISRStack[ n ] == portISR_STACK_FILL ) )
    {
        n++;
    }

    return n;
}
