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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the RISC-V port.
 *----------------------------------------------------------*/

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "mpu_syscall_numbers.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Standard includes. */
#include "string.h"

#ifndef configMTIME_BASE_ADDRESS
    #warning "configMTIME_BASE_ADDRESS must be defined in FreeRTOSConfig.h. If the target chip includes a memory-mapped mtime register then set configMTIME_BASE_ADDRESS to the mapped address.  Otherwise set configMTIME_BASE_ADDRESS to 0.  See www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html"
#endif

#ifndef configMTIMECMP_BASE_ADDRESS
    #warning "configMTIMECMP_BASE_ADDRESS must be defined in FreeRTOSConfig.h. If the target chip includes a memory-mapped mtimecmp register then set configMTIMECMP_BASE_ADDRESS to the mapped address.  Otherwise set configMTIMECMP_BASE_ADDRESS to 0.  See www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html"
#endif

/* Let the user override the pre-loading of the initial RA. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    0
#endif

/* The stack used by interrupt service routines.  Set configISR_STACK_SIZE_WORDS
 * to use a statically allocated array as the interrupt stack.  Alternative leave
 * configISR_STACK_SIZE_WORDS undefined and update the linker script so that a
 * linker variable names __freertos_irq_stack_top has the same value as the top
 * of the stack used by main.  Using the linker script method will repurpose the
 * stack that was used by main before the scheduler was started for use as the
 * interrupt stack after the scheduler has started. */
#ifdef configISR_STACK_SIZE_WORDS
static __attribute__( ( aligned( 16 ) ) ) StackType_t xISRStack[ configISR_STACK_SIZE_WORDS ] = { 0 };
const StackType_t xISRStackTop = ( StackType_t ) &( xISRStack[ configISR_STACK_SIZE_WORDS & ~portBYTE_ALIGNMENT_MASK ] );

/* Don't use 0xa5 as the stack fill bytes as that is used by the kernel for
 * the task stacks, and so will legitimately appear in many positions within
 * the ISR stack. */
    #define portISR_STACK_FILL_BYTE    0xee
#else
    extern const uint32_t __freertos_irq_stack_top[];
    const StackType_t xISRStackTop = ( StackType_t ) __freertos_irq_stack_top;
#endif

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void ) __attribute__( ( weak ) );

/*-----------------------------------------------------------*/

/* Used to program the machine timer compare register. */
uint64_t ullNextTime = 0ULL;
const uint64_t * pullNextTime = &ullNextTime;
const size_t uxTimerIncrementsForOneTick = ( size_t ) ( ( configCPU_CLOCK_HZ ) / ( configTICK_RATE_HZ ) ); /* Assumes increment won't go over 32-bits. */
UBaseType_t const ullMachineTimerCompareRegisterBase = configMTIMECMP_BASE_ADDRESS;
volatile uint64_t * pullMachineTimerCompareRegister = NULL;

size_t xCriticalNesting = ( size_t ) 0x0;

/* Used to catch tasks that attempt to return from their implementing function. */
size_t xTaskReturnAddress = ( size_t ) portTASK_RETURN_ADDRESS;

PRIVILEGED_DATA static BaseType_t xSchedulerRunning = pdFALSE;

#define portCHECK_ISR_STACK()

/*-----------------------------------------------------------*/

#if ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 )

    void vPortSetupTimerInterrupt( void )
    {
        uint32_t ulCurrentTimeHigh, ulCurrentTimeLow;
        volatile uint32_t * const pulTimeHigh = ( volatile uint32_t * const ) ( ( configMTIME_BASE_ADDRESS ) + 4UL ); /* 8-byte type so high 32-bit word is 4 bytes up. */
        volatile uint32_t * const pulTimeLow = ( volatile uint32_t * const ) ( configMTIME_BASE_ADDRESS );
        volatile uint32_t ulHartId;

        __asm volatile ( "csrr %0, mhartid" : "=r" ( ulHartId ) );

        pullMachineTimerCompareRegister = ( volatile uint64_t * ) ( ullMachineTimerCompareRegisterBase + ( ulHartId * sizeof( uint64_t ) ) );

        do
        {
            ulCurrentTimeHigh = *pulTimeHigh;
            ulCurrentTimeLow = *pulTimeLow;
        } while( ulCurrentTimeHigh != *pulTimeHigh );

        ullNextTime = ( uint64_t ) ulCurrentTimeHigh;
        ullNextTime <<= 32ULL; /* High 4-byte word is 32-bits up. */
        ullNextTime |= ( uint64_t ) ulCurrentTimeLow;
        ullNextTime += ( uint64_t ) uxTimerIncrementsForOneTick;
        *pullMachineTimerCompareRegister = ullNextTime;

        /* Prepare the time to use after the next tick interrupt. */
        ullNextTime += ( uint64_t ) uxTimerIncrementsForOneTick;
    }

#endif /* ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIME_BASE_ADDRESS != 0 ) */
/*-----------------------------------------------------------*/

static inline void prvPMPAllowAllAccess( void )
{
    /* Constants for PMP config bits */
    /* bit0 = R, bit1 = W, bit2 = X, bits3:4 = A field, bit7 = L (lock) */
    const unsigned long PMP_R = 1UL << 0;
    const unsigned long PMP_W = 1UL << 1;
    const unsigned long PMP_X = 1UL << 2;
    const unsigned long PMP_A_TOR = 1UL << 3; /* A = 01 -> TOR; note: bit3 not bit4 */

    __asm__ volatile (
        /* pmpaddr0 = 0 (lower bound for TOR) */
        "li      t0, 0\n"
        "csrw    pmpaddr0, t0\n"

        /* pmpaddr1 = all ones (upper bound = whole address space)
         * Use -1 to generate all-ones for both RV32 and RV64 (assembler will expand).
         */
        "li      t0, -1\n"
        "csrw    pmpaddr1, t0\n"

        /* Build cfg byte for entry1: RWX + A=TOR
         * entry1 is byte 1 in pmpcfg0, so shift left by 8.
         */
        "li      t0, %[rwx] \n"      /* lower 32-bit immediate for RWX|A */
        "slli    t0, t0, 8\n"       /* place into byte1 (entry1) */
        "csrw    pmpcfg0, t0\n"

        "fence\n"
        :
        : [rwx] "i" ( (unsigned long)(PMP_R | PMP_W | PMP_X | PMP_A_TOR) )
        : "t0"
    );
}

BaseType_t xPortStartScheduler( void )
{
    extern void xPortStartFirstTask( void );

    #if ( configASSERT_DEFINED == 1 )
    {
        /* Check alignment of the interrupt stack - which is the same as the
         * stack that was being used by main() prior to the scheduler being
         * started. */
        configASSERT( ( xISRStackTop & portBYTE_ALIGNMENT_MASK ) == 0 );

        #ifdef configISR_STACK_SIZE_WORDS
        {
            memset( ( void * ) xISRStack, portISR_STACK_FILL_BYTE, sizeof( xISRStack ) );
        }
        #endif /* configISR_STACK_SIZE_WORDS */
    }
    #endif /* configASSERT_DEFINED */

    /* If there is a CLINT then it is ok to use the default implementation
     * in this file, otherwise vPortSetupTimerInterrupt() must be implemented to
     * configure whichever clock is to be used to generate the tick interrupt. */
    vPortSetupTimerInterrupt();

    #if ( ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 ) )
    {
        /* Enable mtime and external interrupts.  1<<7 for timer interrupt,
         * 1<<11 for external interrupt.  _RB_ What happens here when mtime is
         * not present as with pulpino? */
        __asm volatile ( "csrs mie, %0" ::"r" ( 0x880 ) );
    }
    #endif /* ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 ) */

    xSchedulerRunning = pdTRUE;

    /* Enable MSIP interrupt */
    __asm volatile ( "csrsi mie, 0x8" );

    xPortStartFirstTask();

    /* Should not get here as after calling xPortStartFirstTask() only tasks
     * should be executing. */
    return pdFAIL;
}
/*-----------------------------------------------------------*/

void vPortMSIPHandler()
{
    *((volatile unsigned long *)configCLINT_BASE_ADDRESS) = 0UL;
    portYIELD_FROM_ISR( pdTRUE );
}
/*-----------------------------------------------------------*/


void vPortEndScheduler( void )
{
    /* Not implemented. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/* MPU v2 */
typedef struct PARTIAL_TASK_CONTEXT {
    uint32_t mepc;
    uint32_t mstatus;
    uint32_t mscratch;
    uint32_t x1;
    uint32_t x2;
    uint32_t x3;
    uint32_t x4;
    uint32_t x5;
    uint32_t x6;
    uint32_t x7;
    uint32_t x8;
    uint32_t x9;
    uint32_t x10;
    uint32_t x11;
    uint32_t x12;
    uint32_t x13;
    uint32_t x14;
    uint32_t x15;
    uint32_t x16;
    uint32_t x17;
    uint32_t x18;
    uint32_t x19;
    uint32_t x20;
    uint32_t x21;
    uint32_t x22;
    uint32_t x23;
    uint32_t x24;
    uint32_t x25;
    uint32_t x26;
    uint32_t x27;
    uint32_t x28;
    uint32_t x29;
    uint32_t x30;
    uint32_t x31;
} xPARTIAL_TASK_CONTEXT;

typedef uint32_t (*ulSyscallImplFunc)(uint32_t a0,
                                      uint32_t a1,
                                      uint32_t a2,
                                      uint32_t a3,
                                      uint32_t a4,
                                      uint32_t a5,
                                      uint32_t a6,
                                      uint32_t a7);

void vPortEcallHandler( xPARTIAL_TASK_CONTEXT *pxTaskContext) PRIVILEGED_FUNCTION;
void vPortEcallHandler( xPARTIAL_TASK_CONTEXT *pxTaskContext)
{
    extern UBaseType_t uxSystemCallImplementations[ NUM_SYSTEM_CALLS ];
    uint32_t ulSystemCallNumber = pxTaskContext->x17; /* a7 */
    uint32_t ulSystemCallReturnValue;

    if( ( ulSystemCallNumber < NUM_SYSTEM_CALLS ) &&
        ( uxSystemCallImplementations[ ulSystemCallNumber ] != ( UBaseType_t ) 0 ) )
    {
        ulSyscallImplFunc ulSyscallImpl = ( ulSyscallImplFunc ) uxSystemCallImplementations[ ulSystemCallNumber ];
        ulSystemCallReturnValue = ulSyscallImpl(pxTaskContext->x10, /* a0 */
                                                pxTaskContext->x11, /* a1 */
                                                pxTaskContext->x12, /* a2 */
                                                pxTaskContext->x13, /* a3 */
                                                pxTaskContext->x14, /* a4 */
                                                pxTaskContext->x15, /* a5 */
                                                pxTaskContext->x16, /* a6 */
                                                pxTaskContext->x17);/* a7 */
        pxTaskContext->x10 = ulSystemCallReturnValue;
    }
    else
    {
        while (1) { };
    }
}

static void prvInitialiseKernelStackInfo( StackType_t * pxTopOfStack,
                                          StackType_t * pxEndOfStack,
                                          BaseType_t xRunPrivileged,
                                          xKERNEL_STACK_INFO * pxKernelStackInfo ) PRIVILEGED_FUNCTION;

static void prvInitialiseKernelStackInfo( StackType_t * pxTopOfStack,
                                          StackType_t * pxEndOfStack,
                                          BaseType_t xRunPrivileged,
                                          xKERNEL_STACK_INFO * pxKernelStackInfo )
{
    /*
     *   - For Unprivileged task, we allocate kernel stack for it
     *   - For Privileged task, we use provided stack as kernel stack
     */
    StackType_t * pxKernelStackLow;
    StackType_t * pxKernelStackHigh;

    /* Stack grow is -1, so: pxEndOfStack is the low address, pxTopOfStack is the high address */
    if ( xRunPrivileged != pdTRUE )
    {
        pxKernelStackLow = (StackType_t *)pvPortMallocStack( configSYSTEM_CALL_STACK_SIZE );
        configASSERT ( pxKernelStackLow != NULL );
        pxKernelStackHigh = pxKernelStackLow + configSYSTEM_CALL_STACK_SIZE;
    }
    else
    {
        pxKernelStackLow = pxEndOfStack;
        pxKernelStackHigh = pxTopOfStack;
    }

    pxKernelStackInfo->pxKernelStackLow = pxKernelStackLow;
    pxKernelStackInfo->pxKernelStackHigh = pxKernelStackHigh;
}

StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     StackType_t * pxEndOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters,
                                     BaseType_t xRunPrivileged,
                                     xMPU_SETTINGS * xMPUSettings ) /* PRIVILEGED_FUNCTION */
{
    extern StackType_t * pxPortInitialiseStackInternal( StackType_t *pxTopOfKernelStack,
                                                        TaskFunction_t pxCode,
                                                        void *pvParameters,
                                                        BaseType_t xRunPrivileged,
                                                        StackType_t *pxTopOfTaskStack );
    /* alignment check of stack range is done in vPortStoreTaskMPUSettings */

    xKERNEL_STACK_INFO * pxKernelStackInfo = &(xMPUSettings->xKernelStackInfo);

    prvInitialiseKernelStackInfo(pxTopOfStack, pxEndOfStack, xRunPrivileged, pxKernelStackInfo);

    xMPUSettings->xTaskIsPrivileged = xRunPrivileged;

    /* Task Context is now saved on kernel stack */
    pxKernelStackInfo->pxTopOfKernelStack =  pxPortInitialiseStackInternal(
                                                pxKernelStackInfo->pxKernelStackHigh,
                                                pxCode,
                                                pvParameters,
                                                xRunPrivileged,
                                                pxTopOfStack );
    /* Return Top of Task Stack to FreeRTOS kernel */
    if ( xRunPrivileged == pdTRUE)
    {
        /* For Privileged Task, there's no Task Stack */
        /* We still return kernel stack top for stack checks inside FreeRTOS kernel */
        return pxKernelStackInfo->pxTopOfKernelStack;
    }
    else
    {
        return pxTopOfStack;
    }
}

static uint8_t pmp_build_cfg( uint32_t attr )
{
    uint8_t cfg = portPMP_A_NAPOT;

    if( attr & portMPU_REGION_READ )
    {
        cfg |= portPMP_R;
    }

    if( attr & portMPU_REGION_WRITE )
    {
        cfg |= portPMP_W;
    }

    if( attr & portMPU_REGION_EXECUTE )
    {
        cfg |= portPMP_X;
    }

    return cfg;
}

static BaseType_t pmp_check_access( uint8_t cfg,
                                    uint32_t access )
{
    if( ( access & tskMPU_READ_PERMISSION ) )
    {
        if( ( cfg & portPMP_R ) == 0 )
        {
            return pdFAIL;
        }
    }

    if( access & tskMPU_WRITE_PERMISSION )
    {
        if( ( cfg & portPMP_W ) == 0 )
        {
            return pdFAIL;
        }
    }
    return pdPASS;
}

static void pmp_encode_napot_addr( uintptr_t start,
                                   uintptr_t end,
                                   uint32_t * out )
{
    uint32_t size;

    configASSERT( end > start );

    size = end - start;

    /* NAPOT size must be power of two */
    configASSERT( ( size & ( size - 1UL ) ) == 0 );

    /* PMP granularity constraint */
    configASSERT( size >= portPMP_GRANULARITY_BYTES );

    /* NAPOT base alignment */
    configASSERT( ( start & ( size - 1UL ) ) == 0 );

    /*
     * NAPOT encoding:
     * pmpaddr = (base >> 2) | ((size >> 3) - 1)
     */
    *out = ( start >> 2 ) | ( ( size >> 3 ) - 1U );
}

static void pmp_decode_napot_addr( uint32_t pmpaddr,
                                   uint32_t * start,
                                   uint32_t * end )
{
    uint32_t n;

    /* count trailing ones */
    n = __builtin_ctz( ~pmpaddr );

    /*
     * granularity = 8 bytes
     * minimal NAPOT requires at least one trailing 1
     */
    configASSERT( n >= 1 )

    uint32_t size = 1UL << ( n + 3 );
    uint32_t base = ( pmpaddr & ~((1UL << n) - 1UL) ) << 2;

    *start = base;
    *end   = base + size;
}

static void pmp_add_entry( xPMP_SETTINGS * pmp,
                           uint32_t start,
                           uint32_t end,
                           uint32_t attr )
{
    uint32_t entry;
    uint32_t pmpaddr;
    uint8_t  cfg;

    configASSERT( pmp->pmp_cfg_used < portPMP_MAX_ENTRY );

    entry = pmp->pmp_cfg_used;

    pmp_encode_napot_addr( ( uintptr_t ) start,
                           ( uintptr_t ) end,
                           &pmpaddr );

    cfg = pmp_build_cfg( attr );

    pmp->pmpaddr[ entry ] = pmpaddr;

    {
        uint32_t idx   = entry / portPMP_ENTRY_PER_CONFIG;
        uint32_t shift = ( entry % portPMP_ENTRY_PER_CONFIG ) * portPMP_ENTRY_BIT;

        pmp->pmpcfg[ idx ] &= ~( 0xFFU << shift );
        pmp->pmpcfg[ idx ] |= ( ( uint32_t ) cfg << shift );
    }

    pmp->pmp_cfg_used++;
}

void vPortStoreTaskMPUSettings( xMPU_SETTINGS * xMPUSettings,
                                const struct xMEMORY_REGION * const xRegions,
                                StackType_t * pxBottomOfStack,
                                configSTACK_DEPTH_TYPE uxStackDepth ) PRIVILEGED_FUNCTION;

void vPortStoreTaskMPUSettings( xMPU_SETTINGS * xMPUSettings,
                                const struct xMEMORY_REGION * const xRegions,
                                StackType_t * pxBottomOfStack,
                                configSTACK_DEPTH_TYPE uxStackDepth ) /* PRIVILEGED_FUNCTION */
{
    xPMP_SETTINGS * pmp = &(xMPUSettings->xPMPSettings);

    ( void ) pxBottomOfStack;
    ( void ) uxStackDepth;

    if ( xMPUSettings->xTaskIsPrivileged )
    {
        /* FIXME: We cannot restrict M-mode access using PMP */
        return;
    }
    if ( xRegions == NULL )
    {
        return;
    }

    memset( pmp, 0, sizeof( *pmp ) );

    /* This function is called automatically when the task is created - in
     * which case the stack region parameters will be valid.  At all other
     * times the stack parameters will not be valid and it is assumed that the
     * stack region has already been configured. */
    if( uxStackDepth > 0 )
    {
        pmp_add_entry(
            pmp,
            ( uint32_t ) pxBottomOfStack,
            ( uint32_t ) pxBottomOfStack + uxStackDepth * sizeof(StackType_t),
            portMPU_REGION_READ | portMPU_REGION_WRITE
        );
    }

    for( uint32_t i = 0; i < portNUM_CONFIGURABLE_REGIONS; i++ )
    {
        const MemoryRegion_t * r = &xRegions[ i ];

        if( r->ulLengthInBytes == 0 )
        {
            continue;
        }

        pmp_add_entry(
            pmp,
            ( uint32_t ) r->pvBaseAddress,
            ( uint32_t ) r->pvBaseAddress + r->ulLengthInBytes,
            r->ulParameters
        );
    }
}

static inline void pmp_write_pmpaddr( uint32_t idx, uint32_t val )
{
    switch( idx )
    {
        case 0:  __asm volatile("csrw pmpaddr0,  %0" :: "r"(val)); break;
        case 1:  __asm volatile("csrw pmpaddr1,  %0" :: "r"(val)); break;
        case 2:  __asm volatile("csrw pmpaddr2,  %0" :: "r"(val)); break;
        case 3:  __asm volatile("csrw pmpaddr3,  %0" :: "r"(val)); break;
        case 4:  __asm volatile("csrw pmpaddr4,  %0" :: "r"(val)); break;
        case 5:  __asm volatile("csrw pmpaddr5,  %0" :: "r"(val)); break;
        case 6:  __asm volatile("csrw pmpaddr6,  %0" :: "r"(val)); break;
        case 7:  __asm volatile("csrw pmpaddr7,  %0" :: "r"(val)); break;
        case 8:  __asm volatile("csrw pmpaddr8,  %0" :: "r"(val)); break;
        case 9:  __asm volatile("csrw pmpaddr9,  %0" :: "r"(val)); break;
        case 10: __asm volatile("csrw pmpaddr10, %0" :: "r"(val)); break;
        case 11: __asm volatile("csrw pmpaddr11, %0" :: "r"(val)); break;
        case 12: __asm volatile("csrw pmpaddr12, %0" :: "r"(val)); break;
        case 13: __asm volatile("csrw pmpaddr13, %0" :: "r"(val)); break;
        case 14: __asm volatile("csrw pmpaddr14, %0" :: "r"(val)); break;
        case 15: __asm volatile("csrw pmpaddr15, %0" :: "r"(val)); break;
        //case 16: __asm volatile("csrw pmpaddr16, %0" :: "r"(val)); break;
        //case 17: __asm volatile("csrw pmpaddr17, %0" :: "r"(val)); break;
        //case 18: __asm volatile("csrw pmpaddr18, %0" :: "r"(val)); break;
        //case 19: __asm volatile("csrw pmpaddr19, %0" :: "r"(val)); break;
        //case 20: __asm volatile("csrw pmpaddr20, %0" :: "r"(val)); break;
        //case 21: __asm volatile("csrw pmpaddr21, %0" :: "r"(val)); break;
        //case 22: __asm volatile("csrw pmpaddr22, %0" :: "r"(val)); break;
        //case 23: __asm volatile("csrw pmpaddr23, %0" :: "r"(val)); break;
        //case 24: __asm volatile("csrw pmpaddr24, %0" :: "r"(val)); break;
        //case 25: __asm volatile("csrw pmpaddr25, %0" :: "r"(val)); break;
        //case 26: __asm volatile("csrw pmpaddr26, %0" :: "r"(val)); break;
        //case 27: __asm volatile("csrw pmpaddr27, %0" :: "r"(val)); break;
        //case 28: __asm volatile("csrw pmpaddr28, %0" :: "r"(val)); break;
        //case 29: __asm volatile("csrw pmpaddr29, %0" :: "r"(val)); break;
        //case 30: __asm volatile("csrw pmpaddr30, %0" :: "r"(val)); break;
        //case 31: __asm volatile("csrw pmpaddr31, %0" :: "r"(val)); break;
        //case 32: __asm volatile("csrw pmpaddr32, %0" :: "r"(val)); break;
        //case 33: __asm volatile("csrw pmpaddr33, %0" :: "r"(val)); break;
        //case 34: __asm volatile("csrw pmpaddr34, %0" :: "r"(val)); break;
        //case 35: __asm volatile("csrw pmpaddr35, %0" :: "r"(val)); break;
        //case 36: __asm volatile("csrw pmpaddr36, %0" :: "r"(val)); break;
        //case 37: __asm volatile("csrw pmpaddr37, %0" :: "r"(val)); break;
        //case 38: __asm volatile("csrw pmpaddr38, %0" :: "r"(val)); break;
        //case 39: __asm volatile("csrw pmpaddr39, %0" :: "r"(val)); break;
        //case 40: __asm volatile("csrw pmpaddr40, %0" :: "r"(val)); break;
        //case 41: __asm volatile("csrw pmpaddr41, %0" :: "r"(val)); break;
        //case 42: __asm volatile("csrw pmpaddr42, %0" :: "r"(val)); break;
        //case 43: __asm volatile("csrw pmpaddr43, %0" :: "r"(val)); break;
        //case 44: __asm volatile("csrw pmpaddr44, %0" :: "r"(val)); break;
        //case 45: __asm volatile("csrw pmpaddr45, %0" :: "r"(val)); break;
        //case 46: __asm volatile("csrw pmpaddr46, %0" :: "r"(val)); break;
        //case 47: __asm volatile("csrw pmpaddr47, %0" :: "r"(val)); break;
        //case 48: __asm volatile("csrw pmpaddr48, %0" :: "r"(val)); break;
        //case 49: __asm volatile("csrw pmpaddr49, %0" :: "r"(val)); break;
        //case 50: __asm volatile("csrw pmpaddr50, %0" :: "r"(val)); break;
        //case 51: __asm volatile("csrw pmpaddr51, %0" :: "r"(val)); break;
        //case 52: __asm volatile("csrw pmpaddr52, %0" :: "r"(val)); break;
        //case 53: __asm volatile("csrw pmpaddr53, %0" :: "r"(val)); break;
        //case 54: __asm volatile("csrw pmpaddr54, %0" :: "r"(val)); break;
        //case 55: __asm volatile("csrw pmpaddr55, %0" :: "r"(val)); break;
        //case 56: __asm volatile("csrw pmpaddr56, %0" :: "r"(val)); break;
        //case 57: __asm volatile("csrw pmpaddr57, %0" :: "r"(val)); break;
        //case 58: __asm volatile("csrw pmpaddr58, %0" :: "r"(val)); break;
        //case 59: __asm volatile("csrw pmpaddr59, %0" :: "r"(val)); break;
        //case 60: __asm volatile("csrw pmpaddr60, %0" :: "r"(val)); break;
        //case 61: __asm volatile("csrw pmpaddr61, %0" :: "r"(val)); break;
        //case 62: __asm volatile("csrw pmpaddr62, %0" :: "r"(val)); break;
        //case 63: __asm volatile("csrw pmpaddr63, %0" :: "r"(val)); break;
        default: configASSERT( pdFALSE );
    }
}

static inline void pmp_write_pmpcfg( uint32_t idx, uint32_t val )
{
    switch( idx )
    {
        case 0:  __asm volatile("csrw pmpcfg0,  %0" :: "r"(val)); break;
        case 1:  __asm volatile("csrw pmpcfg1,  %0" :: "r"(val)); break;
        case 2:  __asm volatile("csrw pmpcfg2,  %0" :: "r"(val)); break;
        case 3:  __asm volatile("csrw pmpcfg3,  %0" :: "r"(val)); break;
        //case 4:  __asm volatile("csrw pmpcfg4,  %0" :: "r"(val)); break;
        //case 5:  __asm volatile("csrw pmpcfg5,  %0" :: "r"(val)); break;
        //case 6:  __asm volatile("csrw pmpcfg6,  %0" :: "r"(val)); break;
        //case 7:  __asm volatile("csrw pmpcfg7,  %0" :: "r"(val)); break;
        //case 8:  __asm volatile("csrw pmpcfg8,  %0" :: "r"(val)); break;
        //case 9:  __asm volatile("csrw pmpcfg9,  %0" :: "r"(val)); break;
        //case 10: __asm volatile("csrw pmpcfg10, %0" :: "r"(val)); break;
        //case 11: __asm volatile("csrw pmpcfg11, %0" :: "r"(val)); break;
        //case 12: __asm volatile("csrw pmpcfg12, %0" :: "r"(val)); break;
        //case 13: __asm volatile("csrw pmpcfg13, %0" :: "r"(val)); break;
        //case 14: __asm volatile("csrw pmpcfg14, %0" :: "r"(val)); break;
        //case 15: __asm volatile("csrw pmpcfg15, %0" :: "r"(val)); break;
        default: configASSERT( pdFALSE );
    }
}

void vPortRestorePMPSettings( void )
{
    extern TaskHandle_t pxCurrentTCB;
    xMPU_SETTINGS * pxMPUSettings = xTaskGetMPUSettings( pxCurrentTCB );
    xPMP_SETTINGS * pmp;
    uint32_t used;
    uint32_t i;

    if ( pxMPUSettings->xTaskIsPrivileged == pdTRUE )
    {
        return;
    }

    pmp  = &(pxMPUSettings->xPMPSettings);

    used = pmp->pmp_cfg_used;

    /* Clear all pmpcfg first */
    for( i = 0; i < portPMP_NUM_CONFIG; i++ )
    {
        pmp_write_pmpcfg( i, 0 );
    }

    for( i = 0; i < used; i++ )
    {
        pmp_write_pmpaddr( i, pmp->pmpaddr[ i ] );
    }

    for( i = 0; i < ( ( used + 3U ) / 4U ); i++ )
    {
        pmp_write_pmpcfg( i, pmp->pmpcfg[ i ] );
    }
}

BaseType_t xPortIsAuthorizedToAccessBuffer( const void * pvBuffer,
                                            uint32_t ulBufferLength,
                                            uint32_t ulAccessRequested ) PRIVILEGED_FUNCTION;

BaseType_t xPortIsAuthorizedToAccessBuffer( const void * pvBuffer,
                                            uint32_t ulBufferLength,
                                            uint32_t ulAccessRequested ) /* PRIVILEGED_FUNCTION */
{
    uint32_t i, ulBufferStartAddress, ulBufferEndAddress;
    const xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL ); /* Calling task's MPU settings. */
    const xPMP_SETTINGS * pmp = &( xTaskMpuSettings->xPMPSettings );

    if( xSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the kernel objects before the scheduler
         * is started. It is necessary because there is no task running
         * yet and therefore, we cannot use the permissions of any
         * task. */
        return pdTRUE;
    }

    if( xTaskMpuSettings->xTaskIsPrivileged == pdTRUE )
    {
        return pdTRUE;
    }

    if( ulBufferLength == 0 )
    {
        return pdTRUE;
    }

    if( (uint32_t) pvBuffer > UINT32_MAX - ulBufferLength )
    {
        /* Overflow */
        return pdFALSE;
    }

    BaseType_t xAccessGranted = pdFALSE;
    {
        ulBufferStartAddress = ( uint32_t ) pvBuffer;
        ulBufferEndAddress = ( ( ( uint32_t ) pvBuffer ) + ulBufferLength - 1UL );
        for( i = 0; i < pmp->pmp_cfg_used; i++ )
        {
            uint32_t ulRegionStartAddress, ulRegionEndAddress;
            uint32_t ulConfigIndex = i / 4;
            uint32_t ulConfigShift = ( i % 4 ) * 8;
            uint8_t  pmpcfg = ( pmp->pmpcfg[ ulConfigIndex ] >> ulConfigShift ) & 0xFFU;

            configASSERT( ( pmpcfg & (3U << 3) )  == portPMP_A_NAPOT );

            pmp_decode_napot_addr( pmp->pmpaddr[ i ],
                                   &ulRegionStartAddress,
                                   &ulRegionEndAddress );

            /* no overlap */
            if( ulBufferEndAddress < ulRegionStartAddress || ulRegionEndAddress <= ulBufferStartAddress)
            {
                continue;
            }

            /* overlap but permission denied */
            if( pmp_check_access( pmpcfg, ulAccessRequested ) != pdPASS )
            {
                xAccessGranted = pdFALSE;
                break;
            }

            /* fully contains and allow -> OK */
            if( ulRegionStartAddress <= ulBufferStartAddress && ulBufferEndAddress < ulRegionEndAddress)
            {
                xAccessGranted = pdTRUE;
                break;
            }
        }
    }
    return xAccessGranted;
}

BaseType_t xPortIsAuthorizedToAccessKernelObject( int32_t lInternalIndexOfKernelObject ) PRIVILEGED_FUNCTION;

BaseType_t xPortIsAuthorizedToAccessKernelObject( int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    BaseType_t xAccessGranted = pdFALSE;
    const xMPU_SETTINGS * xTaskMpuSettings;

    if( xSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the kernel objects before the scheduler
         * is started. It is necessary because there is no task running
         * yet and therefore, we cannot use the permissions of any
         * task. */
        xAccessGranted = pdTRUE;
    }
    else
    {
        xTaskMpuSettings = xTaskGetMPUSettings( NULL ); /* Calling task's MPU settings. */

        ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
        ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

        if( xTaskMpuSettings->xTaskIsPrivileged == pdTRUE )
        {
            xAccessGranted = pdTRUE;
        }
        else
        {
            if( ( xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] & ( 1U << ulAccessControlListEntryBit ) ) != 0 )
            {
                xAccessGranted = pdTRUE;
            }
        }
    }

    return xAccessGranted;
}

void vPortGrantAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                     int32_t lInternalIndexOfKernelObject ) PRIVILEGED_FUNCTION;

void vPortGrantAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                     int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    xMPU_SETTINGS * xTaskMpuSettings;

    ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
    ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

    xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

    xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] |= ( 1U << ulAccessControlListEntryBit );
}

void vPortRevokeAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                      int32_t lInternalIndexOfKernelObject ) PRIVILEGED_FUNCTION;

void vPortRevokeAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                      int32_t lInternalIndexOfKernelObject ) /* PRIVILEGED_FUNCTION */
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    xMPU_SETTINGS * xTaskMpuSettings;

    ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject / portACL_ENTRY_SIZE_BITS );
    ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject % portACL_ENTRY_SIZE_BITS );

    xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

    xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] &= ~( 1U << ulAccessControlListEntryBit );
}
/*-----------------------------------------------------------*/
