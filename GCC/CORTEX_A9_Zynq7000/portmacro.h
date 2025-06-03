/*
 * FreeRTOS Kernel V10.6.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
	extern "C" {
#endif

/* BSP includes. */
#include "xil_types.h"
#if ( configNUMBER_OF_CORES > 1 )
#include "xscugic.h"
#include "xil_spinlock.h"
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the given hardware
 * and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR                    char
#define portFLOAT                   float
#define portDOUBLE                  double
#define portLONG                    long
#define portSHORT                   short
#define portSTACK_TYPE              uint32_t
#define portBASE_TYPE               long

typedef portSTACK_TYPE              StackType_t;
typedef portBASE_TYPE               BaseType_t;
typedef unsigned portBASE_TYPE      UBaseType_t;

typedef uint32_t                    TickType_t;
#define portMAX_DELAY               ( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC     1

/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portSTACK_GROWTH            ( -1 )
#define portTICK_PERIOD_MS          ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT          8
#define portPOINTER_SIZE_TYPE       uint32_t
#define portMAX_CORE_COUNT          2
#define portCORE0                   0U
#define portCORE1                   1U

/* In all GICs 255 can be written to the priority mask register to unmask all
(but the lowest) interrupt priority. */
#define portUNMASK_VALUE                ( 0xFFUL )

/* Interrupt controller access addresses. */
#define portICCPMR_PRIORITY_MASK_OFFSET                     ( 0x04 )
#define portICCIAR_INTERRUPT_ACKNOWLEDGE_OFFSET             ( 0x0C )
#define portICCEOIR_END_OF_INTERRUPT_OFFSET                 ( 0x10 )
#define portICCBPR_BINARY_POINT_OFFSET                      ( 0x08 )
#define portICCRPR_RUNNING_PRIORITY_OFFSET                  ( 0x14 )

#define portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS      ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET )
#define portICCPMR_PRIORITY_MASK_REGISTER                   ( *( ( volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCPMR_PRIORITY_MASK_OFFSET ) ) )
#define portICCIAR_INTERRUPT_ACKNOWLEDGE_REGISTER_ADDRESS   ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCIAR_INTERRUPT_ACKNOWLEDGE_OFFSET )
#define portICCEOIR_END_OF_INTERRUPT_REGISTER_ADDRESS       ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCEOIR_END_OF_INTERRUPT_OFFSET )
#define portICCPMR_PRIORITY_MASK_REGISTER_ADDRESS           ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCPMR_PRIORITY_MASK_OFFSET )
#define portICCBPR_BINARY_POINT_REGISTER                    ( *( ( const volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCBPR_BINARY_POINT_OFFSET ) ) )
#define portICCRPR_RUNNING_PRIORITY_REGISTER                ( *( ( volatile uint32_t * ) ( portINTERRUPT_CONTROLLER_CPU_INTERFACE_ADDRESS + portICCRPR_RUNNING_PRIORITY_OFFSET ) ) )    

/*-----------------------------------------------------------*/

/* Task utilities. */

/* Called at the end of an ISR that can cause a context switch. */
#if ( configNUMBER_OF_CORES == 1 )
    extern volatile uint32_t ulPortYieldRequired;
    #define portEND_SWITCHING_ISR( xSwitchRequired )\
    {												\
    												\
    	if( xSwitchRequired != pdFALSE )			\
    	{								            \
            traceISR_EXIT_TO_SCHEDULER();			\
    		ulPortYieldRequired = pdTRUE;			\
    	}											\
        else                                        \
        {                                           \
            traceISR_EXIT();                        \
        }                                           \
    }
#else
    extern volatile uint32_t ulPortYieldRequired[ configNUMBER_OF_CORES ];
    #define portEND_SWITCHING_ISR( xSwitchRequired )            \
    {												            \
    												            \
    	if( xSwitchRequired != pdFALSE )			            \
    	{								                        \
            traceISR_EXIT_TO_SCHEDULER();			            \
    		ulPortYieldRequired[ portGET_CORE_ID() ] = pdTRUE;  \
    	}											            \
        else                                                    \
        {                                                       \
            traceISR_EXIT();                                    \
        }                                                       \
    }

#endif

#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
#define portYIELD() __asm volatile ( "SWI 0" ::: "memory" );

/*-----------------------------------------------------------*/

/* Symmetric MultiProcessing (SMP) utility */
#if ( configNUMBER_OF_CORES > 1 )

    /* Most of the following definitions and assembly instructions 
    can be found inside the Xilinx headers xpseudo_asm_gcc.h and
    xreg_cortexa9.h. However, they are redefined here to ease the
    readability of the code and its portability to other Cortex-A9
    based devices. */
    #define portMPIDR_AFFINITY_MASK             0x3U
    
    /* Get the ID of the core that is executing the application.
    The library function XGetCoreId() in xplatform_info.h performs
    the same operation. */
    static inline BaseType_t xPortGetCoreID (void)
    {
        BaseType_t xCoreID;
        __asm volatile( "MRC p15, 0, %[coreID], c0, c0, 5   \n\t"
                        "AND %[coreID], %[coreID], %[mask]  \n\t"
                        : [coreID] "=r" (xCoreID) : [mask] "I" (portMPIDR_AFFINITY_MASK));
        return xCoreID;
    }
    
    #define portGET_CORE_ID()               xPortGetCoreID()
    
    /* Force core x to yield. */
    extern XScuGic xInterruptControllers[configNUMBER_OF_CORES];
    #define portSGI_ID                      0
    #define portYIELD_CORE( x )             XScuGic_SoftwareIntr(                               \
                                                &xInterruptControllers[portGET_CORE_ID()],      \
                                                portSGI_ID, /* Interrupt ID */                  \
                                                (1 << x)    /* Target core */                   \
                                            );
    
    /* Macro to initialize the Xil_spinlock API */
    extern u32 Xil_InitializeSpinLock( UINTPTR lockaddr, UINTPTR lockflagaddr, u32 lockflag );
    extern uint32_t ulXilSpinlockAddr;      /* Declared in port.c */
    extern uint32_t ulXilSpinlockFlagAddr;
    #define portXIL_SPINLOCK_INIT()   Xil_InitializeSpinLock(                 \
                                            (UINTPTR) &ulXilSpinlockAddr,       \
                                            (UINTPTR) &ulXilSpinlockFlagAddr,   \
                                            XIL_SPINLOCK_ENABLE)

#endif


/* NOTE: portCHECK_IF_IN_ISR() is used in vApplicationAssert even in the
single-core configuration. */

#define portICCRPR_RUNNING_PRIORITY_MASK    0xFFU
#define portICCRPR_IDLE_PRIORITY            0xFFU

/* Check if the core is executing an ISR or not.
Returns pdFALSE in case the core is not executing an ISR.

The ICC_RPR is a GIC register and it is core specific so it 
cannot be used for system-wide solutions (e.g. interrupts handled 
by other cores or not managed through the GIC). */
static inline BaseType_t xPortCheckIfInISR(void)
{
    /* The ICCRPR is updated by the GIC so its content should be
    stable if we call this function from an interrupt. It changes
    only when a valid write to the ICCEOIR is performed or an higher
    priority interrupt preempts the current interrupt. The interrupts
    are disabled before writing to the ICCEOIR in FreeRTOS_IRQ_Handler
    so interrupts cannot nest after updating the ICCEOIR. */
    return (portICCRPR_RUNNING_PRIORITY_REGISTER & portICCRPR_RUNNING_PRIORITY_MASK) != portICCRPR_IDLE_PRIORITY;
}

#define portCHECK_IF_IN_ISR()           xPortCheckIfInISR()

/*-----------------------------------------------------------
 * Critical section control
 *----------------------------------------------------------*/
extern uint32_t ulPortSetInterruptMask( void );
extern void vPortClearInterruptMask( uint32_t ulNewMaskValue );
extern void vPortInstallFreeRTOSVectorTable( void );
/* These macros do not globally disable/enable interrupts.  They do mask off
interrupts that have a priority below configMAX_API_CALL_INTERRUPT_PRIORITY. */
#define portSET_INTERRUPT_MASK_FROM_ISR()	    ulPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )  vPortClearInterruptMask( x )

#if( configNUMBER_OF_CORES == 1 )

    #define portDISABLE_INTERRUPTS()                ulPortSetInterruptMask()
    #define portENABLE_INTERRUPTS()                 vPortClearInterruptMask( pdFALSE )
    
    /* Critical section entry/exit macros */
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );
    
    #define portENTER_CRITICAL()                    vPortEnterCritical()
    #define portEXIT_CRITICAL()                     vPortExitCritical()
    
#else /* #if( configNUMBER_OF_CORES == 1 ) */

    #define portDISABLE_INTERRUPTS()                ulPortSetInterruptMask()
    #define portENABLE_INTERRUPTS()                 vPortClearInterruptMask( portUNMASK_VALUE )
    #define portSET_INTERRUPT_MASK()                ulPortSetInterruptMask()
    #define portCLEAR_INTERRUPT_MASK( x )           vPortClearInterruptMask( x )

    /* Critical section entry/exit macros */
    extern void vTaskEnterCritical( void );
    extern void vTaskExitCritical( void );
    extern UBaseType_t vTaskEnterCriticalFromISR( void );
    extern void vTaskExitCriticalFromISR( UBaseType_t uxSavedInterruptStatus );
    #define portENTER_CRITICAL()                    vTaskEnterCritical()
    #define portEXIT_CRITICAL()                     vTaskExitCritical()
    #define portENTER_CRITICAL_FROM_ISR()           vTaskEnterCriticalFromISR()
    #define portEXIT_CRITICAL_FROM_ISR( x )         vTaskExitCriticalFromISR( x )


    /* Lock macros */
    #define portLOCK_COUNT  2U

    #define portISR_LOCK    0U
    #define portTASK_LOCK   1U

    /* The value of this macro has to be the same of the one
    defined inside the assembly files. */
    #define portLOCK_FREE   2U

    typedef struct{
        uint32_t ulLock;
        uint32_t ulRecursionCount;
    } lock_t;

    #if( configUSE_TRACE_MACROS == 1 && configTRACE_RECURSIVE_LOCKS == 1 )
        extern void vPortRecursiveLock(UBaseType_t uxLock, BaseType_t xAcquire, BaseType_t xCoreID );

        /* vPortRecursiveLock() is defined in trace.c and it is exclusively used
        to trace the operations on the ISR and TASK locks. */
        #define portGET_TASK_LOCK( xCoreID )            vPortRecursiveLock(portTASK_LOCK,pdTRUE ,xCoreID)         
        #define portGET_ISR_LOCK( xCoreID )             vPortRecursiveLock(portISR_LOCK ,pdTRUE ,xCoreID)
        #define portRELEASE_TASK_LOCK( xCoreID )        vPortRecursiveLock(portTASK_LOCK,pdFALSE,xCoreID)
        #define portRELEASE_ISR_LOCK( xCoreID )         vPortRecursiveLock(portISR_LOCK ,pdFALSE,xCoreID)
    #else /* #if( configUSE_TRACE_MACROS == 1 ) */
        extern void vPortGetLock(volatile lock_t* xLockAddr, BaseType_t xCoreID );
        extern void vPortReleaseLock(volatile lock_t* xLockAddr, BaseType_t xCoreID );
        extern volatile lock_t xLocks[portLOCK_COUNT];
    
        #define portGET_TASK_LOCK( xCoreID )            vPortGetLock(&xLocks[portTASK_LOCK], xCoreID)         
        #define portGET_ISR_LOCK( xCoreID )             vPortGetLock(&xLocks[portISR_LOCK], xCoreID)
        #define portRELEASE_TASK_LOCK( xCoreID )        vPortReleaseLock(&xLocks[portTASK_LOCK], xCoreID)
        #define portRELEASE_ISR_LOCK( xCoreID )         vPortReleaseLock(&xLocks[portISR_LOCK], xCoreID)
    #endif /* #if( configUSE_TRACE_MACROS == 1 ) */


    /* Restores the saved interrupt mask returned by portDISABLE_INTERRUPTS().
    Functionally equivalent to portCLEAR_INTERRUPT_MASK/FROM_ISR(). */
    #define portRESTORE_INTERRUPTS( x )     vPortClearInterruptMask( x )


    /* Critical nesting count management. */
    extern volatile uint32_t ulCriticalNesting[ configNUMBER_OF_CORES ];
    #define portGET_CRITICAL_NESTING_COUNT( xCoreID )           ( ulCriticalNesting[ xCoreID ] )
    #define portSET_CRITICAL_NESTING_COUNT( xCoreID, x )        ( ulCriticalNesting[ xCoreID ] = ( x ) )
    #define portINCREMENT_CRITICAL_NESTING_COUNT( xCoreID )     ( ulCriticalNesting[ xCoreID ]++ )
    #define portDECREMENT_CRITICAL_NESTING_COUNT( xCoreID )     ( ulCriticalNesting[ xCoreID ]-- )


    /* Assert if portENTER_CRITICAL() is called from an ISR instead of
    portENTER_CRITICAL_FROM_ISR(). */
    #define portASSERT_IF_IN_ISR()          configASSERT( !portCHECK_IF_IN_ISR() );
#endif /* #if( configNUMBER_OF_CORES == 1 ) */


/* The critical nesting count is managed by the port.
This macro must be set to 0. */
#define portCRITICAL_NESTING_IN_TCB             0

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not required for this port but included in case common demo code that uses these
macros is used. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )  void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )        void vFunction( void *pvParameters )

/* Prototype of the FreeRTOS tick handler.  This must be installed as the
handler for whichever peripheral is used to generate the RTOS tick. */
void FreeRTOS_Tick_Handler( void );

/*
 * Installs pxHandler as the interrupt handler for the peripheral specified by
 * the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have pxHandler assigned as its interrupt
 * handler.  Peripheral IDs are defined in the xparameters.h header file, which
 * is itself part of the BSP project.
 *
 * pxHandler:
 *
 * A pointer to the interrupt handler function itself.  This must be a void
 * function that takes a (void *) parameter.
 *
 * pvCallBackRef:
 *
 * The parameter passed into the handler function.  In many cases this will not
 * be used and can be NULL.  Some times it is used to pass in a reference to
 * the peripheral instance variable, so it can be accessed from inside the
 * handler function.
 *
 * pdPASS is returned if the function executes successfully.  Any other value
 * being returned indicates that the function did not execute correctly.
 */

#if ( configNUMBER_OF_CORES == 1 )

#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef );
#else
BaseType_t xPortInstallInterruptHandler( uint16_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef );
#endif
/*
 * Enables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt enabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
void vPortEnableInterrupt( uint8_t ucInterruptID );
#else
void vPortEnableInterrupt( uint16_t ucInterruptID );
#endif
/*
 * Disables the interrupt, within the interrupt controller, for the peripheral
 * specified by the ucInterruptID parameter.
 *
 * ucInterruptID:
 *
 * The ID of the peripheral that will have its interrupt disabled in the
 * interrupt controller.  Peripheral IDs are defined in the xparameters.h header
 * file, which is itself part of the BSP project.
 */
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
void vPortDisableInterrupt( uint8_t ucInterruptID );
#else
void vPortDisableInterrupt( uint16_t ucInterruptID );
#endif

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

/* The interrupt management API for multi-core configurations are rawer than
the single-core API but they also allow higher integration in the application
code and higher performances. 

Additional features:

    - Redundant checks are executed with configASSERT() in order to exclude them
    in production code.

    - Interrupts can now be fully configured.

    - Compliance to multi-core constraints of interrupt enabling/disabling
    is checked by the API. This is necessary due to limitations of the scugic driver.  

    - SPIs can be enabled on one core from other cores.


NOTE: interrupt controller instance intialization, handler installation and
interrupt configuration do not need a parameter to identify the target core.
The init of an IC instance needs to be done on every core that will use the
GIC while handler installation and interrupt configuration can be done from
any core remembering that only SPIs can be configured from other cores. 
Additional checks are implemented in vPortEnableInterrupt.
*/

/* Initialise the interrupt controller instance of the current core. */
void vInitialiseInterruptController( void );

/* Install the interrupt handler for a certain interrupt. */
void vPortInstallInterruptHandler( UBaseType_t uxInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef );

/* Configure an interrupt. */
void vPortConfigureInterrupt( UBaseType_t uxInterruptID, uint8_t ucInterruptPriority , uint8_t ucInterruptTrigger );

/* Enable an interrupt on the core <xTargetCore>.
Only SPIs can be enabled from other cores. */
void vPortEnableInterrupt( UBaseType_t uxInterruptID, BaseType_t xTargetCore );

/* Disable an interrupt on the core <xTargetCore>.
Only SPIs can be disabled from other cores. */
void vPortDisableInterrupt( UBaseType_t uxInterruptID, BaseType_t xTargetCore );

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/* If configUSE_TASK_FPU_SUPPORT is set to 1 (or left undefined) then tasks are
created without an FPU context and must call vPortTaskUsesFPU() to give
themselves an FPU context before using any FPU instructions.  If
configUSE_TASK_FPU_SUPPORT is set to 2 then all tasks will have an FPU context
by default. */
#if( configUSE_TASK_FPU_SUPPORT != 2 )
    void vPortTaskUsesFPU( void );
#else
    /* Each task has an FPU context already, so define this function away to
    nothing to prevent it being called accidentally. */
    #define vPortTaskUsesFPU()
#endif
#define portTASK_USES_FLOATING_POINT()  vPortTaskUsesFPU()

/* These macros do not take into account the unused priority bits. */
#define portLOWEST_INTERRUPT_PRIORITY           ( ( ( uint32_t ) configUNIQUE_INTERRUPT_PRIORITIES ) - 1UL )
#define portLOWEST_USABLE_INTERRUPT_PRIORITY    ( portLOWEST_INTERRUPT_PRIORITY - 1UL )

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #if ( configNUMBER_OF_CORES == 1 )
        #define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
    #else
        #define configUSE_PORT_OPTIMISED_TASK_SELECTION 0 
    #endif
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

    /* Store/clear the ready priorities in a bit map. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

    /*-----------------------------------------------------------*/

    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31UL - ( uint32_t ) __builtin_clz( uxReadyPriorities ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

#if ( configASSERT_DEFINED == 1 )
    void vPortValidateInterruptPriority( void );
    #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()  vPortValidateInterruptPriority()
#endif /* configASSERT */

/* Optimization control */
#define portNOP()               __asm volatile( "NOP" )
#define portINLINE              __inline
#define portDONT_DISCARD        __attribute__( ( used ) )
#define portMEMORY_BARRIER()    __asm volatile( "" ::: "memory" )

/* The number of bits to shift for an interrupt priority is dependent on the
number of bits implemented by the interrupt controller. */
#if configUNIQUE_INTERRUPT_PRIORITIES == 16
    #define portPRIORITY_SHIFT 4
    #define portMAX_BINARY_POINT_VALUE  3
#elif configUNIQUE_INTERRUPT_PRIORITIES == 32
    #define portPRIORITY_SHIFT 3
    #define portMAX_BINARY_POINT_VALUE  2
#elif configUNIQUE_INTERRUPT_PRIORITIES == 64
    #define portPRIORITY_SHIFT 2
    #define portMAX_BINARY_POINT_VALUE  1
#elif configUNIQUE_INTERRUPT_PRIORITIES == 128
    #define portPRIORITY_SHIFT 1
    #define portMAX_BINARY_POINT_VALUE  0
#elif configUNIQUE_INTERRUPT_PRIORITIES == 256
    #define portPRIORITY_SHIFT 0
    #define portMAX_BINARY_POINT_VALUE  0
#else /* if configUNIQUE_INTERRUPT_PRIORITIES == 16 */
    #error "Invalid configUNIQUE_INTERRUPT_PRIORITIES setting.  configUNIQUE_INTERRUPT_PRIORITIES must be set to the number of unique priorities implemented by the target hardware"
#endif /* if configUNIQUE_INTERRUPT_PRIORITIES == 16 */

/* Run Time Statistics */

#if ( configGENERATE_RUN_TIME_STATS == 1 )
    void xCONFIGURE_TIMER_FOR_RUN_TIME_STATS(void);
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    xCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
    uint32_t xGET_RUN_TIME_COUNTER_VALUE(void);
    #define portGET_RUN_TIME_COUNTER_VALUE()            xGET_RUN_TIME_COUNTER_VALUE()
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
