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

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Xilinx includes. */
#include "xscugic.h"

#if defined(XPAR_XILTIMER_ENABLED) || defined(SDT)
#include "bspconfig.h"
#include "xinterrupt_wrap.h"
#endif

#ifndef configINTERRUPT_CONTROLLER_BASE_ADDRESS
    #error configINTERRUPT_CONTROLLER_BASE_ADDRESS must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET
    #error configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configUNIQUE_INTERRUPT_PRIORITIES
    #error configUNIQUE_INTERRUPT_PRIORITIES must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#ifndef configSETUP_TICK_INTERRUPT
    #error configSETUP_TICK_INTERRUPT() must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif /* configSETUP_TICK_INTERRUPT */

#ifndef configMAX_API_CALL_INTERRUPT_PRIORITY
    #error configMAX_API_CALL_INTERRUPT_PRIORITY must be defined.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-Cortex-A-Embedded-Processors.html
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY == 0
    #error configMAX_API_CALL_INTERRUPT_PRIORITY must not be set to 0
#endif

#if configMAX_API_CALL_INTERRUPT_PRIORITY > configUNIQUE_INTERRUPT_PRIORITIES
    #error configMAX_API_CALL_INTERRUPT_PRIORITY must be less than or equal to configUNIQUE_INTERRUPT_PRIORITIES as the lower the numeric priority value the higher the logical interrupt priority
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
    /* Check the configuration. */
    #if( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
    #endif
#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* In case security extensions are implemented. */
#if configMAX_API_CALL_INTERRUPT_PRIORITY <= ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
    #error configMAX_API_CALL_INTERRUPT_PRIORITY must be greater than ( configUNIQUE_INTERRUPT_PRIORITIES / 2 )
#endif

#if ( configNUMBER_OF_CORES > portMAX_CORE_COUNT || configNUMBER_OF_CORES < 1 )
    #error Number of cores not supported: configNUMBER_OF_CORES cannot be lower than 1 or higher than portMAX_CORE_COUNT
#endif

#if ( ( configNUMBER_OF_CORES > 1 ) && ( defined( XPAR_XILTIMER_ENABLED ) || defined( SDT ) ))
    #error Symmetric MultiProcessing is not supported yet in the current configuration
#endif

/* Some vendor specific files default configCLEAR_TICK_INTERRUPT() in
portmacro.h. */
#ifndef configCLEAR_TICK_INTERRUPT
    #define configCLEAR_TICK_INTERRUPT()
#endif

/* A critical section is exited when the critical section nesting count reaches
this value. */
#define portNO_CRITICAL_NESTING         ( ( uint32_t ) 0 )

/* Tasks are not created with a floating point context, but can be given a
floating point context after they have been created.  A variable is stored as
part of the tasks context that holds portNO_FLOATING_POINT_CONTEXT if the task
does not have an FPU context, or any other value if the task does have an FPU
context. */
#define portNO_FLOATING_POINT_CONTEXT   ( ( StackType_t ) 0 )

/* Constants required to setup the initial task context. */
#define portINITIAL_SPSR                ( ( StackType_t ) 0x1f ) /* System mode, ARM mode, IRQ enabled FIQ enabled. */
#define portTHUMB_MODE_BIT              ( ( StackType_t ) 0x20 )
#define portINTERRUPT_ENABLE_BIT        ( 0x80UL )
#define portTHUMB_MODE_ADDRESS          ( 0x01UL )

/* Used by portASSERT_IF_INTERRUPT_PRIORITY_INVALID() when ensuring the binary
point is zero. */
#define portBINARY_POINT_BITS           ( ( uint8_t ) 0x03 )

/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK         ( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_USER_MODE              ( 0x10 )

/* The critical section macros only mask interrupts up to an application
determined priority level.  Sometimes it is necessary to turn interrupt off in
the CPU itself before modifying certain hardware registers. */
#define portCPU_IRQ_DISABLE()                                       \
    __asm volatile ( "CPSID i" ::: "memory" );                      \
    __asm volatile ( "DSB" );                                       \
    __asm volatile ( "ISB" );

#define portCPU_IRQ_ENABLE()                                        \
    __asm volatile ( "CPSIE i" ::: "memory" );                      \
    __asm volatile ( "DSB" );                                       \
    __asm volatile ( "ISB" );

#if ( configNUMBER_OF_CORES == 1 )

    /* Unmask the interrupts in the Priority Mask Register. */
    #define portCLEAR_INTERRUPT_MASK()                                  \
    {                                                                   \
        portCPU_IRQ_DISABLE();                                          \
        portICCPMR_PRIORITY_MASK_REGISTER = portUNMASK_VALUE;           \
        __asm volatile (    "DSB        \n"                             \
                            "ISB        \n" );                          \
        portCPU_IRQ_ENABLE();                                           \
    }

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

    /* Check if the interrupts are disabled on the current core.
    The return value is (0) if interrupts are enabled, another value
    otherwise. */
    #define portCHECK_IF_INTERRUPTS_DISABLED()                      \
    ({                                                              \
        uint32_t ulInterruptsDisabled;                              \
        __asm volatile (                                            \
            "MRS    %[IntDisabled], cpsr                    \n\t"   \
            "AND    %[IntDisabled], %[IntDisabled], %[iMask]\n\t"   \
        :[IntDisabled] "=r" (ulInterruptsDisabled)                  \
        :[iMask] "I" (1 << 7)                                       \
        :                                                           \
        );                                                          \
        ulInterruptsDisabled;                                       \
    })

    /* Check if the Interrupt Controller instance is intialized. */
    #define portCHECK_IF_IC_INITIALIZED( xCoreID )                                                          \
    ({                                                                                                      \
        BaseType_t xStatus;                                                                                 \
        xStatus = (xInterruptControllers[ xCoreID ].IsReady == XIL_COMPONENT_IS_READY)? pdTRUE: pdFALSE;    \
        xStatus;                                                                                            \
    })

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

#define portINTERRUPT_PRIORITY_REGISTER_OFFSET      0x400UL
#define portMAX_8_BIT_VALUE                         ( ( uint8_t ) 0xff )
#define portBIT_0_SET                               ( ( uint8_t ) 0x01 )

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS

/* The space on the stack required to hold the FPU registers.  This is 32 64-bit
registers, plus a 32-bit status register. */
#define portFPU_REGISTER_WORDS  ( ( 32 * 2 ) + 1 )

/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES == 1 )

/*
 * Initialise the interrupt controller instance.
 */
static int32_t prvInitialiseInterruptController( void );

/* Ensure the interrupt controller instance variable is initialised before it is
 * used, and that the initialisation only happens once.
 */
static int32_t prvEnsureInterruptControllerIsInitialised( void );

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

/*
 * The primary core wakes up the secondary core and waits
 * until it is initialized.
 */
extern void vPortLaunchSecondaryCore( void ); 

/*
 * Installs the handler for the SGI generated by
 * portYIELD_CORE(x).
 */
extern void vPortSetupYieldRequestHandler( BaseType_t xCoreID );

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*
 * Starts the first task executing.  This function is necessarily written in
 * assembly code so is implemented in portASM.s.
 */
extern void vPortRestoreTaskContext( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*
 * If the application provides an implementation of vApplicationIRQHandler(),
 * then it will get called directly without saving the FPU registers on
 * interrupt entry, and this weak implementation of
 * vApplicationFPUSafeIRQHandler() is just provided to remove linkage errors -
 * it should never actually get called so its implementation contains a
 * call to configASSERT() that will always fail.
 *
 * If the application provides its own implementation of
 * vApplicationFPUSafeIRQHandler() then the implementation of
 * vApplicationIRQHandler() provided in portASM.S will save the FPU registers
 * before calling it.
 *
 * Therefore, if the application writer wants FPU registers to be saved on
 * interrupt entry their IRQ handler must be called
 * vApplicationFPUSafeIRQHandler(), and if the application writer does not want
 * FPU registers to be saved on interrupt entry their IRQ handler must be
 * called vApplicationIRQHandler().
 */
void vApplicationFPUSafeIRQHandler( uint32_t ulICCIAR ) __attribute__((weak) );

/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES == 1 )

    /* 
    * The instance of the interrupt controller used by this port. This is required
    * by the Xilinx library API functions.
    */
    #if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
        XScuGic xInterruptController;
    #else
        uintptr_t IntrControllerAddr = configINTERRUPT_CONTROLLER_BASE_ADDRESS;
    #endif

    /* A variable is used to keep track of the critical section nesting. This
    variable has to be stored as part of the task context and must be initialised to
    a non zero value to ensure interrupts don't inadvertently become unmasked before
    the scheduler starts.  As it is stored as part of the task context it will
    automatically be set to 0 when the first task is started. */
    volatile uint32_t ulCriticalNesting = 9999UL;

    /* Counts the interrupt nesting depth. A context switch is only performed if
    if the nesting depth is 0. */
    volatile uint32_t ulPortInterruptNesting = 0UL;
    
    /* Set to 1 to pend a context switch from an ISR. */
    volatile uint32_t ulPortYieldRequired = pdFALSE;

    /* Saved as part of the task context.  If ulPortTaskHasFPUContext is non-zero then
    a floating point context must be saved and restored for the task. */
    volatile uint32_t ulPortTaskHasFPUContext = pdFALSE;

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

    /* The ScuGic driver does not allow to handle separately multiple
    cores with the same interrupt controller instance. */
    XScuGic xInterruptControllers[configNUMBER_OF_CORES];

    /* The Xil_spinlock API is used inside the ScuGic driver to
    ensure shared registers are accessed by one core at a time.
    This lock is not recursive so claiming it multiple times may
    lead to a deadlock. Check xil_spinlock.c for more info. */
    uint32_t ulXilSpinlockAddr, ulXilSpinlockFlagAddr;

    /* Recursive ISR and TASK spinlocks */
    volatile lock_t xLocks[portLOCK_COUNT] = { [0 ... (portLOCK_COUNT - 1)] = {portLOCK_FREE, 0}};


    /* Since different tasks can run concurrently on different cores, it is
     * necessary to record the critical nesting count for each core. The
     * same consideration can be made for the variables that hold
     * the interrupt nesting, context switch requested from an ISR and 
     * tasks FPU context. */ 
    
    /* The initialization syntax of ulCriticalNesting is a GNU extension
    supported by GCC and Clang when using C99 or later standards. */
    volatile uint32_t ulCriticalNesting[configNUMBER_OF_CORES] = {[0 ... (configNUMBER_OF_CORES - 1)] = 9999UL};
    volatile uint32_t ulPortInterruptNesting[configNUMBER_OF_CORES] = { 0 };
    volatile uint32_t ulPortYieldRequired[configNUMBER_OF_CORES] = { pdFALSE };
    volatile uint32_t ulPortTaskHasFPUContext[configNUMBER_OF_CORES] = { pdFALSE };

    /* Flag the awakening of the secondary core */
    volatile uint32_t ulSecondaryCoreAwake = pdFALSE;

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*
 * Global counter used for calculation of run time statistics of tasks.
 * Defined only when the relevant option is turned on
 */
#if (configGENERATE_RUN_TIME_STATS==1)
volatile uint32_t ulHighFrequencyTimerTicks;
#endif

/* Used in the asm file. */
portDONT_DISCARD const uint32_t ulICCIAR = portICCIAR_INTERRUPT_ACKNOWLEDGE_REGISTER_ADDRESS;
portDONT_DISCARD const uint32_t ulICCEOIR = portICCEOIR_END_OF_INTERRUPT_REGISTER_ADDRESS;
portDONT_DISCARD const uint32_t ulICCPMR = portICCPMR_PRIORITY_MASK_REGISTER_ADDRESS;
portDONT_DISCARD const uint32_t ulMaxAPIPriorityMask = ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    /* Setup the initial stack of the task.  The stack is set exactly as
    expected by the portRESTORE_CONTEXT() macro.

    The fist real value on the stack is the status register, which is set for
    system mode, with interrupts enabled.  A few NULLs are added first to ensure
    GDB does not try decoding a non-existent return address. */
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) NULL;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) portINITIAL_SPSR;

    if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x00UL )
    {
        /* The task will start in THUMB mode. */
        *pxTopOfStack |= portTHUMB_MODE_BIT;
    }

    pxTopOfStack--;

    /* Next the return address, which in this case is the start of the task. */
    *pxTopOfStack = ( StackType_t ) pxCode;
    pxTopOfStack--;

    /* Next all the registers other than the stack pointer. */
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;    /* R14 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x12121212; /* R12 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x11111111; /* R11 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x10101010; /* R10 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x09090909; /* R9 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x08080808; /* R8 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x07070707; /* R7 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x06060606; /* R6 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x05050505; /* R5 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x04040404; /* R4 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x03030303; /* R3 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x02020202; /* R2 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x01010101; /* R1 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
    pxTopOfStack--;

    /* The task will start with a critical nesting count of 0 as interrupts are
    enabled. */
    *pxTopOfStack = portNO_CRITICAL_NESTING;

    #if( configUSE_TASK_FPU_SUPPORT == 1 )
    {
        /* The task will start without a floating point context.  A task that
        uses the floating point hardware must call vPortTaskUsesFPU() before
        executing any floating point instructions. */
        pxTopOfStack--;
        *pxTopOfStack = portNO_FLOATING_POINT_CONTEXT;
    }
    #elif( configUSE_TASK_FPU_SUPPORT == 2 )
    {
        /* The task will start with a floating point context.  Leave enough
        space for the registers - and ensure they are initialised to 0. */
        pxTopOfStack -= portFPU_REGISTER_WORDS;
        memset( pxTopOfStack, 0x00, portFPU_REGISTER_WORDS * sizeof( StackType_t ) );

        pxTopOfStack--;
        *pxTopOfStack = pdTRUE;
        
        /* Assigning ulPortTaskHasFPUContext to pdTRUE should not be necessary
        since it is updated by popping pdTRUE from the stack when the task context
        is restored. */
        
        //#if ( configNUMBER_OF_CORES == 1 )
        //    ulPortTaskHasFPUContext = pdTRUE;
        //#else
        //    /* The affinity of the created task does not influence the
        //    value of the ulPortTaskHasFPUContext for a core since all tasks
        //    have FPU support when configUSE_TASK_FPU_SUPPORT == 2.*/
        //    ulPortTaskHasFPUContext[0] = ulPortTaskHasFPUContext[1] = pdTRUE;
        //#endif
    }
    #else
    {
        #error Invalid configUSE_TASK_FPU_SUPPORT setting - configUSE_TASK_FPU_SUPPORT must be set to 1, 2, or left undefined.
    }
    #endif

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    /* A function that implements a task must not exit or attempt to return to
    its caller as there is nothing to return to.  If a task wants to exit it
    should instead call vTaskDelete( NULL ).

    Artificially force an assert() to be triggered if configASSERT() is
    defined, then stop here so application writers can catch the error. */
	xil_printf("Warning: return statement has been called from task %s, deleting it\n",pcTaskGetName(NULL));
	if (uxTaskGetNumberOfTasks() == 2)
	{
		xil_printf("Warning: Kernel does not have any task to manage other than idle task\n");
	}
	vTaskDelete( NULL );
}
/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES == 1 )

#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
BaseType_t xPortInstallInterruptHandler( uint8_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef )
#else
BaseType_t xPortInstallInterruptHandler( uint16_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef )
#endif
{
int32_t lReturn;

	/* An API function is provided to install an interrupt handler */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
		lReturn = XScuGic_Connect( &xInterruptController, ucInterruptID, pxHandler, pvCallBackRef );
#else
      lReturn = XConnectToInterruptCntrl(ucInterruptID, pxHandler, pvCallBackRef, IntrControllerAddr);
#endif
	}
	if( lReturn == XST_SUCCESS )
	{
		lReturn = pdPASS;
	}
	configASSERT( lReturn == pdPASS );
	
	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvEnsureInterruptControllerIsInitialised( void )
{
static int32_t lInterruptControllerInitialised = pdFALSE;
int32_t lReturn = pdPASS;

	/* Ensure the interrupt controller instance variable is initialised before
	it is used, and that the initialisation only happens once. */
	if( lInterruptControllerInitialised != pdTRUE )
	{
		lReturn = prvInitialiseInterruptController();

		if( lReturn == pdPASS )
		{
			lInterruptControllerInitialised = pdTRUE;
		}
	}
	
	return lReturn;
}
/*-----------------------------------------------------------*/

static int32_t prvInitialiseInterruptController( void )
{
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
BaseType_t xStatus;
XScuGic_Config *pxGICConfig;

	/* Initialize the interrupt controller driver. */
	pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
	xStatus = XScuGic_CfgInitialize( &xInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );	
#else
    int xStatus;
    xStatus = XConfigInterruptCntrl(IntrControllerAddr);
#endif
	if( xStatus == XST_SUCCESS )
	{
		xStatus = pdPASS;
#if defined(XPAR_XILTIMER_ENABLED) || defined(SDT)
    XRegisterInterruptHandler(NULL, IntrControllerAddr);
    Xil_ExceptionInit();
    Xil_ExceptionEnable();
#endif
	}
	else
	{
		xStatus = pdFAIL;
	}	
	configASSERT( xStatus == pdPASS );

	return xStatus;
}
/*-----------------------------------------------------------*/
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
void vPortEnableInterrupt( uint8_t ucInterruptID )
#else
void vPortEnableInterrupt( uint16_t ucInterruptID )
#endif
{
int32_t lReturn;

	/* An API function is provided to enable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
	XScuGic_Enable( &xInterruptController, ucInterruptID );
#else
    XEnableIntrId(ucInterruptID, IntrControllerAddr);
#endif
	}	
	configASSERT( lReturn );
}
/*-----------------------------------------------------------*/

#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
void vPortDisableInterrupt( uint8_t ucInterruptID )
#else
void vPortDisableInterrupt( uint16_t ucInterruptID )
#endif
{
int32_t lReturn;

	/* An API function is provided to disable an interrupt in the interrupt
	controller. */
	lReturn = prvEnsureInterruptControllerIsInitialised();
	if( lReturn == pdPASS )
	{
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
		XScuGic_Disable( &xInterruptController, ucInterruptID );
#else
      XDisableIntrId(ucInterruptID, IntrControllerAddr);
#endif
	}
	configASSERT( lReturn );
}

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

void vInitialiseInterruptController( void )
{
    BaseType_t xStatus;
    XScuGic_Config *pxGICConfig;
    const BaseType_t xCoreID = portGET_CORE_ID();

    /* Initialize the spinlock */
    if ( Xil_IsSpinLockEnabled() != pdTRUE ){
	    portXIL_SPINLOCK_INIT();
        configASSERT( Xil_IsSpinLockEnabled() == pdTRUE );
    }
    
    /* Initialize the interrupt controller driver. */
	pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
	xStatus = XScuGic_CfgInitialize( &xInterruptControllers[ xCoreID ], pxGICConfig, pxGICConfig->CpuBaseAddress );	
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus;
}

/*-----------------------------------------------------------*/

void vPortInstallInterruptHandler( UBaseType_t uxInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef )
{
    BaseType_t xStatus;
    const BaseType_t xCoreID = portGET_CORE_ID();

    /* The IC instance needs to be initialized. */
    configASSERT( portCHECK_IF_IC_INITIALIZED( xCoreID ) );

    /* Connect the handler to the interrupt. */
    xStatus = XScuGic_Connect( &xInterruptControllers[ xCoreID ], uxInterruptID, pxHandler, pvCallBackRef );
	configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus;
}

/*-----------------------------------------------------------*/
void vPortConfigureInterrupt( UBaseType_t uxInterruptID, uint8_t ucInterruptPriority , uint8_t ucInterruptTrigger )
{
    const BaseType_t xCoreID = portGET_CORE_ID();
    
    /* The IC instance needs to be initialized. */
    configASSERT( portCHECK_IF_IC_INITIALIZED( xCoreID ) );

    /* Configure the interrupt. */
    XScuGic_SetPriorityTriggerType( &xInterruptControllers[ xCoreID ], uxInterruptID, ucInterruptPriority, ucInterruptTrigger );
}

/*-----------------------------------------------------------*/

void vPortEnableInterrupt( UBaseType_t uxInterruptID, BaseType_t xTargetCore )
{
    const BaseType_t xCoreID = portGET_CORE_ID();
	
    /* The IC instance needs to be initialized. */
    configASSERT( portCHECK_IF_IC_INITIALIZED( xCoreID ) );

    /* Shared Peripheral Interrupts (SPI). */
    if ( uxInterruptID >= XSCUGIC_SPI_INT_ID_START)
    {
        /* Set CpuId to the core the interrupt has to be enabled on
        (i.e. xTargetCore) before calling XScuGic_Enable. */    
        XScuGic_SetCpuID( xTargetCore );
    }
    else
    /* Private Peripheral Interrupts (PPI) and
    Software Generated Interrupts (SGI). */
    {
        /* PPIs and SGIs are private so they cannot be enabled
        on other cores. */
        configASSERT( xCoreID == xTargetCore );
    }

    XScuGic_Enable( &xInterruptControllers[ xCoreID ], uxInterruptID );
}
/*-----------------------------------------------------------*/

void vPortDisableInterrupt( UBaseType_t uxInterruptID, BaseType_t xTargetCore )
{
    const BaseType_t xCoreID = portGET_CORE_ID();

    /* The IC instance needs to be initialized. */
    configASSERT( portCHECK_IF_IC_INITIALIZED( xCoreID ) );

    /* Shared Peripheral Interrupts (SPI). */
    if ( uxInterruptID >= XSCUGIC_SPI_INT_ID_START)
    {
        /* Set CpuId to the core the interrupt has to be disabled on
        (i.e. xTargetCore) before calling XScuGic_Disable. */    
        XScuGic_SetCpuID( xTargetCore );
    }
    else
    /* Private Peripheral Interrupts (PPI) and
    Software Generated Interrupts (SGI). */
    {
        /* PPIs and SGIs are private so they cannot be disabled
        on other cores. */
        configASSERT( xCoreID == xTargetCore );
    }
    XScuGic_Disable( &xInterruptControllers[ xCoreID ], uxInterruptID );

}

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
uint32_t ulAPSR;

    #if( configASSERT_DEFINED == 1 )
    {
		volatile uint8_t ucOriginalPriority;
        volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + portINTERRUPT_PRIORITY_REGISTER_OFFSET );
        volatile uint8_t ucMaxPriorityValue;

        /* Determine how many priority bits are implemented in the GIC.

        Save the interrupt priority value that is about to be clobbered. */
		ucOriginalPriority = *pucFirstUserPriorityRegister;

        /* Determine the number of priority bits available.  First write to
        all possible bits. */
        *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

        /* Read the value back to see how many bits stuck. */
        ucMaxPriorityValue = *pucFirstUserPriorityRegister;

        /* Shift to the least significant bits. */
        while( ( ucMaxPriorityValue & portBIT_0_SET ) != portBIT_0_SET )
        {
            ucMaxPriorityValue >>= ( uint8_t ) 0x01;
        }

        /* Sanity check configUNIQUE_INTERRUPT_PRIORITIES matches the read
        value. */
        configASSERT( ucMaxPriorityValue == portLOWEST_INTERRUPT_PRIORITY );

        /* Restore the clobbered interrupt priority register to its original
        value. */
		*pucFirstUserPriorityRegister = ucOriginalPriority;
    }
    #endif /* configASSERT_DEFINED */


    /* Only continue if the CPU is not in User mode.  The CPU must be in a
    Privileged mode for the scheduler to start. */
    __asm volatile ( "MRS %0, APSR" : "=r" ( ulAPSR ) :: "memory" );
    ulAPSR &= portAPSR_MODE_BITS_MASK;
    configASSERT( ulAPSR != portAPSR_USER_MODE );

    if( ulAPSR != portAPSR_USER_MODE )
    {
        /* Only continue if the binary point value is set to its lowest possible
        setting.  See the comments in vPortValidateInterruptPriority() below for
        more information. */
        configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );

        if( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE )
        {
            /* Interrupts are turned off in the CPU itself to ensure tick does
            not execute while the scheduler is being started.  Interrupts are
            automatically turned back on in the CPU when the first task starts
            executing. */
            portCPU_IRQ_DISABLE();

            /* Start the timer that generates the tick ISR. */
            configSETUP_TICK_INTERRUPT();

#if ( configNUMBER_OF_CORES == 1 )            
            /* Start the first task executing. */
            vPortRestoreTaskContext();
#else
            /* Installs the portSGI_ID interrupt handler. */
            vPortSetupYieldRequestHandler( portCORE0 );

            /* Initialize the Xil_Spinlock API */
            if ( Xil_IsSpinLockEnabled() != pdTRUE ){
                portXIL_SPINLOCK_INIT();
                configASSERT( Xil_IsSpinLockEnabled() == pdTRUE );
            }

            /* Start the scheduler on all the available cores. */
            vPortLaunchSecondaryCore();
            configASSERT( ulSecondaryCoreAwake == pdTRUE );

            /* Start the first task executing. */
            vPortRestoreTaskContext();
#endif
        }
    }

    /* Will only get here if vTaskStartScheduler() was called with the CPU in
    a non-privileged mode or the binary point register was not set to its lowest
    possible value.  prvTaskExitError() is referenced to prevent a compiler
    warning about it being defined but not referenced in the case that the user
    defines their own exit address. */
    ( void ) prvTaskExitError;
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
    Artificially force an assert. */
    configASSERT( 0 );

    /* Only the core that calls vPortEndScheduler() is stopped
    when configNUMBER_OF_CORES > 1. */

}
/*-----------------------------------------------------------*/

/* When configNUMBER_OF_CORES > 1 the functions used to enter/exit
a critical section are vTaskEnterCritical() and vTaskExitCritical(). */
#if ( configNUMBER_OF_CORES == 1 )

    void vPortEnterCritical( void )
    {
        /* Mask interrupts up to the max syscall interrupt priority. */
        ulPortSetInterruptMask();

        /* Now interrupts are disabled ulCriticalNesting can be accessed
        directly.  Increment ulCriticalNesting to keep a count of how many times
        portENTER_CRITICAL() has been called. */
        ulCriticalNesting++;

        /* This is not the interrupt safe version of the enter critical function so
        assert() if it is being called from an interrupt context.  Only API
        functions that end in "FromISR" can be used in an interrupt.  Only assert if
        the critical nesting count is 1 to protect against recursive calls if the
        assert function also uses a critical section. */
        if( ulCriticalNesting == 1 )
        {
            configASSERT( ulPortInterruptNesting == 0 );
        }
    }

    /*-----------------------------------------------------------*/

    void vPortExitCritical( void )
    {
        if( ulCriticalNesting > portNO_CRITICAL_NESTING )
        {
            /* Decrement the nesting count as the critical section is being
            exited. */
            ulCriticalNesting--;

            /* If the nesting level has reached zero then all interrupt
            priorities must be re-enabled. */
            if( ulCriticalNesting == portNO_CRITICAL_NESTING )
            {
                /* Critical nesting has reached zero so all interrupt priorities
                should be unmasked. */
                portCLEAR_INTERRUPT_MASK();
            }
        }
    }

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES == 1 )
    void FreeRTOS_Tick_Handler( void )
    {

        /*
        * The Xilinx implementation of generating run time task stats uses the same timer used for generating
        * FreeRTOS ticks. In case user decides to generate run time stats the tick handler is called more
        * frequently (10 times faster). The timer/tick handler uses logic to handle the same. It handles
        * the FreeRTOS tick once per 10 interrupts.
        * For handling generation of run time stats, it increments a pre-defined counter every time the
        * interrupt handler executes.
        */
    #if (configGENERATE_RUN_TIME_STATS == 1)
        ulHighFrequencyTimerTicks++;
        if (!(ulHighFrequencyTimerTicks % 10))
    #endif
        {
        /* Set interrupt mask before altering scheduler structures.   The tick
        handler runs at the lowest priority, so interrupts cannot already be masked,
        so there is no need to save and restore the current mask value.  It is
        necessary to turn off interrupts in the CPU itself while the ICCPMR is being
        updated. */
        portCPU_IRQ_DISABLE();
        portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
        __asm volatile (    "dsb        \n"
                            "isb        \n" ::: "memory" );
        portCPU_IRQ_ENABLE();

        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            ulPortYieldRequired = pdTRUE;
        }
        }

        /* Ensure all interrupt priorities are active again. */
        portCLEAR_INTERRUPT_MASK();
        configCLEAR_TICK_INTERRUPT();
    }
#else /* #if ( configNUMBER_OF_CORES == 1 ) */
    void FreeRTOS_Tick_Handler( void )
    {
        /* Access to shared kernel data in interrupt handler need to be 
        performed in critical section due to multiple cores consideration. */
        uint32_t ulSavedInterruptStatus; 
        
        /*
        * The Xilinx implementation of generating run time task stats uses the same timer used for generating
        * FreeRTOS ticks. In case user decides to generate run time stats the tick handler is called more
        * frequently (10 times faster). The timer/tick handler uses logic to handle the same. It handles
        * the FreeRTOS tick once per 10 interrupts.
        * For handling generation of run time stats, it increments a pre-defined counter every time the
        * interrupt handler executes.
        */
    #if (configGENERATE_RUN_TIME_STATS == 1)
        ulHighFrequencyTimerTicks++;
        if (!(ulHighFrequencyTimerTicks % 10))
    #endif
        {
            /* Set interrupt mask before altering scheduler structures.   The tick
            handler runs at the lowest priority, so interrupts cannot already be masked,
            so there is no need to save and restore the current mask value.  It is
            necessary to turn off interrupts in the CPU itself while the ICCPMR is being
            updated. */
            portCPU_IRQ_DISABLE();
            portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
            __asm volatile (    "dsb        \n"
                                "isb        \n" ::: "memory" );
            portCPU_IRQ_ENABLE();

            /* Enter critical section */
            ulSavedInterruptStatus = portENTER_CRITICAL_FROM_ISR();

            /* Increment the RTOS tick. */
            if( xTaskIncrementTick() != pdFALSE )
            {
                /* The SysTick handler is installed on CORE0.
                Other cores are notified of a yield with inter-core
                signaling. */
                ulPortYieldRequired[ portCORE0 ] = pdTRUE;
            }

            portEXIT_CRITICAL_FROM_ISR(ulSavedInterruptStatus);
        }

        /* Ensure all interrupt priorities are active again. */
        portCLEAR_INTERRUPT_MASK(portUNMASK_VALUE);
        configCLEAR_TICK_INTERRUPT();
    }

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*-----------------------------------------------------------*/

#if( configUSE_TASK_FPU_SUPPORT != 2 )

    void vPortTaskUsesFPU( void )
    {
    uint32_t ulInitialFPSCR = 0;

        /* A task is registering the fact that it needs an FPU context.  Set the
        FPU flag (which is saved as part of the task context). */
    #if ( configNUMBER_OF_CORES == 1 )
        ulPortTaskHasFPUContext = pdTRUE;
    #else
        ulPortTaskHasFPUContext[portGET_CORE_ID()] = pdTRUE;
    #endif

        /* Initialise the floating point status register. */
        __asm volatile ( "FMXR  FPSCR, %0" :: "r" (ulInitialFPSCR) : "memory" );
    }

#endif /* configUSE_TASK_FPU_SUPPORT */
/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES == 1 ) 

    void vPortClearInterruptMask( uint32_t ulNewMaskValue )
    {
        if( ulNewMaskValue == pdFALSE )
        {
            // Interrupts are disabled before the ICCPMR is updated
            portCLEAR_INTERRUPT_MASK();
        }
    }

    /*-----------------------------------------------------------*/


    uint32_t ulPortSetInterruptMask( void )
    {
    uint32_t ulReturn;

        // Interrupt in the CPU must be turned off while the ICCPMR is being updated.
        portCPU_IRQ_DISABLE();
        if( portICCPMR_PRIORITY_MASK_REGISTER == ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) )
        {
            // Interrupts were already masked.
            ulReturn = pdTRUE;
        }
        else
        {
            ulReturn = pdFALSE;
            portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
            __asm volatile (    "dsb        \n"
                                "isb        \n" ::: "memory" );
        }
        portCPU_IRQ_ENABLE();

        return ulReturn;
    }

#else /* #if ( configNUMBER_OF_CORES == 1 ) */

    void vPortClearInterruptMask( uint32_t ulSavedInterruptState )
    {
        const uint32_t ulInterruptsEnabled = !portCHECK_IF_INTERRUPTS_DISABLED();

        /* Interrupts are disabled before the ICCPMR is updated */
        if( ulInterruptsEnabled )
        {
            portCPU_IRQ_DISABLE();

            portICCPMR_PRIORITY_MASK_REGISTER = ulSavedInterruptState;
            __asm volatile (    "DSB        \n"        
                                "ISB        \n" ::: "memory" );      
        
            portCPU_IRQ_ENABLE();
        }
        else
        {
            portICCPMR_PRIORITY_MASK_REGISTER = ulSavedInterruptState;
            __asm volatile (    "DSB        \n"        
                                "ISB        \n" ::: "memory" );      
        }
    }

    /*-----------------------------------------------------------*/

    uint32_t ulPortSetInterruptMask( void )
    {
        uint32_t ulSavedInterruptState;
        const uint32_t ulInterruptsEnabled = !portCHECK_IF_INTERRUPTS_DISABLED();
        
        /* Interrupts are disabled before the ICCPMR is updated */
        if( ulInterruptsEnabled )
        {
            portCPU_IRQ_DISABLE();

            ulSavedInterruptState = portICCPMR_PRIORITY_MASK_REGISTER;
            portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
            __asm volatile (    "DSB        \n"               
                                "ISB        \n" ::: "memory" );
        
            portCPU_IRQ_ENABLE();
        }
        else
        {
            ulSavedInterruptState = portICCPMR_PRIORITY_MASK_REGISTER;
            portICCPMR_PRIORITY_MASK_REGISTER = ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT );
            __asm volatile (    "DSB        \n"               
                                "ISB        \n" ::: "memory" );
        }

        return ulSavedInterruptState;        
    }

#endif /* #if ( configNUMBER_OF_CORES == 1 ) */

/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

    void vPortValidateInterruptPriority( void )
    {
        /* The following assertion will fail if a service routine (ISR) for
        an interrupt that has been assigned a priority above
        configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
        function.  ISR safe FreeRTOS API functions must *only* be called
        from interrupts that have been assigned a priority at or below
        configMAX_SYSCALL_INTERRUPT_PRIORITY.

        Numerically low interrupt priority numbers represent logically high
        interrupt priorities, therefore the priority of the interrupt must
        be set to a value equal to or numerically *higher* than
        configMAX_SYSCALL_INTERRUPT_PRIORITY.

        FreeRTOS maintains separate thread and ISR API functions to ensure
        interrupt entry is as fast and simple as possible. */
        configASSERT( portICCRPR_RUNNING_PRIORITY_REGISTER >= ( uint32_t ) ( configMAX_API_CALL_INTERRUPT_PRIORITY << portPRIORITY_SHIFT ) );

        /* Priority grouping:  The interrupt controller (GIC) allows the bits
        that define each interrupt's priority to be split between bits that
        define the interrupt's pre-emption priority bits and bits that define
        the interrupt's sub-priority.  For simplicity all bits must be defined
        to be pre-emption priority bits.  The following assertion will fail if
        this is not the case (if some bits represent a sub-priority).

        The priority grouping is configured by the GIC's binary point register
        (ICCBPR).  Writting 0 to ICCBPR will ensure it is set to its lowest
        possible value (which may be above 0). */
        configASSERT( ( portICCBPR_BINARY_POINT_REGISTER & portBINARY_POINT_BITS ) <= portMAX_BINARY_POINT_VALUE );
    }

#endif /* configASSERT_DEFINED */
/*-----------------------------------------------------------*/

void vApplicationFPUSafeIRQHandler( uint32_t ulICCIAR )
{
    ( void ) ulICCIAR;
    configASSERT( ( volatile void * ) NULL );
}

#if( configGENERATE_RUN_TIME_STATS == 1 )
    /*
    * For Xilinx implementation this is a dummy function that does a redundant operation
    * of zeroing out the global counter.
    * It is called by FreeRTOS kernel.
    */
    void xCONFIGURE_TIMER_FOR_RUN_TIME_STATS (void)
    {
        ulHighFrequencyTimerTicks = 0;
    }
    /*
    * For Xilinx implementation this function returns the global counter used for
    * run time task stats calculation.
    * It is called by FreeRTOS kernel task handling logic.
    */
    uint32_t xGET_RUN_TIME_COUNTER_VALUE (void)
    {
        return ulHighFrequencyTimerTicks;
}
#endif
