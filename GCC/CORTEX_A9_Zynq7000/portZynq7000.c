/*
 * FreeRTOS Kernel V10.6.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (C) 2009-2021 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Xilinx includes. */
#include "xscugic.h"
#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
#include "xscutimer.h"
#else
#include "xiltimer.h"
#endif


#include "xil_exception.h"

#define XSCUTIMER_CLOCK_HZ	configCPU_CLOCK_HZ

/*
 * Some FreeRTOSConfig.h settings require the application writer to provide the
 * implementation of a callback function that has a specific name, and a linker
 * error will result if the application does not provide the required function.
 * To avoid the risk of a configuration file setting resulting in a linker error
 * this file provides default implementations of each callback that might be
 * required.  The default implementations are declared as weak symbols to allow
 * the application writer to override the default implementation by providing
 * their own implementation in the application itself.
 */
void vApplicationAssert( const char *pcFileName, uint32_t ulLine ) __attribute__((weak));
void vApplicationTickHook( void ) __attribute__((weak));
void vApplicationIdleHook( void ) __attribute__((weak));
void vApplicationMallocFailedHook( void ) __attribute((weak));
void vApplicationDaemonTaskStartupHook( void ) __attribute__((weak));
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) __attribute__((weak));

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
								__attribute__((weak));

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
								__attribute__((weak));
#endif

/* Port functions for SysTick management */
void FreeRTOS_ClearTickInterrupt( void );
void FreeRTOS_SetupTickInterrupt( void );

#if ( configNUMBER_OF_CORES == 1 )
	#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
		/* Timer used to generate the tick interrupt. */
		static XScuTimer xTimer;
		/* Interrupt controller instance */
		extern XScuGic xInterruptController; 	
	#else
		void TimerCounterHandler(void *CallBackRef, u32 TmrCtrNumber);

		extern uintptr_t IntrControllerAddr;
	#endif
#else
	/* Timer used to generate the tick interrupt. */
	static XScuTimer xTimer; 
	/* Interrupt controller instance */
	extern XScuGic xInterruptControllers[configNUMBER_OF_CORES];

	/* GIC management on multiple cores. */

	/* Initialize the interrupt controller instance for secondary cores. */
	void vPortInitCoreInterruptController( void );
	/* Install the interrupt handler for the SGI triggered by portYIELD_CORE(x). */
	void vPortSetupYieldRequestHandler( BaseType_t xCoreID );
	/* Handles the SGI generated with portYIELD_CORE(x). */
	void prvYieldCoreHandler( void );
#endif

/*-----------------------------------------------------------*/

#if !defined(XPAR_XILTIMER_ENABLED) && !defined(SDT)
	void FreeRTOS_SetupTickInterrupt( void )
	{
		BaseType_t xStatus;
		extern void FreeRTOS_Tick_Handler( void );
		XScuTimer_Config *pxTimerConfig;
		XScuGic_Config *pxGICConfig;
		const uint8_t ucRisingEdge = 3;
		XScuGic* pxInterruptController;

		#if ( configNUMBER_OF_CORES == 1 )
			pxInterruptController = &xInterruptController;
		#else
			/* The SysTick handler must be installed on core 0. */
			configASSERT( portGET_CORE_ID() == portCORE0 );
			
			pxInterruptController = &xInterruptControllers[ portCORE0 ]; 
		#endif

		/* This function is called with the IRQ interrupt disabled, and the IRQ
		interrupt should be left disabled.  It is enabled automatically when the
		scheduler is started. */

		/* Ensure XScuGic_CfgInitialize() has been called.  In this demo it has
		already been called from prvSetupHardware() in main(). */
		pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
		xStatus = XScuGic_CfgInitialize( pxInterruptController, pxGICConfig, pxGICConfig->CpuBaseAddress );
		configASSERT( xStatus == XST_SUCCESS );
		( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

		/* The priority must be the lowest possible. */
		XScuGic_SetPriorityTriggerType( pxInterruptController, XPAR_SCUTIMER_INTR, portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT, ucRisingEdge );

		/* Install the FreeRTOS tick handler. */
		xStatus = XScuGic_Connect( pxInterruptController, XPAR_SCUTIMER_INTR, (Xil_ExceptionHandler) FreeRTOS_Tick_Handler, ( void * ) &xTimer );
		configASSERT( xStatus == XST_SUCCESS );
		( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

		/* Initialise the timer. */
		pxTimerConfig = XScuTimer_LookupConfig( XPAR_SCUTIMER_DEVICE_ID );
		xStatus = XScuTimer_CfgInitialize( &xTimer, pxTimerConfig, pxTimerConfig->BaseAddr );
		configASSERT( xStatus == XST_SUCCESS );
		( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

		/* Enable Auto reload mode. */
		XScuTimer_EnableAutoReload( &xTimer );

		/* Ensure there is no prescale. */
		XScuTimer_SetPrescaler( &xTimer, 0 );

		/* Load the timer counter register.
		* The Xilinx implementation of generating run time task stats uses the same timer used for generating
		* FreeRTOS ticks. In case user decides to generate run time stats the timer time out interval is changed
		* as "configured tick rate * 10". The multiplying factor of 10 is hard coded for Xilinx FreeRTOS ports.
		*/
	#if (configGENERATE_RUN_TIME_STATS == 1)
		XScuTimer_LoadTimer( &xTimer, XSCUTIMER_CLOCK_HZ / (configTICK_RATE_HZ * 10) );
	#else
		XScuTimer_LoadTimer( &xTimer, XSCUTIMER_CLOCK_HZ / configTICK_RATE_HZ );
	#endif

		/* Start the timer counter and then wait for it to timeout a number of
		times. */
		XScuTimer_Start( &xTimer );

		/* Enable the interrupt for the xTimer in the interrupt controller. */
		XScuGic_Enable( pxInterruptController, XPAR_SCUTIMER_INTR );

		/* Enable the interrupt in the xTimer itself. */
		FreeRTOS_ClearTickInterrupt();
		XScuTimer_EnableInterrupt( &xTimer );
	}
	

	#if ( configNUMBER_OF_CORES > 1 )
		void vPortInitCoreInterruptController( void ){
			BaseType_t xStatus;
			XScuGic_Config *pxGICConfig;
			
			/* Only called from CORE1 in the port for Zynq7000. */
			configASSERT( portGET_CORE_ID() == portCORE1 );

			/* This function is called with the IRQ interrupt disabled, and the IRQ
			interrupt should be left disabled. It is enabled automatically when the
			first task is executed. 
			FIQ are not masked by default. */

			pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );
			xStatus = XScuGic_CfgInitialize( &xInterruptControllers[ portCORE1 ], pxGICConfig, pxGICConfig->CpuBaseAddress );
			configASSERT( xStatus == XST_SUCCESS );
			( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

			/* All core should be able to handle a portYIELD_CORE(x) from another core. */
			vPortSetupYieldRequestHandler( portCORE1 );
		}

		void vPortSetupYieldRequestHandler( BaseType_t xCoreID ){
			BaseType_t xStatus;
			const uint8_t ucRisingEdge = 3;
			
			/* Configure the Software Generated Interrupt for inter-core signaling. 
			The priority is the lowest possible.
			SGIs are always triggered on the rising-edge. */
			XScuGic_SetPriorityTriggerType( &xInterruptControllers[ xCoreID ], portSGI_ID, portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT, ucRisingEdge );
			
			xStatus = XScuGic_Connect( &xInterruptControllers[ xCoreID ], portSGI_ID, (Xil_ExceptionHandler) prvYieldCoreHandler, NULL );
			configASSERT( xStatus == XST_SUCCESS );
			( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */
		
			XScuGic_Enable( &xInterruptControllers[ xCoreID ], portSGI_ID );
		}

		void prvYieldCoreHandler( void )
		{
			extern volatile uint32_t ulPortYieldRequired[ configNUMBER_OF_CORES ];
			/* The core needs to yield the current task. */
			ulPortYieldRequired[ portGET_CORE_ID() ] = pdTRUE;
		}
		
	#endif /* #if ( configNUMBER_OF_CORES > 1 ) */

/*-----------------------------------------------------------*/

void FreeRTOS_ClearTickInterrupt( void )
{
	XScuTimer_ClearInterruptStatus( &xTimer );
}

#else
void TimerCounterHandler(void *CallBackRef, u32 TmrCtrNumber)
{
	(void) CallBackRef;
	(void) TmrCtrNumber;
        FreeRTOS_Tick_Handler();
}

void FreeRTOS_SetupTickInterrupt( void )
{
	/* Limit the configTICK_RATE_HZ to 1000 if user configured greater than 1000 */
	uint32_t Tick_Rate = (configTICK_RATE_HZ > 1000) ? 1000 : configTICK_RATE_HZ;

	/*
	 * The Xilinx implementation of generating run time task stats uses the same timer used for generating
	 * FreeRTOS ticks. In case user decides to generate run time stats the timer time out interval is changed
	 * as "configured tick rate * 10". The multiplying factor of 10 is hard coded for Xilinx FreeRTOS ports.
	 */
#if (configGENERATE_RUN_TIME_STATS == 1)
	/* XTimer_SetInterval() API expects delay in milli seconds
         * Convert the user provided tick rate to milli seconds.
         */
	XTimer_SetInterval(XTIMER_DELAY_MSEC/(Tick_Rate * 10));
#else
	/* XTimer_SetInterval() API expects delay in milli seconds
         * Convert the user provided tick rate to milli seconds.
         */
	XTimer_SetInterval(XTIMER_DELAY_MSEC/Tick_Rate);
#endif
	XTimer_SetHandler(TimerCounterHandler, 0,
			portLOWEST_USABLE_INTERRUPT_PRIORITY << portPRIORITY_SHIFT);
}

void FreeRTOS_ClearTickInterrupt( void )
{
	XTimer_ClearTickInterrupt();
}
#endif

/*-----------------------------------------------------------*/

void vApplicationIRQHandler( uint32_t ulICCIAR )
{
	extern XScuGic_Config XScuGic_ConfigTable[];
	static const XScuGic_VectorTableEntry *pxVectorTable = XScuGic_ConfigTable[ XPAR_SCUGIC_SINGLE_DEVICE_ID ].HandlerTable;
	uint32_t ulInterruptID;
	const XScuGic_VectorTableEntry *pxVectorEntry;

	/* The ID of the interrupt is obtained by bitwise anding the ICCIAR value
	with 0x3FF. */
	ulInterruptID = ulICCIAR & 0x3FFUL;
	if( ulInterruptID < XSCUGIC_MAX_NUM_INTR_INPUTS )
	{
		/* Call the function installed in the array of installed handler functions. */
		pxVectorEntry = &( pxVectorTable[ ulInterruptID ] );
		pxVectorEntry->Handler( pxVectorEntry->CallBackRef );
	}
}
/*-----------------------------------------------------------*/

/* This version of vApplicationAssert() is declared as a weak symbol to allow it
to be overridden by a version implemented within the application that is using
this BSP. */
#if ( configNUMBER_OF_CORES == 1 )
void vApplicationAssert( const char *pcFileName, uint32_t ulLine )
{
	volatile uint32_t ul = 0;
	volatile const char *pcLocalFileName = pcFileName; /* To prevent pcFileName being optimized away. */
	volatile uint32_t ulLocalLine = ulLine; /* To prevent ulLine being optimized away. */

	/* Prevent compile warnings about the following two variables being set but
	not referenced.  They are intended for viewing in the debugger. */
	( void ) pcLocalFileName;
	( void ) ulLocalLine;

	xil_printf( "Assert failed in file %s, line %u\r\n", pcLocalFileName, ulLocalLine );

	/* If this function is entered then a call to configASSERT() failed in the
	FreeRTOS code because of a fatal error.  The pcFileName and ulLine
	parameters hold the file name and line number in that file of the assert
	that failed.  Additionally, if using the debugger, the function call stack
	can be viewed to find which line failed its configASSERT() test.  Finally,
	the debugger can be used to set ul to a non-zero value, then step out of
	this function to find where the assert function was entered. */
	
	if( portCHECK_IF_IN_ISR() == pdTRUE )
	{
		taskENTER_CRITICAL_FROM_ISR();
		{
			while( ul == 0 )
			{
				__asm volatile( "NOP" );
			}
		}
		taskEXIT_CRITICAL_FROM_ISR( pdFALSE );
	}
	else
	{
		taskENTER_CRITICAL();
		{
			while( ul == 0 )
			{
				__asm volatile( "NOP" );
			}
		}
		taskEXIT_CRITICAL();
	}
}
#else /* #if ( configNUMBER_OF_CORES == 1 ) */
void vApplicationAssert( const char *pcFileName, uint32_t ulLine )
{
	volatile uint32_t ul = 0;
	volatile const char *pcLocalFileName = pcFileName; /* To prevent pcFileName being optimized away. */
	volatile uint32_t ulLocalLine = ulLine; /* To prevent ulLine being optimized away. */
	volatile BaseType_t xCoreID = portGET_CORE_ID();

	/* Prevent compile warnings about the following two variables being set but
	not referenced.  They are intended for viewing in the debugger. */
	( void ) pcLocalFileName;
	( void ) ulLocalLine;
	( void ) xCoreID;	

	xil_printf( "[CORE #%d] Assert failed in file %s, line %u\r\n", xCoreID, pcLocalFileName, ulLocalLine );

	/* If this function is entered then a call to configASSERT() failed in the
	FreeRTOS code because of a fatal error.  The pcFileName and ulLine
	parameters hold the file name and line number in that file of the assert
	that failed.  Additionally, if using the debugger, the function call stack
	can be viewed to find which line failed its configASSERT() test.  Finally,
	the debugger can be used to set ul to a non-zero value, then step out of
	this function to find where the assert function was entered. */
	
	if( portCHECK_IF_IN_ISR() != pdFALSE )
	{
		UBaseType_t uxSavedInterruptStatus;
		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
		{
			while( ul == 0 )
			{
				__asm volatile( "NOP" );
			}
		}
		taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
	}
	else
	{
		taskENTER_CRITICAL();
		{
			while( ul == 0 )
			{
				__asm volatile( "NOP" );
			}
		}
		taskEXIT_CRITICAL();
	}
}
#endif /* #if ( configNUMBER_OF_CORES == 1 ) */
/*-----------------------------------------------------------*/

/* This default tick hook does nothing and is declared as a weak symbol to allow
the application writer to override this default by providing their own
implementation in the application code. */
void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/

/* This default idle hook does nothing and is declared as a weak symbol to allow
the application writer to override this default by providing their own
implementation in the application code. */
void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

/* This default malloc failed hook does nothing and is declared as a weak symbol
to allow the application writer to override this default by providing their own
implementation in the application code. */
void vApplicationMallocFailedHook( void )
{
}
/*-----------------------------------------------------------*/

/* This default daemon task startup hook does nothing and is declared as a weak
symbol to allow the application writer to override this default by providing
their own implementation in the application code. */
void vApplicationDaemonTaskStartupHook( void )
{	
}
/*-----------------------------------------------------------*/

/* This default stack overflow hook will stop the application from executing.
It is declared as a weak symbol to allow the application writer to override
this default by providing their own implementation in the application code. */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
/* Attempt to prevent the handle and name of the task that overflowed its stack
from being optimised away because they are not used. */
volatile TaskHandle_t xOverflowingTaskHandle = xTask;
volatile char *pcOverflowingTaskName = pcTaskName;

	( void ) xOverflowingTaskHandle;
	( void ) pcOverflowingTaskName;

	xil_printf( "HALT: Task %s overflowed its stack.", pcOverflowingTaskName );
	portDISABLE_INTERRUPTS();
	for( ;; );
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
/* Buffers below are used for static memory allocation for idle
 * task. */
static StaticTask_t xIdleTaskTCB;
static StackType_t  xIdleTaskStack[ configMINIMAL_STACK_SIZE ];
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = xIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------*/
/* Buffers below are used for static memory allocation for timer
 * task. */
static StaticTask_t xTimerTaskTCB;
static StackType_t  xTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
	/* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = xTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#if ( configNUMBER_OF_CORES > 1 )
/* Buffers below are used for static memory allocation for passive idle
 * task. */
static StaticTask_t xIdleTaskTCBs[ configNUMBER_OF_CORES - 1 ];
static StackType_t uxIdleTaskStacks[ configNUMBER_OF_CORES - 1 ][ configMINIMAL_STACK_SIZE ];

void vApplicationGetPassiveIdleTaskMemory(  StaticTask_t ** ppxIdleTaskTCBBuffer,
                                            StackType_t ** ppxIdleTaskStackBuffer,
                                            configSTACK_DEPTH_TYPE * puxIdleTaskStackSize,
                                            BaseType_t xPassiveIdleTaskIndex ){
	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &( xIdleTaskTCBs[ xPassiveIdleTaskIndex ] );

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = &( uxIdleTaskStacks[ xPassiveIdleTaskIndex ][ 0 ] );

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#endif /* ( configNUMBER_OF_CORES > 1 ) */

#endif /* #if (configSUPPORT_STATIC_ALLOCATION == 1) */
