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


#ifndef PORTMACRO_H
#define PORTMACRO_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#define portSTACK_TYPE           uint32_t
#define portBASE_TYPE            int32_t
#define portUBASE_TYPE           uint32_t
#define portMAX_DELAY            ( TickType_t ) 0xffffffffUL

typedef portSTACK_TYPE   StackType_t;
typedef portBASE_TYPE    BaseType_t;
typedef portUBASE_TYPE   UBaseType_t;
typedef portUBASE_TYPE   TickType_t;

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC    1
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH          ( -1 )
#define portTICK_PERIOD_MS        ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT    16
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
extern void vTaskSwitchContext( void );
#define portYIELD()                (*((volatile unsigned long *)configCLINT_BASE_ADDRESS) = 1UL);
#define portEND_SWITCHING_ISR( xSwitchRequired ) \
    do                                           \
    {                                            \
        if( xSwitchRequired != pdFALSE )         \
        {                                        \
            traceISR_EXIT_TO_SCHEDULER();        \
            vTaskSwitchContext();                \
        }                                        \
        else                                     \
        {                                        \
            traceISR_EXIT();                     \
        }                                        \
    } while( 0 )
#define portYIELD_FROM_ISR( x )    portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Critical section management. */
#define portCRITICAL_NESTING_IN_TCB    0

#define portDISABLE_INTERRUPTS()                                   __asm volatile ( "csrc mstatus, 8" )
#define portENABLE_INTERRUPTS()                                    __asm volatile ( "csrs mstatus, 8" )

extern size_t xCriticalNesting;
#define portENTER_CRITICAL()      \
    {                             \
        portDISABLE_INTERRUPTS(); \
        xCriticalNesting++;       \
    }

#define portEXIT_CRITICAL()          \
    {                                \
        xCriticalNesting--;          \
        if( xCriticalNesting == 0 )  \
        {                            \
            portENABLE_INTERRUPTS(); \
        }                            \
    }

/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
#endif

#if ( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )

/* Check the configuration. */
    #if ( configMAX_PRIORITIES > 32 )
        #error "configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice."
    #endif

/* Store/clear the ready priorities in a bit map. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    uxTopPriority = ( 31UL - __builtin_clz( uxReadyPriorities ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */


/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. These are
 * not necessary for to use this port.  They are defined so the common demo
 * files (which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )

/*-----------------------------------------------------------*/

#define portNOP()    __asm volatile ( " nop " )
#define portINLINE              __inline

#ifndef portFORCE_INLINE
    #define portFORCE_INLINE    inline __attribute__( ( always_inline ) )
#endif

#define portMEMORY_BARRIER()    __asm volatile ( "" ::: "memory" )
/*-----------------------------------------------------------*/

#if !defined( configMTIME_BASE_ADDRESS ) || !defined( configMTIMECMP_BASE_ADDRESS )
    #error "configMTIME_BASE_ADDRESS and configMTIMECMP_BASE_ADDRESS must be defined in FreeRTOSConfig.h.  Set them to zero if there is no MTIME (machine time) clock.  See www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html"
#endif /* if !defined( configMTIME_BASE_ADDRESS ) || ( configMTIMECMP_BASE_ADDRESS == 0 ) */

#if !defined( configCLINT_BASE_ADDRESS )
    #error "configCLINT_BASE_ADDRESS must be defined in FreeRTOSConfig.h."
#endif /* if !defined( configCLINT_BASE_ADDRESS == 0 ) */
/*-----------------------------------------------------------*/

/* MPU v2 */
#ifdef configENABLE_MPU
    #if ( configUSE_MPU_WRAPPERS_V1 == 1 )
        #error "This port only support mpu_wrappers_v2"
    #endif

    #ifndef configSYSTEM_CALL_STACK_SIZE
        #error "configSYSTEM_CALL_STACK_SIZE must be defined to the desired size of the system call stack in words for using MPU wrappers v2."
    #endif

    #define portUSING_MPU_WRAPPERS 1
    #define portHAS_STACK_OVERFLOW_CHECKING 1

    #define portPRIVILEGE_BIT ( 1UL << 31 )

    #define portIS_TASK_PRIVILEGED() pdTRUE

    #define portPMP_MAX_ENTRY           ( 16UL )
    #define portPMP_ENTRY_BIT           ( 8UL )
    #define portPMP_ENTRY_PER_CONFIG    ( 4UL )
    #define portPMP_NUM_CONFIG          ( portPMP_MAX_ENTRY / portPMP_ENTRY_PER_CONFIG )
    #define portPMP_GRANULARITY_BYTES   ( 8UL ) // FIXME: only supprot 8-byte alignment for simplicity
    #define portPMP_R                   ( 1UL << 0 )
    #define portPMP_W                   ( 1UL << 1 )
    #define portPMP_X                   ( 1UL << 2 )
    #define portPMP_A_NAPOT             ( 3UL << 3 )

    #define portNUM_CONFIGURABLE_REGIONS ( 8 ) // FIXME: only allow 8 for now

    /* FIXME: Different from ARM ports */
    #define portMPU_REGION_READ                                 ( 1UL << 0UL )
    #define portMPU_REGION_WRITE                                ( 1UL << 1UL )
    #define portMPU_REGION_EXECUTE                              ( 1UL << 2UL )

    /* Size of an Access Control List (ACL) entry in bits. */
    #define portACL_ENTRY_SIZE_BITS             ( 32U )

    typedef struct KERNEL_STACK_INFO
    {
        StackType_t * pxTopOfKernelStack; /* This *MUST* be the first member of xMPU_SETTINGS */
        StackType_t * pxKernelStackHigh;
        StackType_t * pxKernelStackLow;
    } xKERNEL_STACK_INFO;

    typedef struct PMP_SETTINGS
    {
        uint32_t pmpcfg[ portPMP_NUM_CONFIG ];
        uint32_t pmpaddr[ portPMP_MAX_ENTRY ];
        uint32_t pmp_cfg_used;
    } xPMP_SETTINGS;

    typedef struct MPU_SETTINGS
    {
        xKERNEL_STACK_INFO xKernelStackInfo;
        BaseType_t xTaskIsPrivileged;
        xPMP_SETTINGS xPMPSettings;
        uint32_t ulAccessControlList[ ( configPROTECTED_KERNEL_OBJECT_POOL_SIZE / portACL_ENTRY_SIZE_BITS ) + 1 ];
    } xMPU_SETTINGS;
#endif /* ifdef configENABLE_MPU */

/*-----------------------------------------------------------*/

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
