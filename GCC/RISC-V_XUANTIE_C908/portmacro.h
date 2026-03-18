 /*
 * Copyright (C) 2017-2024 Alibaba Group Holding Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H

#include <stdlib.h>
#include <stdint.h>
#include <drv/tick.h>
#include <drv/irq.h>
#include <riscv_csr.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define portCHAR                char
#define portFLOAT               float
#define portDOUBLE              double
#define portLONG                long
#define portSHORT               short
#define portSTACK_TYPE          uint64_t
#define portBASE_TYPE           int64_t
#define portUBASE_TYPE          uint64_t
#define portPOINTER_SIZE_TYPE   uint64_t

typedef portSTACK_TYPE          StackType_t;
typedef portBASE_TYPE           BaseType_t;
typedef portUBASE_TYPE          UBaseType_t;
typedef portUBASE_TYPE          TickType_t;

#define portMAX_DELAY           (0xFFFFFFFFFFFFFFFFUL)

/**
 *32-bit tick type on a 32-bit architecture
 *so reads of the tick count do
 *not need to be guarded with a critical section.
 **/
#define portTICK_TYPE_IS_ATOMIC 1

/**
 * Architecture specific macros.
 * */
#define portSTACK_GROWTH            (-1)
#define portTICK_PERIOD_MS          (( TickType_t ) 1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT          (16)

static inline portLONG SaveLocalPSR (void)
{
    portLONG flags = csi_irq_save();
    return flags;
}

static inline void RestoreLocalPSR (portLONG newMask)
{
    csi_irq_restore(newMask);
}

#if CONFIG_RISCV_FPU_ENABLED
    #define portTASK_USES_FLOATING_POINT()    pdTRUE
#else
    #define portTASK_USES_FLOATING_POINT()    pdFALSE
#endif

#if configNUMBER_OF_CORES > 1
    #define portGET_CORE_ID()                       csi_get_cpu_id()
    #define __FENCE(p, s) __ASM volatile ("fence " #p "," #s : : : "memory")
    #define mb()        __FENCE(iorw,iorw)

    #define portRTOS_SPINLOCK_COUNT                 (2)
    typedef volatile uint32_t spin_lock_t;
    extern spin_lock_t hw_sync_locks[portRTOS_SPINLOCK_COUNT];
    void SecondaryCoresUp(void);
    void vPortRecursiveLock(unsigned long ulCoreID, unsigned long ulLockNum, spin_lock_t *pxSpinLock, BaseType_t uxAcquire);
    void clear_software_irq(int xCoreID);
    extern volatile UBaseType_t uxYieldCoreAgain[configNUMBER_OF_CORES];

    extern UBaseType_t uxCriticalNestings[ configNUMBER_OF_CORES ];
    #define portGET_CRITICAL_NESTING_COUNT(xCoreID)            ( uxCriticalNestings[xCoreID] )
    #define portSET_CRITICAL_NESTING_COUNT(xCoreID, xCount)    ( uxCriticalNestings[xCoreID] = (xCount) )
    #define portINCREMENT_CRITICAL_NESTING_COUNT(xCoreID)      ( uxCriticalNestings[xCoreID]++ )
    #define portDECREMENT_CRITICAL_NESTING_COUNT(xCoreID)      ( uxCriticalNestings[xCoreID]-- )

    unsigned long cpu_intrpt_save();
    void cpu_intrpt_restore(unsigned long ulstate);

    #define portDISABLE_INTERRUPTS()                __ASM volatile("csrc mstatus, 8")
    #define portENABLE_INTERRUPTS()                 __ASM volatile("csrs mstatus, 8")
    #define portSET_INTERRUPT_MASK_FROM_ISR()       cpu_intrpt_save();
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)    cpu_intrpt_restore(x)
    #define portSET_INTERRUPT_MASK()                cpu_intrpt_save()
    #define portCLEAR_INTERRUPT_MASK(a)             cpu_intrpt_restore(a)
    #define portENTER_CRITICAL_FROM_ISR()           vTaskEnterCriticalFromISR()
    #define portEXIT_CRITICAL_FROM_ISR(a)           vTaskExitCriticalFromISR(a)

    #define portGET_ISR_LOCK(xCoreID)               vPortRecursiveLock(xCoreID, 0, &hw_sync_locks[0], pdTRUE)
    #define portRELEASE_ISR_LOCK(xCoreID)           vPortRecursiveLock(xCoreID, 0, &hw_sync_locks[0], pdFALSE)
    #define portGET_TASK_LOCK(xCoreID)              vPortRecursiveLock(xCoreID, 1, &hw_sync_locks[1], pdTRUE)
    #define portRELEASE_TASK_LOCK(xCoreID)          vPortRecursiveLock(xCoreID, 1, &hw_sync_locks[1], pdFALSE)
    void vPortYield_Core(int xCoreID);
    #define portYIELD_CORE(a)                       vPortYield_Core(a)
    #define portENTER_CRITICAL()                    vTaskEnterCritical()
    #define portEXIT_CRITICAL()                     vTaskExitCritical()
#else  /* configNUMBER_OF_CORES > 1 */
    void vPortEnterCritical(void);
    void vPortExitCritical(void);
    void clear_software_irq(int xCoreID);
    void vPortYield_Core(int xCoreID);
    void vPortEnableInterrupt(void);
    void vPortDisableInterrupt(void);
    #define portDISABLE_INTERRUPTS()                vPortDisableInterrupt()
    #define portENABLE_INTERRUPTS()                 vPortEnableInterrupt()
    #define portSET_INTERRUPT_MASK_FROM_ISR()       SaveLocalPSR()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR(a)    RestoreLocalPSR(a)
    #define portENTER_CRITICAL()                    vPortEnterCritical()
    #define portEXIT_CRITICAL()                     vPortExitCritical()
#endif /* configNUMBER_OF_CORES > 1 */

#if configGENERATE_RUN_TIME_STATS
    #define portGET_RUN_TIME_COUNTER_VALUE()        csi_tick_get_ms()
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#endif

#define portNOP() __asm volatile                    (" nop ")

extern portLONG ulCriticalNesting;
extern portLONG pendsvflag;

/* Scheduler utilities. */
extern void vPortYield( void );
#define portYIELD()                 vPortYield();

/* Added as there is no such function in FreeRTOS. */
extern void *pvPortRealloc( uint8_t *srcaddr,size_t xWantedSize );

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters ) __attribute__((noreturn))
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#define portEND_SWITCHING_ISR( xSwitchRequired )    do {    \
                                                            if( xSwitchRequired != pdFALSE )    \
                                                            {   \
                                                                portYIELD();    \
                                                            }   \
                                                    } while(0)

#define portYIELD_FROM_ISR( a )     portEND_SWITCHING_ISR( a )

#define portINLINE	static inline

#ifndef portFORCE_INLINE
    #define portFORCE_INLINE static inline __attribute__(( always_inline))
#endif

#ifdef __cplusplus
    }
#endif

#endif /* PORTMACRO_H */
