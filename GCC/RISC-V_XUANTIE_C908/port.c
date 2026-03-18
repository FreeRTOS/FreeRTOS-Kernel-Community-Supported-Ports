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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "string.h"

typedef enum {
    CORE_0 = 0,
    CORE_1,
    CORE_2,
    CORE_3,
    CORE_4,
    CORE_5,
    CORE_6,
    CORE_7
} CoreID;

#define printk printf

extern void vPortStartTask(void);

#if configNUMBER_OF_CORES > 1

    void atomic_store(spin_lock_t *ptr, spin_lock_t val)
    {
        spin_lock_t result = 0;
        asm volatile ("amoswap.w.aqrl %0, %1, (%2)" : "=r"(result) : "r"(val), "r"(ptr) : "memory");
    }

    spin_lock_t atomic_exchange(spin_lock_t* ptr, spin_lock_t val)
    {
        spin_lock_t result = 0;
        asm volatile ("amoswap.w.aqrl %0, %1, (%2)" : "=r"(result) : "r"(val), "r"(ptr) : "memory");
        return result;
    }

    static inline int spinlock_trylock(spin_lock_t *lock)
    {
        spin_lock_t oldval;

        oldval = atomic_exchange(lock, 1);

        return oldval == 0 ? 1 : 0;
    }

    static inline void spinlock_lock(spin_lock_t *lock)
    {
        while(atomic_exchange(lock, 1) != 0) {
        }
    }

    static inline void spinlock_unlock(spin_lock_t *lock)
    {
        atomic_store(lock, 0);
    }

    UBaseType_t uxCriticalNestings[configNUMBER_OF_CORES] = {0};
    spin_lock_t hw_sync_locks[portRTOS_SPINLOCK_COUNT] = {0};

#endif
/*-----------------------------------------------------------*/

/* Used to keep track of the number of nested calls to taskENTER_CRITICAL().  This
will be set to 0 prior to the first task being started. */
portLONG ulCriticalNesting = 0x9999UL;

/* Used to record one tack want to swtich task after enter critical area, we need know it
 * and implement task switch after exit critical area */
portLONG pendsvflag = 0;

#if( portHAS_STACK_OVERFLOW_CHECKING == 1 )
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, StackType_t *pxEndOfStack, TaskFunction_t pxCode, void *pvParameters )
#else
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
#endif
{
    extern int __global_pointer$;
    StackType_t *stk  = NULL;
    StackType_t temp = (StackType_t)pxTopOfStack;
    unsigned long status = SR_MPP_M | SR_MPIE;

    #if ( CONFIG_RISCV_VECTOR_ENABLED )
    {
        status |= SR_VS_INITIAL;
    }
    #endif

    #if ( CONFIG_RISCV_FPU_ENABLED )
    {
        status |= SR_FS_INITIAL;
    }
    #endif

    temp &= 0xFFFFFFFFFFFFFFF8UL;
    stk = (StackType_t *)temp;

    /* FS=0b01 MPP=0b11 MPIE=0b1 */
    *(--stk)  = (uint64_t)status;                    /* status */
    *(--stk)  = (uint64_t)pxCode;                    /* Entry Point */

    *(--stk)  = (uint64_t)0x3131313131313131L;       /* X31         */
    *(--stk)  = (uint64_t)0x3030303030303030L;       /* X30         */
    *(--stk)  = (uint64_t)0x2929292929292929L;       /* X29         */
    *(--stk)  = (uint64_t)0x2828282828282828L;       /* X28         */
    *(--stk)  = (uint64_t)0x2727272727272727L;       /* X27         */
    *(--stk)  = (uint64_t)0x2626262626262626L;       /* X26         */
    *(--stk)  = (uint64_t)0x2525252525252525L;       /* X25         */
    *(--stk)  = (uint64_t)0x2424242424242424L;       /* X24         */
    *(--stk)  = (uint64_t)0x2323232323232323L;       /* X23         */
    *(--stk)  = (uint64_t)0x2222222222222222L;       /* X22         */
    *(--stk)  = (uint64_t)0x2121212121212121L;       /* X21         */
    *(--stk)  = (uint64_t)0x2020202020202020L;       /* X20         */
    *(--stk)  = (uint64_t)0x1919191919191919L;       /* X19         */
    *(--stk)  = (uint64_t)0x1818181818181818L;       /* X18         */
    *(--stk)  = (uint64_t)0x1717171717171717L;       /* X17         */
    *(--stk)  = (uint64_t)0x1616161616161616L;       /* X16         */
    *(--stk)  = (uint64_t)0x1515151515151515L;       /* X15         */
    *(--stk)  = (uint64_t)0x1414141414141414L;       /* X14         */
    *(--stk)  = (uint64_t)0x1313131313131313L;       /* X13         */
    *(--stk)  = (uint64_t)0x1212121212121212L;       /* X12         */
    *(--stk)  = (uint64_t)0x1111111111111111L;       /* X11         */
    *(--stk)  = (uint64_t)pvParameters;              /* X10         */
    *(--stk)  = (uint64_t)0x0909090909090909L;       /* X9          */
    *(--stk)  = (uint64_t)pxTopOfStack;              /* X8, aka. fp */
    *(--stk)  = (uint64_t)0x0707070707070707L;       /* X7          */
    *(--stk)  = (uint64_t)0x0606060606060606L;       /* X6          */
    *(--stk)  = (uint64_t)0x0505050505050505L;       /* X5          */
    *(--stk)  = (uint64_t)0x0404040404040404L;       /* X4          */
    *(--stk)  = (uint64_t)&__global_pointer$;        /* X3          */
    *(--stk)  = (uint64_t)vTaskDelete;       /* X1          */
    #if ( CONFIG_RISCV_FPU_ENABLED )
    {
        *(--stk)  = (uint64_t)0x0L;                  /* FCSR        */

        int num = __riscv_flen / 8 * 32 / sizeof(StackType_t);
        for (int i = 0; i < num; i++) {
            *(--stk)  = (StackType_t)0x1234567812345678L;    /* F31 ~ F0    */
        }
    }
    #endif

    #if ( CONFIG_RISCV_VECTOR_ENABLED )
    {
        *(--stk)  = (uint64_t)0x0L;                  /* VXRM        */
        *(--stk)  = (uint64_t)0x0L;                  /* VXSAT       */
        *(--stk)  = (uint64_t)0x0L;                  /* VSTART      */
        *(--stk)  = (uint64_t)0x0L;                  /* VTYPE       */
        *(--stk)  = (uint64_t)0x0L;                  /* VL          */
        int vlenb = csi_vlenb_get_value();
        int num = vlenb * 32 / sizeof(StackType_t);
        for (int i = 0; i < num; i++) {
            *(--stk)  = (StackType_t)0x1234567812345678L;    /* V31 ~ V0    */
        }
    }
    #endif

    #if( portHAS_STACK_OVERFLOW_CHECKING == 1 )
    {
        if (stk <= pxEndOfStack)
        {
            printf("pxTopOfStack: %p, pxEndOfStack: %p, stk: %p\r\n", pxTopOfStack, pxEndOfStack, stk);
            configASSERT(pdFALSE);
            return NULL;
        }
    }
    #endif

    return stk;
}

#if ( configNUMBER_OF_CORES > 1 )
    static void prvTaskExitError(void)
    {
        volatile uint32_t ulDummy = 0;

        /* A function that implements a task must not exit or attempt to return to
        its caller as there is nothing to return to.  If a task wants to exit it
        should instead call vTaskDelete( NULL ).

        Artificially force an assert() to be triggered if configASSERT() is
        defined, then stop here so application writers can catch the error. */
        configASSERT(ulCriticalNesting == ~0UL);
        portDISABLE_INTERRUPTS();
        while (ulDummy == 0) {
            /* This file calls prvTaskExitError() after the scheduler has been
            started to remove a compiler warning about the function being defined
            but never called.  ulDummy is used purely to quieten other warnings
            about code appearing after this function is called - making ulDummy
            volatile makes the compiler think the function could return and
            therefore not output an 'unreachable code' warning for code that appears
            after it. */
            /* Sleep and wait for interrupt */
            __WFI();
        }
    }

    static volatile unsigned long ulSchedulerReady = 0;
    BaseType_t xPortStartScheduler(void)
    {
        portDISABLE_INTERRUPTS();

        BaseType_t xCoreID = portGET_CORE_ID();
        if (xCoreID == CORE_0) {
            ulSchedulerReady = 1;
        } else {
            // other cores wait for scheduler ready signal
            while (ulSchedulerReady == 0);
        }

        /* Initialise the critical nesting count ready for the first task. */
        portSET_CRITICAL_NESTING_COUNT(xCoreID, 0);

        /* Start the first task. */
        vPortStartTask();

        /* Should never get here as the tasks will now be executing!  Call the task
        exit error function to prevent compiler warnings about a static function
        not being called in the case that the application writer overrides this
        functionality by defining configTASK_RETURN_ADDRESS.  Call
        vTaskSwitchContext() so link time optimisation does not remove the
        symbol. */
        vTaskSwitchContext(xCoreID);
        prvTaskExitError();

        /* Should not get here! */
        return 0;
    }

    extern BaseType_t secondary_boot_flag;
    void SecondaryCoresUp(void)
    {
        extern void riscv_soc_start_cpu(int cpu_num);
        for (int i = 1; i < configNUMBER_OF_CORES; i++) {
            riscv_soc_start_cpu(i);
        }
        mb();
        secondary_boot_flag = 0xa55a;
    }

    void xPortTaskSwitch(long cpuid)
    {
        /* Clear software IRQ Pending bit */
        clear_software_irq( cpuid );
        vTaskSwitchContext( cpuid );
    }
#else /* configNUMBER_OF_CORES > 1 */
    BaseType_t xPortStartScheduler( void )
    {
        ulCriticalNesting = 0UL;

        vPortStartTask();

        return pdFALSE;
    }
#endif /* configNUMBER_OF_CORES > 1 */

void vPortEndScheduler( void )
{
    /* Not implemented. */
    for( ;; );
}

#if configNUMBER_OF_CORES > 1
    /* Note this is a single method with uxAcquire parameter since we have
    * static vars, the method is always called with a compile time constant for
    * uxAcquire, and the compiler should do the right thing! */
    void vPortRecursiveLock(unsigned long ulCoreNum, unsigned long ulLockNum, spin_lock_t *pxSpinLock, BaseType_t uxAcquire)
    {
        // can only control the lock pxSpinLock, don't use this function to control any other lock.
        static volatile uint8_t ucOwnedByCore[ configNUMBER_OF_CORES ][portRTOS_SPINLOCK_COUNT];
        static volatile uint8_t ucRecursionCountByLock[ portRTOS_SPINLOCK_COUNT ];

        configASSERT(ulLockNum < portRTOS_SPINLOCK_COUNT);

        if (uxAcquire) {
            if (!spinlock_trylock(pxSpinLock)) {
                if( ucOwnedByCore[ ulCoreNum ][ ulLockNum ] )
                {
                    configASSERT( ucRecursionCountByLock[ ulLockNum ] != 255u );
                    ucRecursionCountByLock[ ulLockNum ]++;
                    return;
                }
                spinlock_lock(pxSpinLock);
            }

            configASSERT( ucRecursionCountByLock[ ulLockNum ] == 0 );
            ucRecursionCountByLock[ ulLockNum ] = 1;
            ucOwnedByCore[ ulCoreNum ][ ulLockNum ] = 1;
        } else {
            configASSERT( ( ucOwnedByCore[ ulCoreNum ] [ulLockNum ] ) != 0 );
            configASSERT( ucRecursionCountByLock[ ulLockNum ] != 0 );

            if( !--ucRecursionCountByLock[ ulLockNum ] )
            {
                ucOwnedByCore[ ulCoreNum ] [ ulLockNum ] = 0;
                spinlock_unlock(pxSpinLock);
            }
        }
    }

#else  /* configNUMBER_OF_CORES > 1 */

    void vPortEnterCritical( void )
    {
        portDISABLE_INTERRUPTS();
        ulCriticalNesting ++;
    }

    void vPortExitCritical( void )
    {
        if (ulCriticalNesting == 0) {
            while(1);
        }

        ulCriticalNesting --;
        if (ulCriticalNesting == 0) {
            portENABLE_INTERRUPTS();

            if (pendsvflag) {
                pendsvflag = 0;
                portYIELD();
            }
        }
    }
#endif /* configNUMBER_OF_CORES > 1 */

void xPortSysTickHandler( void )
{
    UBaseType_t ulPreviousMask;
    /* Tasks or ISRs running on other cores may still in critical section in
    * multiple cores environment. Incrementing tick needs to performed in
    * critical section. */
    ulPreviousMask = taskENTER_CRITICAL_FROM_ISR();
    {
        /* Increment the RTOS tick. */
        if (xTaskIncrementTick() != pdFALSE) {
            /* A context switch is required.  Context switching is performed in
            the SWI interrupt.  Pend the SWI interrupt. */
            portYIELD();
        }
    }
    taskEXIT_CRITICAL_FROM_ISR( ulPreviousMask );
}


#if configNUMBER_OF_CORES > 1
    void clear_software_irq(int xCoreID)
    {
        *(uint32_t*)((uintptr_t)CLINT_BASE + ((xCoreID) << 2)) = 0x0;
        mb();
    }

    void vPortYield_Core(int xCoreID)
    {
        *(uint32_t*)((uintptr_t)CLINT_BASE + ((xCoreID) << 2)) = 0x1;
        mb();
    }
#endif /* configNUMBER_OF_CORES > 1 */

void vPortEnableInterrupt( void )
{
    __enable_irq();
}

void vPortDisableInterrupt( void )
{
    __disable_irq();
}


__attribute__((weak)) void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    void *pxTopOfStack = (void *)(*(unsigned long *)pxTask);
    printf("!!! task [%s] stack overflow. pxTop: %p\r\n", pcTaskName, pxTopOfStack);
    for(;;);
}

__attribute__((weak)) void vApplicationMallocFailedHook( void )
{
    for(;;);
}
