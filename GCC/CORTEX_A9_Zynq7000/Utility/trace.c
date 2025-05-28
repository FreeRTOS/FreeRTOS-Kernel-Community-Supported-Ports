#include "FreeRTOS.h"

#if ( configUSE_TRACE_MACROS == 1 )
    /* LOG VARIABLES DEFINITION. */
    #if ( configNUMBER_OF_CORES > 1 && configCORE_TO_TRACE == 2 )
        /* A statically defined log buffer is used to record the program execution events. */
        char cLogBuffer[ configNUMBER_OF_CORES ][ traceLOG_ENTRIES ][ traceENTRY_LENGTH ];
        uint8_t ucLogEntry[ configNUMBER_OF_CORES ] = { 0 };
    #else
        char cLogBuffer[ traceLOG_ENTRIES ][ traceENTRY_LENGTH ];
        uint8_t ucLogEntry = 0;
    #endif /* #if ( configNUMBER_OF_CORES > 1 && configCORE_TO_TRACE == 2 ) */

    #if ( configTRACE_CONTEXT_SWITCH == 1)
        /* ucSwitchInDone is used to check that each call to traceTASK_SWITCHED_OUT
         * is followed by a matching call to traceTASK_SWITCHED_IN. This can help
         * to detect unexpected behaviors within vTaskSwitchContext(). */
        #if ( configNUMBER_OF_CORES == 1 )
            uint8_t ucSwitchInDone = (configCORE_TO_TRACE == 0)? pdFALSE:pdTRUE;
        #else
            uint8_t ucSwitchInDone[ configNUMBER_OF_CORES ] = { pdFALSE, pdTRUE };
        #endif
    #endif /* #if ( configTRACE_CONTEXT_SWITCH == 1) */

    #if ( configTRACE_RECURSIVE_LOCKS == 1 )
        extern void vPortGetLock(volatile lock_t* xLockAddr);
        extern void vPortReleaseLock(volatile lock_t* xLockAddr);
        extern volatile lock_t xLocks[configNUMBER_OF_CORES];

        /* C wrapper to trace the cores activity on the locks. 
         * The lock port macros are redefined to vPortRecursiveLock in
         * portmacro.h when configTRACE_RECURSIVE_LOCKS == 1. */
        void vPortRecursiveLock(UBaseType_t uxLock, BaseType_t xAcquire){
            if(xAcquire){
                vPortGetLock(&xLocks[uxLock]);
                traceLOCK_AFTER_ACQUIRE(uxLock, portGET_CORE_ID());
            }
            else{
                traceLOCK_BEFORE_RELEASE(uxLock, portGET_CORE_ID());
                vPortReleaseLock(&xLocks[uxLock]);
            }
        }
    #endif /* #if ( configTRACE_RECURSIVE_LOCKS == 1 ) */

#elif ( configUSE_PERCEPIO_VIEW == 1 ) 
    /* To correctly generate timestamps when multiple cores are traced
     * a timer accessible by all cores is needed. */   
    #include "gtimer.h"

    void vTraceSetupGlobalTimer( void )
    {
        if ( !GlobalTimer_isEnabled() )
        {
            /* Initialize the counter to 0x0. */
            GlobalTimer_SetCounter(0);

            /* The global timer is clocked at the same frequency of the cores
            private timers so the prescaler value must be the same in order to match
            the SysTick timer frequency. This is not strictly necessary but avoids the
            need to manually adjust the timestamps when the traced program timings are
            analyzed. */
            GlobalTimer_SetPrescaler(0);
            GlobalTimer_ClearControlBits();

            GlobalTimer_Start();
        }
        else
        {
            GlobalTimer_SetPrescaler(0);
            GlobalTimer_ClearControlBits();
        }
    }
#endif /* #if ( configUSE_TRACE_MACROS == 1 ) */
