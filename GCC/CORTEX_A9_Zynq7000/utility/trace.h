#ifndef TRACE_H
#define TRACE_H

/*
 * The trace functions in the trace.* files are provided to
 * ease the debug of applications for the Zynq7000 SMP port.
 * 
 * All these functions should also work in other Cortex-A9
 * based systems with minimal effort.
 * 
 * Two trace approaches are available:
 *  - Custom trace functions
 *  - Percepio View
 * 
 * They are mutually exclusive since they redefine the same trace
 * macros in the FreeRTOS kernel code.
 * 
 */
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef configUSE_TRACE_MACROS
    #define configUSE_TRACE_MACROS 0
#endif

#ifndef configUSE_PERCEPIO_VIEW
    #define configUSE_PERCEPIO_VIEW 0
#endif

#if ( configUSE_TRACE_MACROS == 1 && configUSE_PERCEPIO_VIEW == 1 )
    #error configUSE_TRACE_MACROS and configUSE_PERCEPIO_VIEW cannot be used at the same time
#elif ( configUSE_TRACE_MACROS == 1 )

    /*************************************************************/
    /* Custom Trace Macros ***************************************/
    /*************************************************************/

    /* Configuration macros default definition. */
    #ifndef configTRACE_CONTEXT_SWITCH
        #define configTRACE_CONTEXT_SWITCH 0
        #warning Define configTRACE_CONTEXT_SWITCH to 1 to enable the context switch \
        trace macros. Defined to 0 by default.
    #endif

    #if ( configNUMBER_OF_CORES == 1 )
        #if ( configTRACE_RECURSIVE_LOCKS == 1 )
            #error configTRACE_RECURSIVE_LOCKS must be set to 0 when \
            configNUMBER_OF_CORES == 1.
        #endif
    #else /* #if ( configNUMBER_OF_CORES == 1 ) */
        #ifndef configCORE_TO_TRACE
            #define configCORE_TO_TRACE 0
            #warning configCORE_TO_TRACE has been defined to 0. \
            It can be defined to trace CORE1 (1), CORE0 (0) or both (2).
        #endif

        #ifndef configTRACE_RECURSIVE_LOCKS
            #define configTRACE_RECURSIVE_LOCKS 0
            #warning Define configTRACE_RECURSIVE_LOCKS to 1 to enable the recursive locks \
            trace macros. Defined to 0 by default.
        #endif

        #ifndef configTRACE_SCHEDULER_START
            #define configTRACE_RECURSIVE_LOCKS 0
            #warning Define configTRACE_SCHEDULER_START to 1 to enable the scheduler start \
                trace macros. Defined to 0 by default.
        #endif
    #endif /* #if ( configNUMBER_OF_CORES == 1 ) */


    /* Dimension of the log buffer. */
    #define traceLOG_ENTRIES        30
    /* Maximum length of each entry of the log buffer. */
    #define traceENTRY_LENGTH       50


    #if ( configNUMBER_OF_CORES == 1 )
        /* Log data structures. */
        extern char cLogBuffer[ traceLOG_ENTRIES ][ traceENTRY_LENGTH ];
        extern uint8_t ucLogEntry;

        #if ( configTRACE_CONTEXT_SWITCH == 1)
            extern uint8_t ucSwitchInDone;
        #endif

        /* Utility defines. */
        #define traceLOG_MESSAGE(...)                           \
        {                                                       \
            snprintf(                                           \
                cLogBuffer[ucLogEntry],                         \
                (size_t) traceENTRY_LENGTH,                     \
                __VA_ARGS__                                     \
            );                                                  \
            ucLogEntry = (ucLogEntry + 1) % traceLOG_ENTRIES;   \
        }

        /* Log macros definition. */
        #if ( configTRACE_CONTEXT_SWITCH == 1 )
            /* traceTASK_SWITCHED_IN/OUT are called inside vTaskSwitchContext. */
            #define traceTASK_SWITCHED_IN()                     \
            {                                                   \
                ucSwitchInDone = pdTRUE;                        \
                traceLOG_MESSAGE(                               \
                    "%s SWITCHED-IN\r\n",                       \
                    pxCurrentTCB->pcTaskName);                  \
            }

            #define traceTASK_SWITCHED_OUT()                    \
            {                                                   \
                if(ucSwitchInDone == pdTRUE)                    \
                {                                               \
                    ucSwitchInDone = pdFALSE;                   \
                    traceLOG_MESSAGE(                           \
                        "%s SWITCHED-OUT\r\n",                  \
                        pxCurrentTCB->pcTaskName);              \
                }                                               \
                else                                            \
                {                                               \
                    while(1);                                   \
                }                                               \
            }

        #endif /* #if ( configTRACE_CONTEXT_SWITCH == 1 ) */

    #else /* #if ( configNUMBER_OF_CORES == 1 ) */

        /* Log data structures. */
        #if ( configCORE_TO_TRACE == 2 )
            /* A statically defined log buffer is used to record the program execution events. */
            extern char cLogBuffer[configNUMBER_OF_CORES][traceLOG_ENTRIES][traceENTRY_LENGTH];
            extern uint8_t ucLogEntry[configNUMBER_OF_CORES];
        #else /* #if ( configCORE_TO_TRACE == 2 ) */
            extern char cLogBuffer[traceLOG_ENTRIES][traceENTRY_LENGTH];
            extern uint8_t ucLogEntry;
        #endif /* #if ( configCORE_TO_TRACE == 2 ) */

        #if ( configTRACE_CONTEXT_SWITCH == 1)
            /* ucSwitchInDone is used to check that each call to traceTASK_SWITCHED_OUT
            * is followed by a matching call to traceTASK_SWITCHED_IN. This can help
            * to detect unexpected behaviors within vTaskSwitchContext(). */   
            extern uint8_t ucSwitchInDone[configNUMBER_OF_CORES];
        #endif

        /* Utility defines. */
        #if ( configCORE_TO_TRACE == 2 )
            /* The traceENTER_CRITICAL macro is not used when multiple cores are
            * traced because there is no critical section to guard. */
            #define traceENTER_CRITICAL( xCoreID )

            #define traceLOG_MESSAGE( xCoreID, ...)                                 \
            {                                                                       \
                snprintf(                                                           \
                    cLogBuffer[xCoreID][ucLogEntry[xCoreID]],                       \
                    (size_t) traceENTRY_LENGTH,                                     \
                    __VA_ARGS__                                                     \
                );                                                                  \
                ucLogEntry[xCoreID] = (ucLogEntry[xCoreID] + 1) % traceLOG_ENTRIES; \
            }

        #elif ( configCORE_TO_TRACE == 1 || configCORE_TO_TRACE == 0 )
            /* Guard the log macro from the other cores. */
            #define traceENTER_CRITICAL( xCoreID )       if( xCoreID == configCORE_TO_TRACE )

            #define traceLOG_MESSAGE( xCoreID, ...)                                 \
            {                                                                       \
                snprintf(                                                           \
                    cLogBuffer[ucLogEntry],                                         \
                    (size_t) traceENTRY_LENGTH,                                     \
                    __VA_ARGS__                                                     \
                );                                                                  \
                ucLogEntry = (ucLogEntry + 1) % traceLOG_ENTRIES;                   \
            }
        #else
            #error Invalid configuration of configCORE_TO_TRACE; define it to 0, 1 or 2
        #endif /* #if ( configCORE_TO_TRACE == 2 ) */

        /* Log macros definition. */
        #if ( configTRACE_RECURSIVE_LOCKS == 1 )
            /* Macros to trace the activity on the recursive locks. */
            #define traceLOCK_AFTER_ACQUIRE(uxLock, xCoreID)            \
            {                                                           \
                traceENTER_CRITICAL(xCoreID)                            \
                {                                                       \
                    traceLOG_MESSAGE(                                   \
                        xCoreID,                                        \
                        "[CORE %u] ACQUIRED lock %s\r\n",               \
                        (unsigned int) xCoreID, (uxLock)? "TASK":"ISR");\
                }                                                       \
            }
                
            #define traceLOCK_BEFORE_RELEASE(uxLock, xCoreID)           \
            {                                                           \
                traceENTER_CRITICAL(xCoreID)                            \
                {                                                       \
                    traceLOG_MESSAGE(                                   \
                        xCoreID,                                        \
                        "[CORE %u] RELEASING lock %s\r\n",              \
                        (unsigned int) xCoreID, (uxLock)? "TASK":"ISR");\
                }                                                       \
            }

        #endif /* #if ( configTRACE_RECURSIVE_LOCKS == 1 ) */

        #if ( configTRACE_SCHEDULER_START == 1 )
            #define traceENTER_vTaskStartScheduler()                        \
            {                                                               \
                const uint32_t ulCoreID = portGET_CORE_ID();                \
                traceENTER_CRITICAL(ulCoreID)                               \
                {                                                           \
                    traceLOG_MESSAGE(                                       \
                        ulCoreID,                                           \
                        "[CORE %u] CALL to vTaskStartScheduler\r\n",        \
                        (unsigned int) ulCoreID);                           \
                }                                                           \
            }

            #define traceRETURN_vTaskStartScheduler()                       \
            {                                                               \
                const uint32_t ulCoreID = portGET_CORE_ID();                \
                traceENTER_CRITICAL(ulCoreID)                               \
                {                                                           \
                    traceLOG_MESSAGE(                                       \
                        ulCoreID,                                           \
                        "[CORE %u] RETURN from vTaskStartScheduler\r\n",    \
                        (unsigned int) ulCoreID);                           \
                }                                                           \
            }
        #endif /* #if ( configTRACE_SCHEDULER_START == 1 ) */

        #if ( configTRACE_CONTEXT_SWITCH == 1 )
            /* traceTASK_SWITCHED_IN/OUT are called inside vTaskSwitchContext */
            #define traceTASK_SWITCHED_IN()                                             \
            {                                                                           \
                const uint32_t ulCoreID = portGET_CORE_ID();                            \
                traceENTER_CRITICAL(ulCoreID)                                           \
                {                                                                       \
                    ucSwitchInDone[ulCoreID] = pdTRUE;                                  \
                    traceLOG_MESSAGE(                                                   \
                        ulCoreID,                                                       \
                        "[CORE %u] %s SWITCHED-IN\r\n",                                 \
                        (unsigned int) ulCoreID, pxCurrentTCBs[ulCoreID]->pcTaskName);  \
                }                                                                       \
            }

            #define traceTASK_SWITCHED_OUT()                                                \
            {                                                                               \
                const uint32_t ulCoreID = portGET_CORE_ID();                                \
                traceENTER_CRITICAL(ulCoreID)                                               \
                {                                                                           \
                    if(ucSwitchInDone[ulCoreID] == pdTRUE)                                  \
                    {                                                                       \
                        ucSwitchInDone[ulCoreID] = pdFALSE;                                 \
                        traceLOG_MESSAGE(                                                   \
                            ulCoreID,                                                       \
                            "[CORE %u] %s SWITCHED-OUT\r\n",                                \
                            (unsigned int) ulCoreID, pxCurrentTCBs[ulCoreID]->pcTaskName);  \
                    }                                                                       \
                    else                                                                    \
                    {                                                                       \
                        while(1);                                                           \
                    }                                                                       \
                }                                                                           \
            }
        #endif /* #if ( configTRACE_CONTEXT_SWITCH == 1 ) */
    #endif /* #if ( configNUMBER_OF_CORES == 1 ) */
#elif ( configUSE_PERCEPIO_VIEW == 1 )

    /*************************************************************/
    /* Percepio View *********************************************/
    /*************************************************************/
    #include <trcRecorder.h>

    #if ( configNUMBER_OF_CORES > 1 )
        /* To correctly generate timestamps when multiple cores are traced
         * a timer accessible by all cores is needed. 
         * Call vTraceSetupGlobalTimer( ) before xTraceInitialize() when
         * configNUMBER_OF_CORES > 1. */
        void vTraceSetupGlobalTimer( void ); 
    #endif

    #if ( configUSE_DAEMON_TASK_STARTUP_HOOK == 0 )
        #error Percepio View needs configUSE_DAEMON_TASK_STARTUP_HOOK set to 1. \
        Check vApplicationDaemonTaskStartupHook() inside portZynq7000.c for more info. 
    #endif

#endif /* #if ( configUSE_TRACE_MACROS == 1 && configUSE_PERCEPIO_VIEW == 1 ) */

#ifdef __cplusplus
}
#endif

#endif