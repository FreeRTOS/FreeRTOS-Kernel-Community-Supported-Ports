/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "mpu_prototypes.h"
#include "mpu_syscall_numbers.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if ( configUSE_MPU_WRAPPERS_V1 == 1 )
    #error "This port only support mpu_wrappers_v2"
#endif

/*-----------------------------------------------------------*/
#if ( INCLUDE_xTaskDelayUntil == 1 )

    BaseType_t MPU_xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                                    const TickType_t xTimeIncrement ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                                    const TickType_t xTimeIncrement ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskDelayUntilImpl \n"
         "bnez tp, 1f                     \n"
         "    j MPU_xTaskDelayUntilImpl   \n"
         "1:                              \n"
         "    li   a7, %0                 \n"
         "    ecall                       \n"
         "    ret                         \n"
     : : "i"(SYSTEM_CALL_xTaskDelayUntil) : "memory");
 }

#endif /* if ( INCLUDE_xTaskDelayUntil == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskAbortDelay == 1 )

    BaseType_t MPU_xTaskAbortDelay( TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskAbortDelay( TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskAbortDelayImpl \n"
         "bnez tp, 1f                     \n"
         "    j MPU_xTaskAbortDelayImpl   \n"
         "1:                              \n"
         "    li   a7, %0                 \n"
         "    ecall                       \n"
         "    ret                         \n"
     : : "i"(SYSTEM_CALL_xTaskAbortDelay) : "memory");
 }

#endif /* if ( INCLUDE_xTaskAbortDelay == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )

    void MPU_vTaskDelay( const TickType_t xTicksToDelay ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskDelay( const TickType_t xTicksToDelay ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskDelayImpl \n"
         "bnez tp, 1f                \n"
         "    j MPU_vTaskDelayImpl   \n"
         "1:                         \n"
         "    li   a7, %0            \n"
         "    ecall                  \n"
         "    ret                    \n"
     : : "i"(SYSTEM_CALL_vTaskDelay) : "memory");
 }

#endif /* if ( INCLUDE_vTaskDelay == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

    UBaseType_t MPU_uxTaskPriorityGet( const TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    UBaseType_t MPU_uxTaskPriorityGet( const TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTaskPriorityGetImpl \n"
         "bnez tp, 1f                       \n"
         "    j MPU_uxTaskPriorityGetImpl   \n"
         "1:                                \n"
         "    li   a7, %0                   \n"
         "    ecall                         \n"
         "    ret                           \n"
     : : "i"(SYSTEM_CALL_uxTaskPriorityGet) : "memory");
 }

#endif /* if ( INCLUDE_uxTaskPriorityGet == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_eTaskGetState == 1 )

    eTaskState MPU_eTaskGetState( TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    eTaskState MPU_eTaskGetState( TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_eTaskGetStateImpl \n"
         "bnez tp, 1f                   \n"
         "    j MPU_eTaskGetStateImpl   \n"
         "1:                            \n"
         "    li   a7, %0               \n"
         "    ecall                     \n"
         "    ret                       \n"
     : : "i"(SYSTEM_CALL_eTaskGetState) : "memory");
 }

#endif /* if ( INCLUDE_eTaskGetState == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

    void MPU_vTaskGetInfo( TaskHandle_t xTask,
                           TaskStatus_t * pxTaskStatus,
                           BaseType_t xGetFreeStackSpace,
                           eTaskState eState ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskGetInfo( TaskHandle_t xTask,
                           TaskStatus_t * pxTaskStatus,
                           BaseType_t xGetFreeStackSpace,
                           eTaskState eState ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskGetInfoImpl \n"
         "bnez tp, 1f                  \n"
         "    j MPU_vTaskGetInfoImpl   \n"
         "1:                           \n"
         "    li   a7, %0              \n"
         "    ecall                    \n"
         "    ret                      \n"
     : : "i"(SYSTEM_CALL_vTaskGetInfo) : "memory");
 }

#endif /* if ( configUSE_TRACE_FACILITY == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )

    TaskHandle_t MPU_xTaskGetIdleTaskHandle( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TaskHandle_t MPU_xTaskGetIdleTaskHandle( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGetIdleTaskHandleImpl \n"
         "bnez tp, 1f                            \n"
         "    j MPU_xTaskGetIdleTaskHandleImpl   \n"
         "1:                                     \n"
         "    li   a7, %0                        \n"
         "    ecall                              \n"
         "    ret                                \n"
     : : "i"(SYSTEM_CALL_xTaskGetIdleTaskHandle) : "memory");
 }

#endif /* if ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    void MPU_vTaskSuspend( TaskHandle_t xTaskToSuspend ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskSuspend( TaskHandle_t xTaskToSuspend ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskSuspendImpl \n"
         "bnez tp, 1f                  \n"
         "    j MPU_vTaskSuspendImpl   \n"
         "1:                           \n"
         "    li   a7, %0              \n"
         "    ecall                    \n"
         "    ret                      \n"
     : : "i"(SYSTEM_CALL_vTaskSuspend) : "memory");
 }

#endif /* if ( INCLUDE_vTaskSuspend == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskSuspend == 1 )

    void MPU_vTaskResume( TaskHandle_t xTaskToResume ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskResume( TaskHandle_t xTaskToResume ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskResumeImpl \n"
         "bnez tp, 1f                 \n"
         "    j MPU_vTaskResumeImpl   \n"
         "1:                          \n"
         "    li   a7, %0             \n"
         "    ecall                   \n"
         "    ret                     \n"
     : : "i"(SYSTEM_CALL_vTaskResume) : "memory");
 }

#endif /* if ( INCLUDE_vTaskSuspend == 1 ) */
/*-----------------------------------------------------------*/

TickType_t MPU_xTaskGetTickCount( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

TickType_t MPU_xTaskGetTickCount( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGetTickCountImpl \n"
         "bnez tp, 1f                       \n"
         "    j MPU_xTaskGetTickCountImpl   \n"
         "1:                                \n"
         "    li   a7, %0                   \n"
         "    ecall                         \n"
         "    ret                           \n"
     : : "i"(SYSTEM_CALL_xTaskGetTickCount) : "memory");
 }
/*-----------------------------------------------------------*/

UBaseType_t MPU_uxTaskGetNumberOfTasks( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

UBaseType_t MPU_uxTaskGetNumberOfTasks( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTaskGetNumberOfTasksImpl \n"
         "bnez tp, 1f                            \n"
         "    j MPU_uxTaskGetNumberOfTasksImpl   \n"
         "1:                                     \n"
         "    li   a7, %0                        \n"
         "    ecall                              \n"
         "    ret                                \n"
     : : "i"(SYSTEM_CALL_uxTaskGetNumberOfTasks) : "memory");
 }
/*-----------------------------------------------------------*/

#if ( configGENERATE_RUN_TIME_STATS == 1 )

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimeCounter( const TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimeCounter( const TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGetRunTimeCounterImpl \n"
         "bnez tp, 1f                             \n"
         "    j MPU_ulTaskGetRunTimeCounterImpl   \n"
         "1:                                      \n"
         "    li   a7, %0                         \n"
         "    ecall                               \n"
         "    ret                                 \n"
     : : "i"(SYSTEM_CALL_ulTaskGetRunTimeCounter) : "memory");
 }

#endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configGENERATE_RUN_TIME_STATS == 1 )

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimePercent( const TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimePercent( const TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGetRunTimePercentImpl \n"
         "bnez tp, 1f                             \n"
         "    j MPU_ulTaskGetRunTimePercentImpl   \n"
         "1:                                      \n"
         "    li   a7, %0                         \n"
         "    ecall                               \n"
         "    ret                                 \n"
     : : "i"(SYSTEM_CALL_ulTaskGetRunTimePercent) : "memory");
 }

#endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) */
/*-----------------------------------------------------------*/

#if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) )

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimePercent( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimePercent( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGetIdleRunTimePercentImpl \n"
         "bnez tp, 1f                                 \n"
         "    j MPU_ulTaskGetIdleRunTimePercentImpl   \n"
         "1:                                          \n"
         "    li   a7, %0                             \n"
         "    ecall                                   \n"
         "    ret                                     \n"
     : : "i"(SYSTEM_CALL_ulTaskGetIdleRunTimePercent) : "memory");
 }

#endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) )

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimeCounter( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimeCounter( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGetIdleRunTimeCounterImpl \n"
         "bnez tp, 1f                                 \n"
         "    j MPU_ulTaskGetIdleRunTimeCounterImpl   \n"
         "1:                                          \n"
         "    li   a7, %0                             \n"
         "    ecall                                   \n"
         "    ret                                     \n"
     : : "i"(SYSTEM_CALL_ulTaskGetIdleRunTimeCounter) : "memory");
 }

#endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    void MPU_vTaskSetApplicationTaskTag( TaskHandle_t xTask,
                                         TaskHookFunction_t pxHookFunction ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskSetApplicationTaskTag( TaskHandle_t xTask,
                                         TaskHookFunction_t pxHookFunction ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskSetApplicationTaskTagImpl \n"
         "bnez tp, 1f                                \n"
         "    j MPU_vTaskSetApplicationTaskTagImpl   \n"
         "1:                                         \n"
         "    li   a7, %0                            \n"
         "    ecall                                  \n"
         "    ret                                    \n"
     : : "i"(SYSTEM_CALL_vTaskSetApplicationTaskTag) : "memory");
 }

#endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

    TaskHookFunction_t MPU_xTaskGetApplicationTaskTag( TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TaskHookFunction_t MPU_xTaskGetApplicationTaskTag( TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGetApplicationTaskTagImpl \n"
         "bnez tp, 1f                                \n"
         "    j MPU_xTaskGetApplicationTaskTagImpl   \n"
         "1:                                         \n"
         "    li   a7, %0                            \n"
         "    ecall                                  \n"
         "    ret                                    \n"
     : : "i"(SYSTEM_CALL_xTaskGetApplicationTaskTag) : "memory");
 }

#endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */
/*-----------------------------------------------------------*/

#if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )

    void MPU_vTaskSetThreadLocalStoragePointer( TaskHandle_t xTaskToSet,
                                                BaseType_t xIndex,
                                                void * pvValue ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTaskSetThreadLocalStoragePointer( TaskHandle_t xTaskToSet,
                                                BaseType_t xIndex,
                                                void * pvValue ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskSetThreadLocalStoragePointerImpl \n"
         "bnez tp, 1f                                       \n"
         "    j MPU_vTaskSetThreadLocalStoragePointerImpl   \n"
         "1:                                                \n"
         "    li   a7, %0                                   \n"
         "    ecall                                         \n"
         "    ret                                           \n"
     : : "i"(SYSTEM_CALL_vTaskSetThreadLocalStoragePointer) : "memory");
 }

#endif /* if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 ) */
/*-----------------------------------------------------------*/

#if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )

    void * MPU_pvTaskGetThreadLocalStoragePointer( TaskHandle_t xTaskToQuery,
                                                   BaseType_t xIndex ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void * MPU_pvTaskGetThreadLocalStoragePointer( TaskHandle_t xTaskToQuery,
                                                   BaseType_t xIndex ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_pvTaskGetThreadLocalStoragePointerImpl \n"
         "bnez tp, 1f                                        \n"
         "    j MPU_pvTaskGetThreadLocalStoragePointerImpl   \n"
         "1:                                                 \n"
         "    li   a7, %0                                    \n"
         "    ecall                                          \n"
         "    ret                                            \n"
     : : "i"(SYSTEM_CALL_pvTaskGetThreadLocalStoragePointer) : "memory");
 }

#endif /* if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )

    UBaseType_t MPU_uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray,
                                          const UBaseType_t uxArraySize,
                                          configRUN_TIME_COUNTER_TYPE * const pulTotalRunTime ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    UBaseType_t MPU_uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray,
                                          const UBaseType_t uxArraySize,
                                          configRUN_TIME_COUNTER_TYPE * const pulTotalRunTime ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTaskGetSystemStateImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_uxTaskGetSystemStateImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_uxTaskGetSystemState) : "memory");
 }

#endif /* if ( configUSE_TRACE_FACILITY == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )

    UBaseType_t MPU_uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    UBaseType_t MPU_uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTaskGetStackHighWaterMarkImpl \n"
         "bnez tp, 1f                                 \n"
         "    j MPU_uxTaskGetStackHighWaterMarkImpl   \n"
         "1:                                          \n"
         "    li   a7, %0                             \n"
         "    ecall                                   \n"
         "    ret                                     \n"
     : : "i"(SYSTEM_CALL_uxTaskGetStackHighWaterMark) : "memory");
 }

#endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 )

    configSTACK_DEPTH_TYPE MPU_uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    configSTACK_DEPTH_TYPE MPU_uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTaskGetStackHighWaterMark2Impl \n"
         "bnez tp, 1f                                  \n"
         "    j MPU_uxTaskGetStackHighWaterMark2Impl   \n"
         "1:                                           \n"
         "    li   a7, %0                              \n"
         "    ecall                                    \n"
         "    ret                                      \n"
     : : "i"(SYSTEM_CALL_uxTaskGetStackHighWaterMark2) : "memory");
 }

#endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 ) */
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) )

    TaskHandle_t MPU_xTaskGetCurrentTaskHandle( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TaskHandle_t MPU_xTaskGetCurrentTaskHandle( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGetCurrentTaskHandleImpl \n"
         "bnez tp, 1f                               \n"
         "    j MPU_xTaskGetCurrentTaskHandleImpl   \n"
         "1:                                        \n"
         "    li   a7, %0                           \n"
         "    ecall                                 \n"
         "    ret                                   \n"
     : : "i"(SYSTEM_CALL_xTaskGetCurrentTaskHandle) : "memory");
 }

#endif /* if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( INCLUDE_xTaskGetSchedulerState == 1 )

    BaseType_t MPU_xTaskGetSchedulerState( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskGetSchedulerState( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGetSchedulerStateImpl \n"
         "bnez tp, 1f                            \n"
         "    j MPU_xTaskGetSchedulerStateImpl   \n"
         "1:                                     \n"
         "    li   a7, %0                        \n"
         "    ecall                              \n"
         "    ret                                \n"
     : : "i"(SYSTEM_CALL_xTaskGetSchedulerState) : "memory");
 }

#endif /* if ( INCLUDE_xTaskGetSchedulerState == 1 ) */
/*-----------------------------------------------------------*/

void MPU_vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

void MPU_vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTaskSetTimeOutStateImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_vTaskSetTimeOutStateImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_vTaskSetTimeOutState) : "memory");
 }
/*-----------------------------------------------------------*/

BaseType_t MPU_xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut,
                                     TickType_t * const pxTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

BaseType_t MPU_xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut,
                                     TickType_t * const pxTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskCheckForTimeOutImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_xTaskCheckForTimeOutImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_xTaskCheckForTimeOut) : "memory");
 }
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t MPU_xTaskGenericNotifyEntry( const xTaskGenericNotifyParams_t * pxParams ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskGenericNotifyEntry( const xTaskGenericNotifyParams_t * pxParams ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGenericNotifyImpl \n"
         "bnez tp, 1f                             \n"
         "    j MPU_xTaskGenericNotifyImpl   \n"
         "1:                                      \n"
         "    li   a7, %0                         \n"
         "    ecall                               \n"
         "    ret                                 \n"
     : : "i"(SYSTEM_CALL_xTaskGenericNotify) : "memory");
 }

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t MPU_xTaskGenericNotifyWaitEntry( const xTaskGenericNotifyWaitParams_t * pxParams ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskGenericNotifyWaitEntry( const xTaskGenericNotifyWaitParams_t * pxParams ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGenericNotifyWaitImpl \n"
         "bnez tp, 1f                                 \n"
         "    j MPU_xTaskGenericNotifyWaitImpl   \n"
         "1:                                          \n"
         "    li   a7, %0                             \n"
         "    ecall                                   \n"
         "    ret                                     \n"
     : : "i"(SYSTEM_CALL_xTaskGenericNotifyWait) : "memory");
 }

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    uint32_t MPU_ulTaskGenericNotifyTake( UBaseType_t uxIndexToWaitOn,
                                          BaseType_t xClearCountOnExit,
                                          TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    uint32_t MPU_ulTaskGenericNotifyTake( UBaseType_t uxIndexToWaitOn,
                                          BaseType_t xClearCountOnExit,
                                          TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGenericNotifyTakeImpl \n"
         "bnez tp, 1f                             \n"
         "    j MPU_ulTaskGenericNotifyTakeImpl   \n"
         "1:                                      \n"
         "    li   a7, %0                         \n"
         "    ecall                               \n"
         "    ret                                 \n"
     : : "i"(SYSTEM_CALL_ulTaskGenericNotifyTake) : "memory");
 }

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    BaseType_t MPU_xTaskGenericNotifyStateClear( TaskHandle_t xTask,
                                                 UBaseType_t uxIndexToClear ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTaskGenericNotifyStateClear( TaskHandle_t xTask,
                                                 UBaseType_t uxIndexToClear ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTaskGenericNotifyStateClearImpl \n"
         "bnez tp, 1f                                  \n"
         "    j MPU_xTaskGenericNotifyStateClearImpl   \n"
         "1:                                           \n"
         "    li   a7, %0                              \n"
         "    ecall                                    \n"
         "    ret                                      \n"
     : : "i"(SYSTEM_CALL_xTaskGenericNotifyStateClear) : "memory");
 }

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TASK_NOTIFICATIONS == 1 )

    uint32_t MPU_ulTaskGenericNotifyValueClear( TaskHandle_t xTask,
                                                UBaseType_t uxIndexToClear,
                                                uint32_t ulBitsToClear ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    uint32_t MPU_ulTaskGenericNotifyValueClear( TaskHandle_t xTask,
                                                UBaseType_t uxIndexToClear,
                                                uint32_t ulBitsToClear ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_ulTaskGenericNotifyValueClearImpl \n"
         "bnez tp, 1f                                   \n"
         "    j MPU_ulTaskGenericNotifyValueClearImpl   \n"
         "1:                                            \n"
         "    li   a7, %0                               \n"
         "    ecall                                     \n"
         "    ret                                       \n"
     : : "i"(SYSTEM_CALL_ulTaskGenericNotifyValueClear) : "memory");
 }

#endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue,
                                  const void * const pvItemToQueue,
                                  TickType_t xTicksToWait,
                                  const BaseType_t xCopyPosition ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue,
                                  const void * const pvItemToQueue,
                                  TickType_t xTicksToWait,
                                  const BaseType_t xCopyPosition ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueGenericSendImpl \n"
         "bnez tp, 1f                       \n"
         "    j MPU_xQueueGenericSendImpl   \n"
         "1:                                \n"
         "    li   a7, %0                   \n"
         "    ecall                         \n"
         "    ret                           \n"
     : : "i"(SYSTEM_CALL_xQueueGenericSend) : "memory");
 }
/*-----------------------------------------------------------*/

UBaseType_t MPU_uxQueueMessagesWaiting( const QueueHandle_t xQueue ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

UBaseType_t MPU_uxQueueMessagesWaiting( const QueueHandle_t xQueue ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxQueueMessagesWaitingImpl \n"
         "bnez tp, 1f                            \n"
         "    j MPU_uxQueueMessagesWaitingImpl   \n"
         "1:                                     \n"
         "    li   a7, %0                        \n"
         "    ecall                              \n"
         "    ret                                \n"
     : : "i"(SYSTEM_CALL_uxQueueMessagesWaiting) : "memory");
 }
/*-----------------------------------------------------------*/

UBaseType_t MPU_uxQueueSpacesAvailable( const QueueHandle_t xQueue ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

UBaseType_t MPU_uxQueueSpacesAvailable( const QueueHandle_t xQueue ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxQueueSpacesAvailableImpl \n"
         "bnez tp, 1f                            \n"
         "    j MPU_uxQueueSpacesAvailableImpl   \n"
         "1:                                     \n"
         "    li   a7, %0                        \n"
         "    ecall                              \n"
         "    ret                                \n"
     : : "i"(SYSTEM_CALL_uxQueueSpacesAvailable) : "memory");
 }
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueReceive( QueueHandle_t xQueue,
                              void * const pvBuffer,
                              TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

BaseType_t MPU_xQueueReceive( QueueHandle_t xQueue,
                              void * const pvBuffer,
                              TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueReceiveImpl \n"
         "bnez tp, 1f                   \n"
         "    j MPU_xQueueReceiveImpl   \n"
         "1:                            \n"
         "    li   a7, %0               \n"
         "    ecall                     \n"
         "    ret                       \n"
     : : "i"(SYSTEM_CALL_xQueueReceive) : "memory");
 }
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueuePeek( QueueHandle_t xQueue,
                           void * const pvBuffer,
                           TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

BaseType_t MPU_xQueuePeek( QueueHandle_t xQueue,
                           void * const pvBuffer,
                           TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueuePeekImpl \n"
         "bnez tp, 1f                \n"
         "    j MPU_xQueuePeekImpl   \n"
         "1:                         \n"
         "    li   a7, %0            \n"
         "    ecall                  \n"
         "    ret                    \n"
     : : "i"(SYSTEM_CALL_xQueuePeek) : "memory");
 }
/*-----------------------------------------------------------*/

BaseType_t MPU_xQueueSemaphoreTake( QueueHandle_t xQueue,
                                    TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

BaseType_t MPU_xQueueSemaphoreTake( QueueHandle_t xQueue,
                                    TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueSemaphoreTakeImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xQueueSemaphoreTakeImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xQueueSemaphoreTake) : "memory");
 }
/*-----------------------------------------------------------*/

#if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )

    TaskHandle_t MPU_xQueueGetMutexHolder( QueueHandle_t xSemaphore ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TaskHandle_t MPU_xQueueGetMutexHolder( QueueHandle_t xSemaphore ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueGetMutexHolderImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_xQueueGetMutexHolderImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_xQueueGetMutexHolder) : "memory");
 }

#endif /* if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t MPU_xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                             TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                             TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueTakeMutexRecursiveImpl \n"
         "bnez tp, 1f                              \n"
         "    j MPU_xQueueTakeMutexRecursiveImpl   \n"
         "1:                                       \n"
         "    li   a7, %0                          \n"
         "    ecall                                \n"
         "    ret                                  \n"
     : : "i"(SYSTEM_CALL_xQueueTakeMutexRecursive) : "memory");
 }

#endif /* if ( configUSE_RECURSIVE_MUTEXES == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_RECURSIVE_MUTEXES == 1 )

    BaseType_t MPU_xQueueGiveMutexRecursive( QueueHandle_t pxMutex ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xQueueGiveMutexRecursive( QueueHandle_t pxMutex ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueGiveMutexRecursiveImpl \n"
         "bnez tp, 1f                              \n"
         "    j MPU_xQueueGiveMutexRecursiveImpl   \n"
         "1:                                       \n"
         "    li   a7, %0                          \n"
         "    ecall                                \n"
         "    ret                                  \n"
     : : "i"(SYSTEM_CALL_xQueueGiveMutexRecursive) : "memory");
 }

#endif /* if ( configUSE_RECURSIVE_MUTEXES == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    QueueSetMemberHandle_t MPU_xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                                    const TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    QueueSetMemberHandle_t MPU_xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                                    const TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueSelectFromSetImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xQueueSelectFromSetImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xQueueSelectFromSet) : "memory");
 }

#endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_QUEUE_SETS == 1 )

    BaseType_t MPU_xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                   QueueSetHandle_t xQueueSet ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                   QueueSetHandle_t xQueueSet ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xQueueAddToSetImpl \n"
         "bnez tp, 1f                    \n"
         "    j MPU_xQueueAddToSetImpl   \n"
         "1:                             \n"
         "    li   a7, %0                \n"
         "    ecall                      \n"
         "    ret                        \n"
     : : "i"(SYSTEM_CALL_xQueueAddToSet) : "memory");
 }

#endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    void MPU_vQueueAddToRegistry( QueueHandle_t xQueue,
                                  const char * pcName ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vQueueAddToRegistry( QueueHandle_t xQueue,
                                  const char * pcName ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vQueueAddToRegistryImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_vQueueAddToRegistryImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_vQueueAddToRegistry) : "memory");
 }

#endif /* if ( configQUEUE_REGISTRY_SIZE > 0 ) */
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    void MPU_vQueueUnregisterQueue( QueueHandle_t xQueue ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vQueueUnregisterQueue( QueueHandle_t xQueue ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vQueueUnregisterQueueImpl \n"
         "bnez tp, 1f                           \n"
         "    j MPU_vQueueUnregisterQueueImpl   \n"
         "1:                                    \n"
         "    li   a7, %0                       \n"
         "    ecall                             \n"
         "    ret                               \n"
     : : "i"(SYSTEM_CALL_vQueueUnregisterQueue) : "memory");
 }

#endif /* if ( configQUEUE_REGISTRY_SIZE > 0 ) */
/*-----------------------------------------------------------*/

#if ( configQUEUE_REGISTRY_SIZE > 0 )

    const char * MPU_pcQueueGetName( QueueHandle_t xQueue ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    const char * MPU_pcQueueGetName( QueueHandle_t xQueue ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_pcQueueGetNameImpl \n"
         "bnez tp, 1f                    \n"
         "    j MPU_pcQueueGetNameImpl   \n"
         "1:                             \n"
         "    li   a7, %0                \n"
         "    ecall                      \n"
         "    ret                        \n"
     : : "i"(SYSTEM_CALL_pcQueueGetName) : "memory");
 }

#endif /* if ( configQUEUE_REGISTRY_SIZE > 0 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    void * MPU_pvTimerGetTimerID( const TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void * MPU_pvTimerGetTimerID( const TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_pvTimerGetTimerIDImpl \n"
         "bnez tp, 1f                       \n"
         "    j MPU_pvTimerGetTimerIDImpl   \n"
         "1:                                \n"
         "    li   a7, %0                   \n"
         "    ecall                         \n"
         "    ret                           \n"
     : : "i"(SYSTEM_CALL_pvTimerGetTimerID) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    void MPU_vTimerSetTimerID( TimerHandle_t xTimer,
                               void * pvNewID ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTimerSetTimerID( TimerHandle_t xTimer,
                               void * pvNewID ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTimerSetTimerIDImpl \n"
         "bnez tp, 1f                      \n"
         "    j MPU_vTimerSetTimerIDImpl   \n"
         "1:                               \n"
         "    li   a7, %0                  \n"
         "    ecall                        \n"
         "    ret                          \n"
     : : "i"(SYSTEM_CALL_vTimerSetTimerID) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    BaseType_t MPU_xTimerIsTimerActive( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTimerIsTimerActive( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerIsTimerActiveImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xTimerIsTimerActiveImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xTimerIsTimerActive) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    TaskHandle_t MPU_xTimerGetTimerDaemonTaskHandle( void ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TaskHandle_t MPU_xTimerGetTimerDaemonTaskHandle( void ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerGetTimerDaemonTaskHandleImpl \n"
         "bnez tp, 1f                                    \n"
         "    j MPU_xTimerGetTimerDaemonTaskHandleImpl   \n"
         "1:                                             \n"
         "    li   a7, %0                                \n"
         "    ecall                                      \n"
         "    ret                                        \n"
     : : "i"(SYSTEM_CALL_xTimerGetTimerDaemonTaskHandle) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    BaseType_t MPU_xTimerGenericCommandFromTaskEntry( const xTimerGenericCommandFromTaskParams_t * pxParams ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTimerGenericCommandFromTaskEntry( const xTimerGenericCommandFromTaskParams_t * pxParams ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerGenericCommandFromTaskImpl \n"
         "bnez tp, 1f                                       \n"
         "    j MPU_xTimerGenericCommandFromTaskImpl   \n"
         "1:                                                \n"
         "    li   a7, %0                                   \n"
         "    ecall                                         \n"
         "    ret                                           \n"
     : : "i"(SYSTEM_CALL_xTimerGenericCommandFromTask) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    const char * MPU_pcTimerGetName( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    const char * MPU_pcTimerGetName( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_pcTimerGetNameImpl \n"
         "bnez tp, 1f                    \n"
         "    j MPU_pcTimerGetNameImpl   \n"
         "1:                             \n"
         "    li   a7, %0                \n"
         "    ecall                      \n"
         "    ret                        \n"
     : : "i"(SYSTEM_CALL_pcTimerGetName) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    void MPU_vTimerSetReloadMode( TimerHandle_t xTimer,
                                  const BaseType_t xAutoReload ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vTimerSetReloadMode( TimerHandle_t xTimer,
                                  const BaseType_t xAutoReload ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vTimerSetReloadModeImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_vTimerSetReloadModeImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_vTimerSetReloadMode) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    BaseType_t MPU_xTimerGetReloadMode( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xTimerGetReloadMode( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerGetReloadModeImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xTimerGetReloadModeImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xTimerGetReloadMode) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    UBaseType_t MPU_uxTimerGetReloadMode( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    UBaseType_t MPU_uxTimerGetReloadMode( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxTimerGetReloadModeImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_uxTimerGetReloadModeImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_uxTimerGetReloadMode) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    TickType_t MPU_xTimerGetPeriod( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TickType_t MPU_xTimerGetPeriod( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerGetPeriodImpl \n"
         "bnez tp, 1f                     \n"
         "    j MPU_xTimerGetPeriodImpl   \n"
         "1:                              \n"
         "    li   a7, %0                 \n"
         "    ecall                       \n"
         "    ret                         \n"
     : : "i"(SYSTEM_CALL_xTimerGetPeriod) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_TIMERS == 1 )

    TickType_t MPU_xTimerGetExpiryTime( TimerHandle_t xTimer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    TickType_t MPU_xTimerGetExpiryTime( TimerHandle_t xTimer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xTimerGetExpiryTimeImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xTimerGetExpiryTimeImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xTimerGetExpiryTime) : "memory");
 }

#endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_EVENT_GROUPS == 1 )

    EventBits_t MPU_xEventGroupWaitBitsEntry( const xEventGroupWaitBitsParams_t * pxParams ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    EventBits_t MPU_xEventGroupWaitBitsEntry( const xEventGroupWaitBitsParams_t * pxParams ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xEventGroupWaitBitsImpl \n"
         "bnez tp, 1f                              \n"
         "    j MPU_xEventGroupWaitBitsImpl   \n"
         "1:                                       \n"
         "    li   a7, %0                          \n"
         "    ecall                                \n"
         "    ret                                  \n"
     : : "i"(SYSTEM_CALL_xEventGroupWaitBits) : "memory");
 }

#endif /* #if ( configUSE_EVENT_GROUPS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_EVENT_GROUPS == 1 )

    EventBits_t MPU_xEventGroupClearBits( EventGroupHandle_t xEventGroup,
                                          const EventBits_t uxBitsToClear ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    EventBits_t MPU_xEventGroupClearBits( EventGroupHandle_t xEventGroup,
                                          const EventBits_t uxBitsToClear ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xEventGroupClearBitsImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_xEventGroupClearBitsImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_xEventGroupClearBits) : "memory");
 }

#endif /* #if ( configUSE_EVENT_GROUPS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_EVENT_GROUPS == 1 )

    EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                        const EventBits_t uxBitsToSet ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                        const EventBits_t uxBitsToSet ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xEventGroupSetBitsImpl \n"
         "bnez tp, 1f                        \n"
         "    j MPU_xEventGroupSetBitsImpl   \n"
         "1:                                 \n"
         "    li   a7, %0                    \n"
         "    ecall                          \n"
         "    ret                            \n"
     : : "i"(SYSTEM_CALL_xEventGroupSetBits) : "memory");
 }

#endif /* #if ( configUSE_EVENT_GROUPS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_EVENT_GROUPS == 1 )

    EventBits_t MPU_xEventGroupSync( EventGroupHandle_t xEventGroup,
                                     const EventBits_t uxBitsToSet,
                                     const EventBits_t uxBitsToWaitFor,
                                     TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    EventBits_t MPU_xEventGroupSync( EventGroupHandle_t xEventGroup,
                                     const EventBits_t uxBitsToSet,
                                     const EventBits_t uxBitsToWaitFor,
                                     TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xEventGroupSyncImpl \n"
         "bnez tp, 1f                     \n"
         "    j MPU_xEventGroupSyncImpl   \n"
         "1:                              \n"
         "    li   a7, %0                 \n"
         "    ecall                       \n"
         "    ret                         \n"
     : : "i"(SYSTEM_CALL_xEventGroupSync) : "memory");
 }

#endif /* #if ( configUSE_EVENT_GROUPS == 1 ) */
/*-----------------------------------------------------------*/

#if ( ( configUSE_EVENT_GROUPS == 1 ) && ( configUSE_TRACE_FACILITY == 1 ) )

    UBaseType_t MPU_uxEventGroupGetNumber( void * xEventGroup ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    UBaseType_t MPU_uxEventGroupGetNumber( void * xEventGroup ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_uxEventGroupGetNumberImpl \n"
         "bnez tp, 1f                           \n"
         "    j MPU_uxEventGroupGetNumberImpl   \n"
         "1:                                    \n"
         "    li   a7, %0                       \n"
         "    ecall                             \n"
         "    ret                               \n"
     : : "i"(SYSTEM_CALL_uxEventGroupGetNumber) : "memory");
 }

#endif /* #if ( ( configUSE_EVENT_GROUPS == 1 ) && ( configUSE_TRACE_FACILITY == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( ( configUSE_EVENT_GROUPS == 1 ) && ( configUSE_TRACE_FACILITY == 1 ) )

    void MPU_vEventGroupSetNumber( void * xEventGroup,
                                   UBaseType_t uxEventGroupNumber ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    void MPU_vEventGroupSetNumber( void * xEventGroup,
                                   UBaseType_t uxEventGroupNumber ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_vEventGroupSetNumberImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_vEventGroupSetNumberImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_vEventGroupSetNumber) : "memory");
 }

#endif /* #if ( ( configUSE_EVENT_GROUPS == 1 ) && ( configUSE_TRACE_FACILITY == 1 ) ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    size_t MPU_xStreamBufferSend( StreamBufferHandle_t xStreamBuffer,
                                  const void * pvTxData,
                                  size_t xDataLengthBytes,
                                  TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    size_t MPU_xStreamBufferSend( StreamBufferHandle_t xStreamBuffer,
                                  const void * pvTxData,
                                  size_t xDataLengthBytes,
                                  TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferSendImpl \n"
         "bnez tp, 1f                       \n"
         "    j MPU_xStreamBufferSendImpl   \n"
         "1:                                \n"
         "    li   a7, %0                   \n"
         "    ecall                         \n"
         "    ret                           \n"
     : : "i"(SYSTEM_CALL_xStreamBufferSend) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    size_t MPU_xStreamBufferReceive( StreamBufferHandle_t xStreamBuffer,
                                     void * pvRxData,
                                     size_t xBufferLengthBytes,
                                     TickType_t xTicksToWait ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    size_t MPU_xStreamBufferReceive( StreamBufferHandle_t xStreamBuffer,
                                     void * pvRxData,
                                     size_t xBufferLengthBytes,
                                     TickType_t xTicksToWait ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferReceiveImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_xStreamBufferReceiveImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_xStreamBufferReceive) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    BaseType_t MPU_xStreamBufferIsFull( StreamBufferHandle_t xStreamBuffer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xStreamBufferIsFull( StreamBufferHandle_t xStreamBuffer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferIsFullImpl \n"
         "bnez tp, 1f                         \n"
         "    j MPU_xStreamBufferIsFullImpl   \n"
         "1:                                  \n"
         "    li   a7, %0                     \n"
         "    ecall                           \n"
         "    ret                             \n"
     : : "i"(SYSTEM_CALL_xStreamBufferIsFull) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    BaseType_t MPU_xStreamBufferIsEmpty( StreamBufferHandle_t xStreamBuffer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xStreamBufferIsEmpty( StreamBufferHandle_t xStreamBuffer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferIsEmptyImpl \n"
         "bnez tp, 1f                          \n"
         "    j MPU_xStreamBufferIsEmptyImpl   \n"
         "1:                                   \n"
         "    li   a7, %0                      \n"
         "    ecall                            \n"
         "    ret                              \n"
     : : "i"(SYSTEM_CALL_xStreamBufferIsEmpty) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    size_t MPU_xStreamBufferSpacesAvailable( StreamBufferHandle_t xStreamBuffer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    size_t MPU_xStreamBufferSpacesAvailable( StreamBufferHandle_t xStreamBuffer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferSpacesAvailableImpl \n"
         "bnez tp, 1f                                  \n"
         "    j MPU_xStreamBufferSpacesAvailableImpl   \n"
         "1:                                           \n"
         "    li   a7, %0                              \n"
         "    ecall                                    \n"
         "    ret                                      \n"
     : : "i"(SYSTEM_CALL_xStreamBufferSpacesAvailable) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    size_t MPU_xStreamBufferBytesAvailable( StreamBufferHandle_t xStreamBuffer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    size_t MPU_xStreamBufferBytesAvailable( StreamBufferHandle_t xStreamBuffer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferBytesAvailableImpl \n"
         "bnez tp, 1f                                 \n"
         "    j MPU_xStreamBufferBytesAvailableImpl   \n"
         "1:                                          \n"
         "    li   a7, %0                             \n"
         "    ecall                                   \n"
         "    ret                                     \n"
     : : "i"(SYSTEM_CALL_xStreamBufferBytesAvailable) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    BaseType_t MPU_xStreamBufferSetTriggerLevel( StreamBufferHandle_t xStreamBuffer,
                                                 size_t xTriggerLevel ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    BaseType_t MPU_xStreamBufferSetTriggerLevel( StreamBufferHandle_t xStreamBuffer,
                                                 size_t xTriggerLevel ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferSetTriggerLevelImpl \n"
         "bnez tp, 1f                                  \n"
         "    j MPU_xStreamBufferSetTriggerLevelImpl   \n"
         "1:                                           \n"
         "    li   a7, %0                              \n"
         "    ecall                                    \n"
         "    ret                                      \n"
     : : "i"(SYSTEM_CALL_xStreamBufferSetTriggerLevel) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

#if ( configUSE_STREAM_BUFFERS == 1 )

    size_t MPU_xStreamBufferNextMessageLengthBytes( StreamBufferHandle_t xStreamBuffer ) __attribute__( ( naked ) ) FREERTOS_SYSTEM_CALL;

    size_t MPU_xStreamBufferNextMessageLengthBytes( StreamBufferHandle_t xStreamBuffer ) /* __attribute__ (( naked )) FREERTOS_SYSTEM_CALL */
 {
     __asm volatile (
         ".extern MPU_xStreamBufferNextMessageLengthBytesImpl \n"
         "bnez tp, 1f                                         \n"
         "    j MPU_xStreamBufferNextMessageLengthBytesImpl   \n"
         "1:                                                  \n"
         "    li   a7, %0                                     \n"
         "    ecall                                           \n"
         "    ret                                             \n"
     : : "i"(SYSTEM_CALL_xStreamBufferNextMessageLengthBytes) : "memory");
 }

#endif /* #if ( configUSE_STREAM_BUFFERS == 1 ) */
/*-----------------------------------------------------------*/

