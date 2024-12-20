// FreeRTOS Configuration

#ifdef __cplusplus
extern "C" {
#endif

// Inclusion Lock
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Configuration Definitions
#define configUSE_PREEMPTION                1
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 80000000 )
#define configTICK_RATE_HZ                  ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                ( 16 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 130 )
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 20000 ) )
#define configMAX_TASK_NAME_LEN             ( 10 )
#define configUSE_TRACE_FACILITY            1
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             0
#define configUSE_MUTEXES                   1
#define configQUEUE_REGISTRY_SIZE           8
#define configCHECK_FOR_STACK_OVERFLOW      2
#define configUSE_RECURSIVE_MUTEXES         1
#define configUSE_CO_ROUTINES               0
#define configUSE_TASK_NOTIFICATIONS        1
#define configSUPPORT_DYNAMIC_ALLOCATION    1

#define configUSE_MALLOC_FAILED_HOOK        0
#define configUSE_APPLICATION_TASK_TAG      0
#define configUSE_COUNTING_SEMAPHORES       1
#define configGENERATE_RUN_TIME_STATS       0

// Software Timer Definitions
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( 2 )
#define configTIMER_QUEUE_LENGTH            5
#define configTIMER_TASK_STACK_DEPTH        ( configMINIMAL_STACK_SIZE * 2 )

// Set the following definitions to 1 to include the API function (0 to exclude)
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xEventGroupSetBitsFromISR   1

// TM4C123x Specific Definitions
#define configPRIO_BITS                     3

// Interrupt Priority Definitions
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         0x07
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

// Kernel Port Layer Interrupt Definitions
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

// Assert Definitions
#define configASSERT( x ) if( ( x ) == 0 ) \
    { taskDISABLE_INTERRUPTS(); for( ;; ); }

#ifdef __cplusplus
}
#endif

#endif // FREERTOS_CONFIG_H
