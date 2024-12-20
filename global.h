//----------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : GLOBAL.H
// FILE VERSION : 1.0
// PROGRAMMER   : Nikhil Premjani
//----------------------------------------------------------------------------
// REVISION HISTORY
//----------------------------------------------------------------------------
//
// 1.0, 2024-06-06, Nikhil Premjani
//   - Initial release
//
//----------------------------------------------------------------------------
// INCLUSION LOCK
//----------------------------------------------------------------------------

#ifndef GLOBAL_H_
#define GLOBAL_H_

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------
// INCLUDE FILES
//----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"

#include "pin_map.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

//----------------------------------------------------------------------------
// CONSTANTS
//----------------------------------------------------------------------------

#define STACK_SIZE_TASK_SERCOM      256     // Stack size in words
#define STACK_SIZE_TASK_EXIO        128     // Stack size in words
#define STACK_SIZE_TASK_USB         128     // Stack size in words
#define STACK_SIZE_TASK_MANIP       256     // Stack size in words
#define STACK_SIZE_TASK_HEARTBEAT   128     // Stack size in words

#define TASK_PRIORITY_SERCOM        1
#define TASK_PRIORITY_EXIO          1
#define TASK_PRIORITY_USB           1
#define TASK_PRIORITY_MANIP         1
#define TASK_PRIORITY_HEARTBEAT     1

#ifdef __cplusplus
}
#endif

#endif // GLOBAL_H_

//----------------------------------------------------------------------------
// END GLOBAL.H
//----------------------------------------------------------------------------
