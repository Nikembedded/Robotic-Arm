//----------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : MAIN.CPP
// FILE VERSION : 1.0
// PROGRAMMER   : Nikhil Premjani
//----------------------------------------------------------------------------
// REVISION HISTORY
//----------------------------------------------------------------------------
//
// 1.0, 2024-06-07, Nikhil Premjani
//   - Initial release
//
//----------------------------------------------------------------------------
// MODULE DESCRIPTION
//----------------------------------------------------------------------------
//
// Main application entry point.
//
//----------------------------------------------------------------------------
// INCLUDE FILES
//----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"

#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"

#include "tasks/exio.h"
#include "tasks/sercom.h"
#include "tasks/usbcom.h"
#include "tasks/manip.h"
#include "tasks/heartbeat.h"

//----------------------------------------------------------------------------
// FUNCTION : vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
// PURPOSE  : Called by FreeRTOS when an stack overflow error is detected
//----------------------------------------------------------------------------

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    // Loop for debugging
    while( 1 );
}

//----------------------------------------------------------------------------
// FUNCTION : main( void )
// PURPOSE  : Application entry point
//----------------------------------------------------------------------------

void main( void )
{
    // Set the system clock to run at 80 MHz from the PLL
    SysCtlClockSet( SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN );

    // Create all tasks
    if( EXIO_Init()      ) while( 1 ); // External I/O Task
    if( SERCOM_Init()    ) while( 1 ); // Serial Communication Task
    if( USBCOM_Init()    ) while( 1 ); // USB Communication Task
    if( MANIP_Init()     ) while( 1 ); // Manipulator Task
    if( HEARTBEAT_Init() ) while( 1 ); // Heartbeat Task

    // Start the scheduler
    vTaskStartScheduler();

    // Error - if the scheduler returns for some reason, loop for debug
    while( 1 );
}

//----------------------------------------------------------------------------
// END MAIN.CPP
//----------------------------------------------------------------------------
