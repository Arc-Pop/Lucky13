/******************************************************************************
ECE4437 Embedded Microcomputer Systems
Fall 2024

Team #13:
Arc, Jonathan
Benavidas, David
Bodehousse Houndekon

This the code for Team #13's maze car.

Linux Terminal Commands to connect Bluetooth module:
sudo rfcomm bind 98:D3:41:F7:18:11 HC-05
sudo rfcomm bind 0 98:D3:41:F7:18:11 1
******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "console_task.h"
#include "distance_task.h"
#include "line_task.h"
#include "motor_task.h"
#include "switch_task.h"

extern void Test();

#ifdef DEBUG                                                                   // The error routine that is called if the driver library encounters an error.
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)            // This hook is called by FreeRTOS when an stack overflow error is detected.
{                                                                               // This function can not return, so loop forever.  Interrupts are disabled
    while(1){  }                                                                  // on entry to this function, so no processor interrupts will interrupt
}                                                                               // this loop.


int
main(void)
{
    FPUEnable();                                                       // Floating-point unit enable
    FPULazyStackingEnable();                                           // Automatically save floating-point data to stack

//    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |   // Set the clocking to run at 50 MHz from the PLL.
//                       SYSCTL_OSC_MAIN);
    SysCtlClockSet                                                      // Set the clocking to run directly from the crystal.
      (SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


    if(ConsoleTaskInit() != 0){ while(1){} }                                    // Initialize console task for UART1 Bluetooth serial console. 9600 baud rate.
//    if(DistanceTaskInit() != 0){ while(1){} }                                   // Initialize distance task for front and right distance sensors. ADC
//    if(LineTaskInit() != 0){ while(1){} }                                       // Initialize line task for bottom reflectivity sensor.
//   if(MotorTaskInit() != 0){ while(1){} }                                      // Initialize motor task for controlling motor speed. PWM and PID
//    if(SwitchTaskInit() != 0){while(1){} }

    vTaskStartScheduler();                                                      // Start the scheduler.  This should not return.

    while(1)                                                                    // In case the scheduler returns for some reason, print an error and loop
    {                                                                           // forever.
    }
}
