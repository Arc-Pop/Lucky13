#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "switch_task.h"

#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "console_task.h"

extern void Stop();
extern void Start();

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define SWITCHTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to LEDTask.
//
//*****************************************************************************
static void
SwitchTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32SwitchDelay = 25;
    uint8_t ui8CurButtonState, ui8PrevButtonState;
    ui8CurButtonState = ui8PrevButtonState = 0;
    ui16LastTime = xTaskGetTickCount();                                         // Get the current tick count.
    while(1)                                                                    // Loop forever.
    {
        ui8CurButtonState = ButtonsPoll(0, 0);                                  // Poll the debounced state of the buttons.
        if(ui8CurButtonState != ui8PrevButtonState)                             // Check if previous debounced state is equal to the current state.
        {
            ui8PrevButtonState = ui8CurButtonState;
            if((ui8CurButtonState & ALL_BUTTONS) != 0)                          // Check to make sure the change in state is due to button press
            {                                                                   // and not due to button release.
                if((ui8CurButtonState & ALL_BUTTONS) == LEFT_BUTTON)            // If left button was pressed.
                {
                    Start();                                                    // Start the car.
                }
                else if((ui8CurButtonState & ALL_BUTTONS) == RIGHT_BUTTON)      // If the right button is pressed.
                {
                    Stop();                                                     // Stop the car.
                }
            }
        }
        vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);     // Wait for the required amount of time to check back.
    }
}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t
SwitchTaskInit(void)
{
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;                       // Unlock the GPIO LOCK register for Right button to work.
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

    ButtonsInit();                                                              // Initialize the buttons

    if(xTaskCreate(SwitchTask, (const portCHAR *)"Switch",                      // Create the switch task.
                   SWITCHTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_SWITCH_TASK, NULL) != pdTRUE)
    { return(1);}                                                               // Failure.

    return(0);                                                                  // Success.
}
