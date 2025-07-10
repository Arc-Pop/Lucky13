#include "console_task.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "line_task.h"
#include "motor_task.h"

extern double percentLeft;
extern double percentRight;

//***************************************************************************//
//VARIABLES FOR TUNING CAR
int thinTime=10;                                                                // Time it takes to cross thin line. Adjust for calibration.
int thickTime= 350;                                                         // Time it takes to cross thick line.
//***************************************************************************//

#define LINETASKSTACKSIZE        128
#define PRIORITY_LINE_TASK       3

xSemaphoreHandle line_int;
extern int collectData;
extern double percentLeft;
extern double percentRight;

extern void Blue_LED_On();
extern void Green_LED_On();
extern void Red_LED_On();
extern void Test();
extern void Stop();


uint32_t startTimePB0 = 0;
uint32_t thinCount = 0;
uint32_t thinLine = 0;
uint32_t thickLine = 0;
extern uint32_t turnTime;
uint32_t timePB0 = 0;


static void
LineTask(void *pvParameters){                                                        // Begin, task for detecting width of tape line.
         while(1){
                 startTimePB0=0;
                 while(startTimePB0==0){
                 vTaskDelay(1);
                 GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);                 // Set PB0 as output to charge sensor capacitor.
                 GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);              // Turn the sensor module pins on so that they charge their capacitors.
                 TickType_t xLineTime = xTaskGetTickCount();
                 vTaskDelayUntil(&xLineTime, 1 );                                // Delay 1 [ms] for capacitor to charge.
                 GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);                  // Change the sensor module connections to input pins.
                 SysCtlDelay((SysCtlClockGet() / (3 * 10000))*6);                    // Wait for the capacitor to discharge partially [800 us]. *CALIBRATE*
                 if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0)){                       // Begin, Check if the line sensor module pins are high. When they are low LineISR occurs.
                     if(startTimePB0==0){                                            // Begin, If the timer for the tape width hasn't started.
                         startTimePB0 = xTaskGetTickCount();                         // Start the right line sensor (PB0) timer
                     }                                                               // End, If the timer for the tape width hasn't started.
                 }
                 }
                 timePB0=0;
                 while(timePB0==0){
                 vTaskDelay(1);
                 GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);                 // Set PB0 as output to charge sensor capacitor.
                 GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);              // Turn the sensor module pins on so that they charge their capacitors.
                 TickType_t xLineTime = xTaskGetTickCount();
                 xLineTime = xTaskGetTickCount();
                 vTaskDelayUntil(&xLineTime, 1 );                                // Delay 1 [ms] for capacitor to charge.
                 GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);                  // Change the sensor module connections to input pins.
                 SysCtlDelay((SysCtlClockGet() / (3 * 10000))*6);                    // Wait for the capacitor to discharge partially [800 us]. *CALIBRATE*
                 if(!GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0)){                       // Begin, Check if the line sensor module pins are high. When they are low LineISR occurs.
                     if(timePB0==0){                                            // Begin, If the timer for the tape width hasn't started.
                         timePB0 = xTaskGetTickCount();                         // Start the right line sensor (PB0) timer
                     }                                                               // End, If the timer for the tape width hasn't started.
                 }
              }
                 timePB0 = timePB0 - startTimePB0;

                 if(timePB0>( 300 - percentRight - percentLeft)){
                 Red_LED_On();
                 Stop();
                 }                                                                          // Both sensors cross thick line, turn on the red LED.
                 else if(timePB0>5){                                                 // Begin, Both sensors cross thin line
                     if(thinLine==2){ thinLine=0; }
                     thinLine++;                                                            // Add one to the thin line count.
                     switch(thinLine){                                                      // Begin, Change LED based on thin line count.
                         case(1):
                            Green_LED_On();
                            collectData = 1;  // Start data collection
                            break;                                        // Crossing 1 thin line turns on the green LED.
                         case(2):
                            Blue_LED_On();
                            collectData = 0;  // Start data collection
                            break;                                         // Crossing 2 thin lines turn on the blue LED.
                         default:
                             break;                                            // Reset the thin line count to 0 to prevent overflow.
                         }                                                                      // End, Change LED based on thin line count.
                 }                                                                          // End, Both sensors cross thin line
                 vTaskDelay(10);
             }                                                                          // End, If both timers have already been started.
        }                                                                              // End, function for detecting width of tape line. Occurs when the line sensor is over white.



uint32_t
LineTaskInit(void)
{                                                                                   // Begin, Function for initializing clock and pins for line sensor.
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                               // Enable the port for the line sensors (PB0 left, PB7 right)

         vSemaphoreCreateBinary(line_int);

         if(xTaskCreate(LineTask, (const portCHAR *)"Line",
                        LINETASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                        PRIORITY_LINE_TASK, NULL) != pdTRUE)
         {
             return 1;
         }

         return 0;
}


