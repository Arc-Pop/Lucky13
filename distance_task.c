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
#include "motor_task.h"
#include "distance_task.h"

//****************************************************************************//
//VARIABLES FOR TUNING CAR
double setpoint = 8;                                                            // Desired distance from the wall (10 cm)
double frontDistanceLimit=10;
double rightTurnSpeed = 32;
uint32_t RightTurnTime = 750;
uint32_t UTurnTime = 100;
//****************************************************************************//

#define DISTANCETASKSTACKSIZE        256
#define PRIORITY_DISTANCE_TASK       2


extern void AdjustRobotSpeed();
extern void Blue_LED_On();
extern void Green_LED_On();
extern void LED_Off();
extern double PIDControl();
extern void PulseWidthSet();
extern void RightTurn();
extern void Stop();
extern void Test();
extern void UTurn();


extern xSemaphoreHandle     adjust_speed;
uint32_t                    ADCVoltage[2];                                      // Voltage reading from the ADC. frontDistanceIntHandler()
double                      frontDistance = 0;
xSemaphoreHandle            front_distance_int;
extern double               max;
extern double               percentLeft;
extern double               percentRight;
double                      rightDistance = 0;
xSemaphoreHandle            right_distance_int;
extern int start;
extern uint32_t             turnTime;
extern xSemaphoreHandle     uart_int;
uint32_t                    UTurnStatus = 0;
double                      volts;


double GetFrontDistance(){
    ADCProcessorTrigger(ADC0_BASE, 3);                                          // Trigger ADC conversion
    ADCSequenceDataGet(ADC0_BASE, 3, &ADCVoltage[0]);                           // Read the ADC value
    volts = ADCVoltage[0] * (3.3 / 4096);                                       // Convert to voltage
    frontDistance = 13.68 / volts;                                              // Convert to distance [cm]'
    return frontDistance;
}


static void
DistanceTask(void *pvParameters)
{
         while(1){


                 GetFrontDistance();

         if(frontDistance<10) {                                  // Less than 8 cm, stop and initiate U-turn
  //           if(xSemaphoreTake(adjust_speed, 10)){
//            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
//            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
//            percentRight=max;
//            percentLeft=max;
//            UARTprintf("frontDistance: %i\n", (int)frontDistance);
//            PulseWidthSet();
//            TickType_t xLastWakeTime = xTaskGetTickCount();
//            vTaskDelayUntil(&xLastWakeTime, UTurnTime );
//            UARTprintf("rightDistance: %i\n", (int)rightDistance);
/*
            while(GetFrontDistance()<10){

                xLastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&xLastWakeTime, 10);                                // Delay 1 [ms] for capacitor to charge.
            }
*/
 //           xLastWakeTime = xTaskGetTickCount();
//            vTaskDelayUntil(&xLastWakeTime, UTurnTime );


//            if(frontDistance>frontDistanceLimit){
//            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
//            }
//            xSemaphoreGive(adjust_speed);
//            }
//            }

//         ADCProcessorTrigger(ADC1_BASE, 3);                                         // Trigger ADC conversion
//         ADCSequenceDataGet(ADC1_BASE, 3, &ADCVoltage[1]);                          // Read the ADC value
//         volts = ADCVoltage[1] * (3.3 / 4096);                                      // Convert to voltage
//         rightDistance = 13.68 / volts;                                             // Convert to distance [cm]

//         if(rightDistance>18){

//         if(xSemaphoreTake(adjust_speed, 10)){


//         percentLeft=max;
//         percentRight = rightTurnSpeed;
//         PulseWidthSet();
//         vTaskDelay(RightTurnTime);


//         while(rightDistance>18){
//             ADCProcessorTrigger(ADC1_BASE, 3);                                         // Trigger ADC conversion
//             ADCSequenceDataGet(ADC1_BASE, 3, &ADCVoltage[1]);                          // Read the ADC value
//             volts = ADCVoltage[1] * (3.3 / 4096);                                      // Convert to voltage
//             rightDistance = 13.68 / volts;                                             // Convert to distance [cm]
//             vTaskDelay(1);
//             }

 //        xSemaphoreGive(adjust_speed);
 //        }
 //        }

//         if(start==3){

  //       double speed_adjustment = PIDControl(rightDistance, setpoint);
  //       AdjustRobotSpeed(speed_adjustment);                             // Base speed is 50%, adjusted by PID
         }
         vTaskDelay(10);

         }

         }                                                                              // End, Function to trigger ADC for right sensor and return the distance

uint32_t
DistanceTaskInit(void) {                                                          // Begin, function for initializing the ADC to ADC0, Sequence 3, using PE0.

         SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                               // Enable the Port E set of GPIO pins for sensor.
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                               // Enable the Port F set of GPIO pins for LED.
         GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);  // Set the built-in LED pins as output pins.
         GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_5);                    // Set the pin for ADC: GPIO_PIN_3 for PE0, GPIO_PIN_5 for PE5
         ADCSequenceDisable(ADC0_BASE, 3);                                          // Disable the sequence (sequence 3) during configuration.
         ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);              // Set the sequence to be interrupted by the processor. (ADC_TRIGGER_PROCESSOR)
         ADCSequenceStepConfigure(ADC0_BASE, 3, 0,                                  // Configure the step channel: ADC_CTL_CH3 for PE0,  ADC_CTL_CH8 for PE5
                            ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);
         ADCSequenceEnable(ADC0_BASE, 3);                                           // Enable the sequence to prepare for sample.

         ADCSequenceDisable(ADC1_BASE, 3);                                          // Disable the sequence (sequence 3) during configuration.
         ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);              // Set the sequence to be interrupted by the processor. (ADC_TRIGGER_PROCESSOR)
         ADCSequenceStepConfigure(ADC1_BASE, 3, 0,
                            ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH8);                // Configure the step channel: ADC_CTL_CH3 for PE0,  ADC_CTL_CH8 for PE5
         ADCSequenceEnable(ADC1_BASE, 3);                                           // Enable the sequence to prepare for sample.

         vSemaphoreCreateBinary(front_distance_int);
         vSemaphoreCreateBinary(right_distance_int);

         if(xTaskCreate(DistanceTask, (const portCHAR *)"Distance",
                        DISTANCETASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                        PRIORITY_DISTANCE_TASK, NULL) != pdTRUE)
         {
             return 1;
         }

         return 0;
}
