#include <stdio.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
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
#include "console_task.h"
#include "motor_task.h"

//***************************************************************************//
//VARIABLES FOR TUNING CAR
//double kp = 13, ki = .12, kd = 75;
double kp = .9, ki = .002, kd = 39;
double max=100;
double min=1;
double tolerance=0.5;
//***************************************************************************//

#define MOTORTASKSTACKSIZE        128
#define PRIORITY_MOTOR_TASK       1
#define BUFFER_SIZE 20

xSemaphoreHandle adjust_speed;

extern void Faster(void);                                                       // Move the robot faster.
extern void Green_LED_On(void);
extern void Yellow_LED_On(void);
extern void Red_LED_On(void);
extern void Blue_LED_On(void);
extern void LED_Off(void);
extern void Test();
extern void Forward();
extern void Backward();
extern void Start();

void acquireData(double error);

int bufferIndex = 0;
int collectData = 0;                                                            // Control data collection. 0 = don't collect, 1 = collect
double currentError=0;
double distanceError;
extern double frontDistance;
double integral = 0.0;
double lastError=0;
double Load;                                                                  // Number of clock cycles at which the pulse repeats for LED strobing.
double percentLeft = 1;                                                         // percentage of the period.frontDistanceS
double percentRight = 1;
int pingBuffer[BUFFER_SIZE];
int pidCounter = 0;  // Add this counter
int pongBuffer[BUFFER_SIZE];
double previous_error = 0.0;
uint32_t pulsePercent = 1;                                                      // percentage of the period.
uint32_t pulseWidth;                                                            // Number of clock cycles the pulse stays on for LED strobing.
uint32_t PWM_clock;                                                             // Clock speed of the pulse width modulation for LED strobing.
uint32_t PWM_freq;                                                              // Frequency of the pulse width modulation for LED strobing.
int start=0;
extern uint32_t thinLine;                                                       // percentage of the period.
int *currentBuffer = pingBuffer;
int *nextBuffer = pongBuffer;


     void PulseWidthSet(void){                                                      // Begin, Function for setting the pulse width to using a percentage variable.
         if(start==1 || start==3){
             pulseWidth = (Load / 100) * percentLeft;                                  // Number of PWM cycles to have the pulse on as a percentage of the period.
             if(pulseWidth>0){PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pulseWidth);}                        // Set the pulse width
             pulseWidth = (Load / 100) * percentRight;                                  // Number of PWM cycles to have the pulse on as a percentage of the period.
             if(pulseWidth>0){PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, pulseWidth);}                        // Set the pulse width
         }
         else{
             pulseWidth=1;
             PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pulseWidth);
             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, pulseWidth);
         }
     }    // End, Function for setting the pulse width to using a percentage variable.


     double PIDControl(double distance, double setpoint) {

         if(start==1 || start==3){


             if(frontDistance>8){
                 int i;
                 currentError = setpoint - distance;
                 lastError = previous_error;
                 integral += currentError;
                 double derivative = currentError - previous_error;
                 previous_error = currentError;

            if(collectData == 1) {
                pidCounter++;
                if (pidCounter == 1) {  // Only acquire data every second call
                    pidCounter = 0;
                    acquireData(currentError);  // Call data acquisition logic
                }
            }

         if (thinLine == 2) {
             //char sampleStr[4];
             // Transmit remaining data from the current buffer (currentBuffer)
             for (i = 0; i < bufferIndex; i++) {
                 UARTprintf("%i ", currentBuffer[i]);

             }

             if(bufferIndex==BUFFER_SIZE){ UARTprintf("\r\n"); }
             bufferIndex = 0; //Empty buffer
         }

//         double testError = (kp * currentError) + (ki * integral) + (kd * derivative);
//         char errorString[20];
//         sprintf(&errorString,"testError = %d \n",testError);


         return (kp * currentError) + (ki * integral) + (kd * derivative);
         }
     }
         return 0;
     }


     void AdjustRobotSpeed(double PIDresult) {                             // Begin, Function to calibrate the speed for each motor
         if(xSemaphoreTake(adjust_speed, 10)){
             if(abs(PIDresult)<tolerance){Faster();}
             if((percentRight+PIDresult)<(max+1)
                     && (percentRight+PIDresult)>min){percentRight += PIDresult;}
             else{
                 if((percentRight+PIDresult)>max){percentRight=max;}
                 else if((percentRight+PIDresult)<min){percentRight=min;}
             }
             if((percentLeft-PIDresult)<(max+1)
                     && (percentLeft-PIDresult)>min){percentLeft -= PIDresult;}
             else{
                 if((percentLeft-PIDresult)>max){percentLeft=max;}
                 else if((percentLeft-PIDresult)<min){percentLeft=min;}
             }
             PulseWidthSet();                                                           // Update PWM for the motors.
             vTaskDelay(1);
             xSemaphoreGive(adjust_speed);

             }
     }                                                              // End, Function to calibrate the speed for each motor


     void transmitData() {
         int i;
         for (i = 0; i < BUFFER_SIZE; i++) {UARTprintf("%i ", nextBuffer[i]);}      // Send data from the full buffer (nextBuffer)
         UARTprintf("\n");                                                          // Send Carriage Return and Linefeed to signify end of frame
     }


     void swapBuffers() {
         if (currentBuffer == pingBuffer) {
             currentBuffer = pongBuffer;
             nextBuffer = pingBuffer;
         } else {
             currentBuffer = pingBuffer;
             nextBuffer = pongBuffer;
         }
     }


     void acquireData(double error) {
         if (bufferIndex < BUFFER_SIZE) {currentBuffer[bufferIndex++] = abs((int)(error * 10));}                       // Multiply by 100 to convert to millimeters
          else {
              bufferIndex = 0;
              swapBuffers();  // Switches between ping and pong
              transmitData(); // Sends data to PC
         }
     }



     void Strobe(void){

         while(true){                                                          // Prevents the program from ending.

             Red_LED_On();
             while(percentLeft<=100){ percentLeft++;
                                  PulseWidthSet();
                                  vTaskDelay(10);
                         }

             Green_LED_On();

             while(percentLeft>1){ percentLeft--;
                                  PulseWidthSet();
                                  vTaskDelay(10);
                                  }
             Blue_LED_On();

             while(percentRight<=100){ percentRight++;
                                 PulseWidthSet();
                                 vTaskDelay(10);
                                 }
             LED_Off();
             while(percentRight>1){ percentRight--;
                                 PulseWidthSet();
                                 vTaskDelay(10);
                                 }

//             PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);

         }
     }


static void
MotorTask(void *pvParameters)
{

}




uint32_t
MotorTaskInit(void)
{                                                                                    // Begin function for initilizing the pulse width modulation used for motors.


          SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                                       // Sets the PWM clock by dividing the system clock by 64. Could be 1, 2, 4, 8, 16, 32, 64
          SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);                                //
          while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){};                       // Waits for the PWM to be ready when it will return a 1.
          SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
          while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)){};                       // Waits for the PWM to be ready when it will return a 1.
          SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
          while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){};                      // Waits for the PWM to be ready when it will return a 1.
          GPIOPinConfigure(GPIO_PD0_M0PWM6);                                         // Set the Port D Pin 0 to be Module 0 PWM 6.
          GPIOPinConfigure(GPIO_PD1_M1PWM1);                                         // Set the Port D Pin 1 to be Module 1 PWM 1.
          GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);                               // Set the PWM pin to be Port D Pin 0.
          GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);                               // Set the PWM pin to be Port D Pin 1.
          PWM_clock = SysCtlClockGet()/64;                                           // Divide the system clock by 64 to get pulse width modulation clock cycles.
          PWM_freq = 1000;                                                           // Set a frequency of the pulse width modulation.
          Load = (PWM_clock / PWM_freq) - 1;                                         // Calculate the number of PWM clock cycles in the period.
          PWMGenPeriodSet(PWM0_BASE , PWM_GEN_3 , Load);                             // Set the PWM Generator to use PWM Generator 3 with the "Load" period
          PWMGenPeriodSet(PWM1_BASE , PWM_GEN_0 , Load);                             // Set the PWM Generator to use PWM Generator 3 with the "Load" period
          pulseWidth = (Load / 100) * pulsePercent;                                  // Calculate the width of the pulse using a percentage setting, initially 50%

          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pulseWidth);                        // Set the pulse width to 50%.
          PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, pulseWidth);                        // Set the pulse width to 50%.
          PWMGenConfigure(PWM0_BASE , PWM_GEN_3 , PWM_GEN_MODE_DOWN                  // Set the PWM Base to 1, Generator to 3 (or 0,1,2,3), and use Count Down mode (or Up Down)
                       | PWM_GEN_MODE_NO_SYNC);
          PWMGenConfigure(PWM1_BASE , PWM_GEN_0 , PWM_GEN_MODE_DOWN                  // Set the PWM Base to 1, Generator to 3 (or 0,1,2,3), and use Count Down mode (or Up Down)
                       | PWM_GEN_MODE_NO_SYNC);
          PWMGenEnable(PWM0_BASE, PWM_GEN_3 );                                       // Enable the PWM generator;
          PWMGenEnable(PWM1_BASE, PWM_GEN_0 );                                       // Enable the PWM generator;

          GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3);

          GPIOPinWrite(GPIO_PORTD_BASE,
                       GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2|GPIO_PIN_3);
          PWMOutputState(PWM0_BASE , PWM_OUT_6_BIT , true);                          // Turn on the output.
          PWMOutputState(PWM1_BASE , PWM_OUT_1_BIT , true);                          // Turn on the output.

          vSemaphoreCreateBinary(adjust_speed);

          //Strobe();

          if(xTaskCreate(MotorTask, (const portCHAR *)"Motor",
                         MOTORTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                         PRIORITY_MOTOR_TASK, NULL) != pdTRUE)
          {
                return 1;
          }

          return 0;
}                                                                              // End function for initilizing the pulse width modulation of the red built in LED pin PF1

