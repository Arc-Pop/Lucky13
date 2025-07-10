#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "console_task.h"
#include "motor_task.h"


xSemaphoreHandle uart_int;

#define CONSOLETASKSTACKSIZE        128                                        // Stack size in words
#define PRIORITY_CONSOLE_TASK       4


void Green_LED_On(void);                                                       // Change the built in LED to green.
void Blue_LED_On(void);                                                        // Change the built in LED to blue.
void Red_LED_On(void);                                                         // Change the built in LED to red.
void Yellow_LED_On(void);                                                      // Change the built in LED to red.
void LED_Off(void);                                                            // Turn off the built in LED.
void Forward(void);                                                            // Move the robot forward.
void Backward(void);                                                           // Move the robot backward.
void Left(void);                                                               // Move the robot left.
void Right(void);                                                              // Move the robot right.
void Faster(void);                                                             // Move the robot faster.
void Slower(void);                                                             // Move the robot slower.
void Stop(void);                                                               // Stop the robot's movement.
void Start(void);
void RightTurn(void);
void UTurn(void);
void Test(void);
int8_t CodeCheck(const char* code);

extern double max;
extern double percentLeft;
extern double percentRight;
extern int start;

extern uint32_t UTurnStatus;



extern uint32_t RightTurnTime;
extern uint32_t UTurnTime;


extern void acquireData(double error);
extern void PulseWidthSet();
extern void UART_BT_IntHandler();


extern xSemaphoreHandle adjust_speed;

static char codeCatch[4];                                                      // Array for holding code plus null character.
static int codeLength=3;                                                       // Number of digits in code.

int startTime, raceTime;


typedef struct tableCode{                                                      // Structure connects code to the function it represents.
     char code[4];
     void (*functionPointer)(void);
 }Codes;


 static const Codes table[] = {                                                // 2 Dimensional Array with string and function columns.
                              {"gOn" , Green_LED_On},                   // Code for turning on the green LED.
                              {"bOn" , Blue_LED_On},                    // Code for turning on the blue LED.
                              {"rOn" , Red_LED_On},                     // Code for turning on the red LED.
                              {"yOn" , Yellow_LED_On},                  // Code for turning on the red LED.
                              {"off" , LED_Off},                        // Code for turning off the built in LED.
                              {"fwd" , Forward},                        // Code for moving robot forward.
                              {"bwd" , Backward},                       // Code for moving the robot backward.
                              {"lft" , Left},                           // Code for turning the robot to its left.
                              {"rgt" , Right},                          // Code for turning the robot to its right.
                              {"fst" , Faster},                         // Code for making the robot move faster.
                              {"slw" , Slower},                         // Code for making the robot move slower.
                              {"stp" , Stop},                           // Code for making the robot stop moving.
                              {"utn" , UTurn},                          // Code for making the robot do u-turn
                              {"rtn" , RightTurn},                      // Code for making the robot do u-turn
                              {"srt" , Start}                           // Code for making the robot do u-turn
 };

 size_t tableSize = sizeof(table) / sizeof(table[0]);                          // Number of codes.

 void LED_Off(void){ GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1                   // Function for turning off all the built-in LED's.
                                  | GPIO_PIN_2 | GPIO_PIN_3 , 0x0);
}


 void Green_LED_On(void){                                                       // Function for turning on the green built-in LED.
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1
                                   | GPIO_PIN_2 | GPIO_PIN_3 , 0x0);
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
}


 void Blue_LED_On(void){                                                        // Function for turning on the blue built-in LED.
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1
                                  | GPIO_PIN_2 | GPIO_PIN_3 , 0x0);
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}


 void Red_LED_On(void){                                                         // Function for turning on the red built-in LED.
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1
                                  | GPIO_PIN_2 | GPIO_PIN_3 , 0x0);
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


 void Yellow_LED_On(void){                                                      // Function for turning on the red built-in LED.
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1
                                   | GPIO_PIN_2 | GPIO_PIN_3 , 0x0);
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


 void Forward(void){
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2|GPIO_PIN_3);
     percentLeft=70;
     percentRight=70;
     PulseWidthSet();
 }


 void Backward(void){
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 0x0);
     percentLeft=70;
     percentRight=70;
     PulseWidthSet();
 }


 void Left(void){
     if(percentRight<max){ percentRight++;}
     if(percentLeft>80){ percentLeft--;}
     PulseWidthSet();
 }

 void Right(void){
     if(percentLeft<max){ percentLeft++;}
     if(percentRight>80){ percentRight--;}
     PulseWidthSet();
 }


 void Faster(void){
     if((percentLeft+10)<max && (percentRight+10)<max){
     percentLeft+=10;
     percentRight+=10;
     PulseWidthSet();
     }
 }

 void Slower(void){
     if(percentLeft>1 && percentRight>1){ percentLeft--; percentRight--; }
     PulseWidthSet();
 }

 void Stop(void){
     if(start!=2){
     if(xSemaphoreTake(adjust_speed, 10)){
     start=2;
     percentLeft=1;
     percentRight=1;
     PulseWidthSet();
     raceTime = xTaskGetTickCount() - startTime;
     raceTime = (raceTime/1000);
     if(xSemaphoreTake(uart_int, 10)){
     UARTprintf("Race time: %i [s].\n", raceTime);
     xSemaphoreGive(uart_int);
     }
     xSemaphoreGive(adjust_speed);
     }
     }
 }

 void Start(void){

     if(xSemaphoreTake(adjust_speed, 10)){
     UTurnStatus=0;
     start=3;
     percentLeft=max;
     percentRight=max;
     PulseWidthSet();
     startTime = xTaskGetTickCount();
     xSemaphoreGive(adjust_speed);
     }

 }

 void UTurn(void){

     if(xSemaphoreTake(adjust_speed, 10)){
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
     percentRight=max;
     percentLeft=max;
     PulseWidthSet();
     vTaskDelay(UTurnTime);
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
     xSemaphoreGive(adjust_speed);
     }
 }


 void RightTurn(void){
     if(xSemaphoreTake(adjust_speed, 10)){
     percentLeft=max;
     percentRight = 55;
     PulseWidthSet();
     vTaskDelay(RightTurnTime);
     xSemaphoreGive(adjust_speed);
     Faster();
     }
 }

 void Test(void){                                                               // Begin function, for testing code section functionality.
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                               // Enable the port for the line sensors (PB0 left, PB7 right)
     GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1
                           | GPIO_PIN_2 | GPIO_PIN_3 );

     int countDown = 10;
     while(countDown){                                                          // Begin infinite loop, continued light cycle.
         Green_LED_On(); SysCtlDelay(1000000);                                  // Flash green.
         Blue_LED_On();  SysCtlDelay(1000000);                                  // Flash blue.
         Red_LED_On();   SysCtlDelay(1000000);                                  // Flash red.
         countDown--;
     }                                                                          // End infinite loop, continued light cycle.
     LED_Off();
 }                                                                              // End function, for testing code section functionality.
                                                                      // End function, for testing code section functionality.




 void
 CommandPrompt(void)
 {
     UARTprintf("\nType a command:\t");                                            // Print demo introduction.
 }



 static void
 ConsoleTask(void *pvParameters)
 {

    while(1){

        if(xSemaphoreTake(uart_int, 100)){

        while(UARTCharsAvail(UART1_BASE))
        {                                                                              // Begin loop while there are characters in the receive FIFO.
            char charTransfer = UARTCharGet(UART1_BASE);
            UARTCharPut(UART1_BASE, charTransfer);                              // Write character to RF serial terminal.
            vTaskDelay(3);                                                      // Pause for 3[ms]
            codeCatch[strlen(codeCatch)] = charTransfer;                        // Insert character into code array.
        }                                                                       // End loop while there are characters in the receive FIFO.

        if(strlen(codeCatch)==codeLength){                                      // When the full code has been received.
            codeCatch[strlen(codeCatch)] = '\0';                                // Add a null character to terminate string.
            int8_t codeFlag = CodeCheck(codeCatch);                             // Check the code and return integer indicating whether code was in table.
            int i;                                                              // Index for the for loop.
            for(i=0; i<=codeLength ; i++){ codeCatch[i]=0;}                     // Empty the code array.
            if(codeFlag){ CommandPrompt(); }                                    // If the code was found, repeat the request for a command.
        }
        xSemaphoreGive(uart_int);

        }

        vTaskDelay(1);
 }
 }


int8_t CodeCheck(const char* code){                                           // Begin function, for command look up and function execution.
    int index=0;                                                              // Index for row of code in array.

    while(strncmp(code, table[index].code, codeLength)                        // Check for the code in the array
            && index<tableSize){ index++;}

    if(index<tableSize){ table[index].functionPointer(); return 1;}           // If code is found in table, execute corresponding function the return 1 (success)

    else{
        UARTprintf("\nInvalid code\n");
        CommandPrompt();
        return 0;
    }
}




uint32_t
ConsoleTaskInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);                               // Enable UART1 (Bluetooth Serial).
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)){}                          // Wait for Bluetooth connection.
    GPIOPinConfigure(GPIO_PC4_U1RX);                                           // Set GPIO B0 and B1 as UART pins for Bluetooth.
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,                // Configure the UART1 Bluetooth for 9,600, 8-N-1 operation.
          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART1_BASE);
    UARTStdioConfig(1, 9600, SysCtlClockGet());
    UARTEnable(UART1_BASE);
    CommandPrompt();

    vSemaphoreCreateBinary(uart_int);

    if(xTaskCreate(ConsoleTask, (const portCHAR *)"Console",
                   CONSOLETASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_CONSOLE_TASK, NULL) != pdTRUE)
    {
        return 1;
    }

    return 0;
}
