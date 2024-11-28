
/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include "../lib/motorlib/motorlib.h"

#include "driverlib/timer.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
// #include "drivers/pinout.h"

// interrupts
#include <stdint.h>
#include <stdbool.h>


#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"

void HallSensorHandler(void);
void hallA(void);
void hallB(void);
void hallC(void);

volatile int hall_count = 0;
volatile uint8_t duty_setpoint = 0;
// volatile uint16_t rpm_setpoint = 0;
extern volatile int SetSpeed;
volatile int RPM;
void vCreateMotorTask( void );
static void prvMotorTask( void *pvParameters );
extern xQueueHandle xStructQueue;

extern xQueueHandle xRPMqueue;
extern SemaphoreHandle_t xMotorUpdateSemaphore;

int RPMBufferArray[5] = {0,0,0,0,0};

long sum = 0;


// void PrintRPM(void);

void vCreateMotorTask( void )
{
    // xQueuePeek();

    // UARTprintf("Motor Task Created\n");
    xRPMqueue = xQueueCreate(
        /* The number of items the queue can hold. */
        5,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(RPM));
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */
    xTaskCreate( prvMotorTask,
                 "???",
                 512,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
}




void KickStartMotor()
{
    HallSensorHandler();
    setDuty(20);
}

void disableMotorInt()
{
    IntDisable(INT_GPIOM);
    IntDisable(INT_GPIOH);
    IntDisable(INT_GPION);
}

void enableMotorInt()
{
    IntEnable(INT_GPIOM);
    IntEnable(INT_GPIOH);
    IntEnable(INT_GPION);
}



static void prvMotorTask( void *pvParameters )
{
    // vTaskDelay(1000);
    // UARTprintf("Motor Task Started\n");
    duty_setpoint = 20;
    uint16_t period_value = 50;

    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    initMotorLib(period_value);
    /* Set at >10% to get it to start */
    setDuty(duty_setpoint);

    
    // set the hall sensor pins as Input
    MAP_IntMasterEnable();

    MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3); // Hall A
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2); // Hall B
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2); // Hall C

    MAP_IntEnable(INT_GPIOM);
    MAP_IntEnable(INT_GPIOH);
    MAP_IntEnable(INT_GPION);

    MAP_IntPrioritySet(INT_GPIOA, 0x00);
    MAP_IntPrioritySet(INT_GPIOB, 0x00);
    MAP_IntPrioritySet(INT_GPIOC, 0x00);

    /* Kick start the motor */
    // Do an initial read of the hall effect sensor GPIO lines
    // give the read hall effect sensor lines to updateMotor() to move the motor
    // one single phase
    // Recommendation is to use an interrupt on the hall effect sensors GPIO lines 
    // So that the motor continues to be updated every time the GPIO lines change from high to low
    // or low to high
    // Include the updateMotor function call in the ISR to achieve this behaviour.

    /* Motor test - ramp up the duty cycle from 10% to 100%, than stop the motor */
    // HallSensorHandler();

    enableMotor();
    // MAP_TimerEnable(TIMER3_BASE, TIMER_A);
    SetSpeed = 1000;
    int index = 0;
    uint8_t windowSize = 5;
    int average = 0;
    for (;;)
    {
        // UARTprintf("Motor Task Running\n");
        // // Set the RPM to the current setpoint

        // // Delay for 259 milliseconds
        // vTaskDelay(pdMS_TO_TICKS(250));

        if (xSemaphoreTake(xMotorUpdateSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Clear the timer interrupt
            RPM = (hall_count * 600) / 4;
            // Call the HallSensorHandler
            // UARTprintf("RPM: %d\n", RPM);
            hall_count = 0;

            if (RPM >= SetSpeed && duty_setpoint >= 0)
            {
                // decrease duty cycle
                if (duty_setpoint >= 1){
                    duty_setpoint -= 1;
                    setDuty(duty_setpoint);
                }else if (duty_setpoint <= 0)
                {
                    duty_setpoint = 0;
                    setDuty(duty_setpoint);
                    // UARTprintf("Motor Stopped\n");
                    disableMotorInt();
                    stopMotor(0);
                }
                
                // setDuty(duty_setpoint);
            }
            else if (RPM < SetSpeed && duty_setpoint < 50)
            {
                // UARTprintf("increasing");
                // if (duty_setpoint = 0 ){
                //     duty_setpoint = 1;
                // }
                // increase duty cycle
                if (duty_setpoint == 0)
                {
                    duty_setpoint += 1;
                    enableMotorInt();
                    enableMotor();
                    // UARTprintf("Motor Started\n");
                    KickStartMotor();
                }
                else{
                    duty_setpoint += 1;
                }
                setDuty(duty_setpoint);
            }

            sum -= RPMBufferArray[index];
            RPMBufferArray[index] = RPM;
            sum += RPMBufferArray[index];
            index = (index+1) % windowSize;
            average = sum/windowSize;

            xQueueSend(xRPMqueue, &average, 0);
            // UARTprintf("Speed: %d\n", average);
            // UARTprintf("Speed: %d\n", SetSpeed);
            // UARTprintf("Duty: %d\n", duty_setpoint);

        }
        

 
        // //trigger adc conversion
        // MAP_ADCProcessorTrigger(ADC1_BASE, 3);
        // while (!MAP_ADCIntStatus(ADC1_BASE, 3, false));
        // uint32_t adcValue;
        // MAP_ADCSequenceDataGet(ADC1_BASE, 3, &adcValue);
        // UARTprintf("ADC Value: %d\n", adcValue);
        // // Increase the RPM setpoint
        // SetSpeed = 3000;

        // // // If RPM setpoint exceeds 4000, reset to 1000
        // // if (rpm_setpoint > 4000)
        // // {
        // //     rpm_setpoint = 1000;
        // // }
    }
}



void Timer3IntHandlerA(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    MAP_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    xSemaphoreGiveFromISR(xMotorUpdateSemaphore, &xHigherPriorityTaskWoken);
    // UARTprintf("Timer3IntHandlerA\n");
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void HallSensorHandler(void)
{
    
    //1. Read hall effect sensors
    bool hall_a = MAP_GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3);
    bool hall_b = MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2);
    bool hall_c = MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2);
    //
    // printf("Hall A: %d\n", hall_a);
    //2. call update motor to change to next phase
    //   updateMotor(??, ??, ??);
    updateMotor(hall_a, hall_b, hall_c);
    
    //3. Clear interrupt
    // GPIOIntClear(??);

    // Could also add speed sensing code here too.

}

void hallA(void)
{
    uint32_t ui32Status;
    hall_count++;
    ui32Status = GPIOIntStatus(GPIO_PORTM_BASE, true);
    // UARTprintf("Hall A Called");
    HallSensorHandler();
    GPIOIntClear(GPIO_PORTM_BASE, ui32Status);
}
void hallB(void)
{
    uint32_t ui32Status;
    ui32Status = GPIOIntStatus(GPIO_PORTH_BASE, true);
    // UARTprintf("Hall B Called");
    HallSensorHandler();
    GPIOIntClear(GPIO_PORTH_BASE, ui32Status);
}
void hallC(void)
{
    uint32_t ui32Status;
    ui32Status = GPIOIntStatus(GPIO_PORTN_BASE, true);
    // UARTprintf("Hall C Called");
    HallSensorHandler();
    GPIOIntClear(GPIO_PORTN_BASE, ui32Status);
}