
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "checkbox.h"
#include "container.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "images.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "drivers/i2cOptDriver.h"
#include "drivers/opt3001.h"
#include "event_groups.h"
#include "queue.h"
#include "driverlib/interrupt.h"

/* BMI160 Drivers */
#include "drivers/i2cBMI160.h"
#include "drivers/bmi160.h"

#define WINDOW_SIZE_LIGHT_SENSOR 6 // Adjust the window size as needed
#define mainQUEUE_LENGTH (4)
#define EVENT_4 (1 << 4)

int sensorBuffer[WINDOW_SIZE_LIGHT_SENSOR];
int bufferIndex = 0;

EventGroupHandle_t xEvent;
QueueHandle_t xSensorQueue = NULL;

struct accelData bmiPacket;

extern SemaphoreHandle_t xTimer2Semaphore;
extern uint32_t g_ui32SysClock;

struct AData
{
    uint32_t ulRawData;
    uint32_t ulFilterData;
    uint32_t ulTimeStamp;
    uint32_t ulFilterData_BMI;
} xData;

static void prvSensorTask(void *pvParameters);
void vCreateSensorTask(void);
static void configureI2C2( void );
static void configureBMI160( void );
int updateBuffer(int newData);

void setupConfig(void){
    configureI2C2();
    configureBMI160();
}

void configureBMI160(void){
    bool worked;
    worked = sensorBMI160Init();
    
    while (!worked) {
        SysCtlDelay(g_ui32SysClock);
        UARTprintf("\nBMI160 initialization failed, Trying again\n");
        worked = sensorBMI160Init();
    }
    UARTprintf("\nBMI160 initalized!\n\n");
    
}

void configureI2C2( void ){
    //
    // The I2C2 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the pin muxing for I2C2 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    //
    // Enable interrupts to the processor.
    //
    I2CMasterIntEnable(I2C2_BASE);
    IntMasterEnable();
}

void vCreateSensorTask(void)
{
    xSensorQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(xData));
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

    xTaskCreate(prvSensorTask,
                "Hello",
                1024,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);


}


/*-----------------------------------------------------------*/
// ████  █████  ███  ████       █████  ███  █████ █  █
// █   █ █     █   █ █   █        █   █   █ █     █ █
// ████  ████  █████ █   █        █   █████ █████ ██ █
// █   █ █     █   █ █   █        █   █   █     █ █  █
// █   █ █████ █   █ ████         █   █   █ █████ █   █
static void prvSensorTask(void *pvParameters)
{
    bool worked, success;
    uint16_t rawData = 0;
    float convertedLux = 0;
    // char tempStr[40];

    for (int i = 0; i < WINDOW_SIZE_LIGHT_SENSOR; i++)
    {
        sensorBuffer[i] = 0;
    }

    // for (int i = 0; i < GRAPH_LEN; i++)
    // {
    //     filter_array_for_graph[i] = 0;
    //     raw_array_for_graph[i] = 0;
    // }

    struct AData xData;

    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // i2c0 interrupt
    //  importtant !!!!!! enable i2c interrupt
    //  IntEnable(INT_I2C0);

    // when this is set i get a bunch of i2c prints
    // I2CMasterIntEnableEx(I2C0_BASE, I2C_MASTER_INT_DATA);
    // I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
    // I2CMasterIntEnable(I2C0_BASE);
    // I2CSlaveIntEnable(I2C0_BASE);
    // Test that sensor is set up correctly

    /* Attempt to create the event group. */
    xEvent = xEventGroupCreate();
    if (xEvent == NULL)
    {
        /* The event group was not created because there was insufficient
        FreeRTOS heap available. */
        UARTprintf("\nxCreatedEventGroup NULL\n");
    }
    else
    {
        /* The event group was created. */
        UARTprintf("\nxCreatedEventGroup created\n");
    }

    // ---- OPT3001 TEST MAKE SURE WORK ----
    UARTprintf("\nTesting OPT3001 Sensor:\n");
    worked = sensorOpt3001Test();

    while (!worked)
    {
        SysCtlDelay(1000);
        UARTprintf("\nTest Failed, Trying again\n");
        worked = sensorOpt3001Test();
    }

    UARTprintf("All Tests Passed!\n\n");

    // Initialize opt3001 sensor
    sensorOpt3001Init();
    sensorOpt3001Enable(true);

    uint16_t    absAccel = 0;
    int         windowSize = 5;
    int         sum = 0;
    int         index = 0;
    int         readings[windowSize];
    int         average = 0;
    
    for(int i = 0; i < windowSize; i++){
        readings[i] = 0;
    }

    for (;;)
    {
        /* Block until the ISR gives the semaphore. */

        // Loop Forever

        // Read and convert OPT values

        // vTaskDelay(30000);
        // UARTprintf("\n before semapghore");
        if (xSemaphoreTake(xTimer2Semaphore, portMAX_DELAY) == pdPASS)
        {
            // UARTprintf("\n insdie semaphore");
            success = sensorOpt3001Read(&rawData);
            sensorBMI160ReadAccel(&bmiPacket);

            absAccel = abs(bmiPacket.xAccel) + abs(bmiPacket.yAccel) + abs(bmiPacket.zAccel);
            
            sum -= readings[index];
            readings[index] = absAccel;
            sum += readings[index];
            index = (index+1) % windowSize;
            average = sum/windowSize;

            xData.ulFilterData_BMI = average / 2048;

            if (success)
            {
                // UARTprintf("\n inside success");
                // naughty
                // MAP_IntMasterDisable();
                sensorOpt3001Convert(rawData, &convertedLux);

                // Construct Text
                // sprintf(tempStr, "Lux: %5.2f\n", convertedLux);
                int lux_int = (int)convertedLux;
                int filteredData = updateBuffer(lux_int);
                // UARTprintf("Lux: %5d\n", lux_int);
                // MAP_IntMasterEnable();
                // UARTprintf("\n after print lux");

                /* Pull the current time stamp. */
                xData.ulTimeStamp = xTaskGetTickCount();
                xData.ulRawData = lux_int;
                xData.ulFilterData = filteredData;
                
                // updateBuffer_for_graph(lux_int, filteredData);
                /* Send the entire structure by value to the queue. */
                // UARTprintf("\n before queue");
                xQueueSend(/* The handle of the queue. */
                           xSensorQueue,
                           /* The address of the xMessage variable.
                            * sizeof( struct AMessage ) bytes are copied from here into
                            * the queue. */
                           (void *)&xData,
                           /* Block time of 0 says don't block if the queue is already
                            * full.  Check the value returned by xQueueSend() to know
                            * if the message was sent to the queue successfully. */
                           (TickType_t)0);
                // UARTprintf("\n after queue");
                /* Create a 5ms delay. */
                // vTaskDelay(pdMS_TO_TICKS(5));

                if (filteredData < 5)
                {
                    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
                }
                else
                {
                    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
                }

                xEventGroupSetBits(xEvent, EVENT_4);
                // UARTprintf("\n after event");
            }
        }
    }
}

int updateBuffer(int newData)
{
    // sensorBuffer[bufferIndex] = newData;
    // bufferIndex = (bufferIndex + 1) % WINDOW_SIZE; // Move to next index, wrapping around if needed

    

    //------------------------------------------------

    int data_pos_0 = sensorBuffer[0];
    int data_pos_1 = sensorBuffer[1];
    int data_pos_2 = sensorBuffer[2];
    int data_pos_3 = sensorBuffer[3];
    int data_pos_4 = sensorBuffer[4];
    // int data_pos_5 = sensorBuffer[5];

    sensorBuffer[0] = newData;
    sensorBuffer[1] = data_pos_0;
    sensorBuffer[2] = data_pos_1;
    sensorBuffer[3] = data_pos_2;
    sensorBuffer[4] = data_pos_3;
    sensorBuffer[5] = data_pos_4;
    // Calculate moving average
    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE_LIGHT_SENSOR; i++)
    {
        sum += sensorBuffer[i];
    }
    int movingAverage = sum / WINDOW_SIZE_LIGHT_SENSOR;

    // Print filtered data (moving average) to console via UART
    // UARTprintf("Filtered Data: %d\n", movingAverage);
    return movingAverage;
}