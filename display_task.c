/*
 * hello_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/******************************************************************************
 *
 * The Hello task creates a simple task to handle the UART output for the
 * 'Hello World!' message.  A loop is executed five times with a count down
 * before ending with the self-termination of the task that prints the UART
 * message by use of vTaskDelete.  The loop also includes a one second delay
 * that is achieved by using vTaskDelay.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
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

#include "drivers/bmi160.h"

/*-----------------------------------------------------------*/

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Gloal variable used to store the frequency of the system clock.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
tContext sContext;
tRectangle sRect;
tRectangle sRect_text_for_lux;
tRectangle sRect_motor_setup;
tRectangle sRect_intro_time;
tRectangle sRect_motor_display;
tRectangle sRect_motor_display2;

int absAccel;

extern SemaphoreHandle_t xTimer2Semaphore;
extern SemaphoreHandle_t xBmi160Semaphore;
extern QueueHandle_t xSensorQueue;
extern QueueHandle_t xRPMqueue;
extern QueueHandle_t xstructQueue;

struct AData
{
    uint32_t ulRawData;
    uint32_t ulFilterData;
    uint32_t ulTimeStamp;
    uint32_t ulFilterData_BMI;
} xData_Read;

int RPM = 0;
int RPMPrev = 0;

struct accelData accelPacket;
//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment = 1024
tDMAControlTable psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(psDMAControlTable, 1024)
tDMAControlTable psDMAControlTable[64];
#else
tDMAControlTable psDMAControlTable[64] __attribute__((aligned(1024)));
#endif

//*****************************************************************************
//
// Forward declarations for the globals required to define the widgets at
// compile-time.
//
//*****************************************************************************

// Variable for sensor
int acceleration_limit = 10;
char acceleration_limit_text[30];

int lux = 0;
char lux_val_text[30];

int acceleration_value = 0;
char acceleration_value_text[30];

static void ClearBanner(void);
static void ClearText_Sensor_Page_opt(void);
static void ClearText_Sensor_Page_bmi(void);

// Motor variables

int speed = 0;
int current = 0;
volatile int SetSpeed = 0;

char speed_lim_text[30];
char current_lim_text[30];
char speed_text[30];

// Variables for timer
int hour = 10;
int minute = 30;
int second = 0;
char date_time[40];

// Motor Display Variables
char str[500];
bool stateChange = false;

int previousFilteredLux = 0;
int previousRawLux = 0;

static void ClearBanner_intro_timer(void);

void OnPrevious(tWidget *psWidget);
void OnNext(tWidget *psWidget);
void OnNext_static_to_Home(tWidget *psWidget);
void OnNext_static_to_Motor_Set_Up(tWidget *psWidget);
void OnNext_static_to_Motor_Display(tWidget *psWidget);
void OnNext_static_to_Sensor_Display(tWidget *psWidget);
void OnIntroPaint(tWidget *psWidget, tContext *psContext);
void OnCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnCheckChange(tWidget *psWidget, uint32_t bSelected);
void OnButtonPress_Home(tWidget *psWidget);
void OnButtonPress_Sensor_Page(tWidget *psWidget);
void OnSliderChange(tWidget *psWidget, int32_t i32Value);
void Motor_Display(tWidget *psWidget);

static void Clear_Banner_Motor_Text(void);

void OnButtonPress_Motor_Setup(tWidget *psWidget);
extern tCanvasWidget g_psPanels[];

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayDemoTask(void *pvParameters);

/*
 * Called by main() to create the Hello print task.
 */
void vDisplayDemoTask(void);
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

// █████ █   █ █████ ████   ███  ████  █   █  ████ █████ █████  ███  █   █
//   █   ██  █   █   █   █ █   █ █   █ █   █ █       █     █   █   █ ██  █
//   █   █ █ █   █   ████  █   █ █   █ █   █ █       █     █   █   █ █ █ █
//   █   █  ██   █   █   █ █   █ █   █ █   █ █       █     █   █   █ █  ██
// █████ █   █   █   █   █  ███  ████  █████  ████   █   █████  ███  █   █
Canvas(g_sIntroduction_Page, g_psPanels, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnIntroPaint);

//     █   █  ███  █   █ █████
//     █   █ █   █ ██ ██ █
//     █████ █   █ █ █ █ ████
//     █   █ █   █ █   █ █
//     █   █  ███  █   █ █████
tCanvasWidget g_psPushButtonIndicators[] =
    {
        CanvasStruct(g_psPanels + 1, g_psPushButtonIndicators + 1, 0,
                     &g_sKentec320x240x16_SSD2119, 90, 35, 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, "Motor Set Up Page",
                     0, 0),
        CanvasStruct(g_psPanels + 1, g_psPushButtonIndicators + 2, 0,
                     &g_sKentec320x240x16_SSD2119, 90, (35 + 50), 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, "Motor Display Page",
                     0, 0),
        CanvasStruct(g_psPanels + 1, 0, 0,
                     &g_sKentec320x240x16_SSD2119, 90, (35 + 50 + 50), 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, "Sensor Display Page",
                     0, 0)};
tPushButtonWidget g_sHome_Page[] =
    {
        RectangularButtonStruct(g_psPanels + 1, g_sHome_Page + 1, 0,
                                &g_sKentec320x240x16_SSD2119, 30, 35, 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "1", 0, 0, 0, 0, OnButtonPress_Home),
        RectangularButtonStruct(g_psPanels + 1, g_sHome_Page + 2, 0,
                                &g_sKentec320x240x16_SSD2119, 30, (35 + 50), 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "2", 0, 0, 0, 0, OnButtonPress_Home),
        RectangularButtonStruct(g_psPanels + 1, g_psPushButtonIndicators, 0,
                                &g_sKentec320x240x16_SSD2119, 30, (35 + 50 + 50), 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "3", 0, 0, 0, 0, OnButtonPress_Home),
};

#define NUM_PUSH_BUTTONS (sizeof(g_sHome_Page) / \
                          sizeof(g_sHome_Page[0]))
uint32_t g_ui32ButtonState;

// ███╗░░░███╗░█████╗░████████╗░█████╗░██████╗░  ░██████╗███████╗████████╗██╗░░░██╗██████╗░
//  ████╗░████║██╔══██╗╚══██╔══╝██╔══██╗██╔══██╗  ██╔════╝██╔════╝╚══██╔══╝██║░░░██║██╔══██╗
//  ██╔████╔██║██║░░██║░░░██║░░░██║░░██║██████╔╝  ╚█████╗░█████╗░░░░░██║░░░██║░░░██║██████╔╝
//  ██║╚██╔╝██║██║░░██║░░░██║░░░██║░░██║██╔══██╗  ░╚═══██╗██╔══╝░░░░░██║░░░██║░░░██║██╔═══╝░
//  ██║░╚═╝░██║╚█████╔╝░░░██║░░░╚█████╔╝██║░░██║  ██████╔╝███████╗░░░██║░░░╚██████╔╝██║░░░░░
//  ╚═╝░░░░░╚═╝░╚════╝░░░░╚═╝░░░░╚════╝░╚═╝░░╚═╝  ╚═════╝░╚══════╝░░░╚═╝░░░░╚═════╝░╚═╝░░░░░

tPushButtonWidget g_psPushButtons_motor[] =
    {
        RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 1, 0,
                                &g_sKentec320x240x16_SSD2119, 10, 35, 50, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "Start", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 2, 0,
                                &g_sKentec320x240x16_SSD2119, 10, 125, 50, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "Stop", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 3, 0,
                                &g_sKentec320x240x16_SSD2119, 75, 35, 60, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "+ M.Curr", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 4, 0,
                                &g_sKentec320x240x16_SSD2119, 75, 125, 60, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "- M.Curr", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        // RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 5, 0,
        //                         &g_sKentec320x240x16_SSD2119, 150, 35, 70, 30,
        //                         PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
        //                         ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
        //                         &g_sFontCm14, "+ M.Speed", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        // RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 6, 0,
        //                         &g_sKentec320x240x16_SSD2119, 150, 125, 70, 30,
        //                         PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
        //                         ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
        //                         &g_sFontCm14, "- M.Speed", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        RectangularButtonStruct(g_psPanels + 2, g_psPushButtons_motor + 5, 0,
                                &g_sKentec320x240x16_SSD2119, 235, 35, 60, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "+ Speed", 0, 0, 0, 0, OnButtonPress_Motor_Setup),

        RectangularButtonStruct(g_psPanels + 2, 0, 0,
                                &g_sKentec320x240x16_SSD2119, 235, 125, 60, 30,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm14, "- Speed", 0, 0, 0, 0, OnButtonPress_Motor_Setup),
};

#define NUM_PUSH_BUTTONS_MOTOR_SETUP (sizeof(g_psPushButtons_motor) / \
                                      sizeof(g_psPushButtons_motor[0]))

uint32_t g_ui32ButtonState_Motor_Setup;

// ███╗░░░███╗░█████╗░████████╗░█████╗░██████╗░  ██████╗░██╗░██████╗██████╗░██╗░░░░░░█████╗░██╗░░░██╗
// ████╗░████║██╔══██╗╚══██╔══╝██╔══██╗██╔══██╗  ██╔══██╗██║██╔════╝██╔══██╗██║░░░░░██╔══██╗╚██╗░██╔╝
// ██╔████╔██║██║░░██║░░░██║░░░██║░░██║██████╔╝  ██║░░██║██║╚█████╗░██████╔╝██║░░░░░███████║░╚████╔╝░
// ██║╚██╔╝██║██║░░██║░░░██║░░░██║░░██║██╔══██╗  ██║░░██║██║░╚═══██╗██╔═══╝░██║░░░░░██╔══██║░░╚██╔╝░░
// ██║░╚═╝░██║╚█████╔╝░░░██║░░░╚█████╔╝██║░░██║  ██████╔╝██║██████╔╝██║░░░░░███████╗██║░░██║░░░██║░░░
// ╚═╝░░░░░╚═╝░╚════╝░░░░╚═╝░░░░╚════╝░╚═╝░░╚═╝  ╚═════╝░╚═╝╚═════╝░╚═╝░░░░░╚══════╝╚═╝░░╚═╝░░░╚═╝░░░

Canvas(g_sMotorDisplay_Page, g_psPanels + 3, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, Motor_Display);

// █████ █████ █   █ █████  ███  ████         ████  █████ █████ ████  █      ███  █   █
// █     █     ██  █ █     █   █ █   █        █   █   █   █     █   █ █     █   █  █ █
// █████ ████  █ █ █ █████ █   █ ████         █   █   █   █████ ████  █     █████   █
//     █ █     █  ██     █ █   █ █   █        █   █   █       █ █     █     █   █   █
// █████ █████ █   █ █████  ███  █   █        ████  █████ █████ █     █████ █   █   █
tCanvasWidget g_psPushButtonIndicators_Sensor_Display[] =
    {
        CanvasStruct(g_psPanels + 4, g_psPushButtonIndicators_Sensor_Display + 1, 0,
                     &g_sKentec320x240x16_SSD2119, 90, 35, 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, "+ acceleration limit",
                     0, 0),
        CanvasStruct(g_psPanels + 4, g_psPushButtonIndicators_Sensor_Display + 2, 0,
                     &g_sKentec320x240x16_SSD2119, 90, (35 + 50 + 15), 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, "- acceleration limit",
                     0, 0),
        CanvasStruct(g_psPanels + 4, 0, 0,
                     &g_sKentec320x240x16_SSD2119, 90, (35 + 50 - 30), 190, 24,
                     CANVAS_STYLE_TEXT, 0, 0, ClrSilver, &g_sFontCm20, acceleration_limit_text,
                     0, 0)};
tPushButtonWidget g_sSensorDisplay_Page[] =
    {
        RectangularButtonStruct(g_psPanels + 4, g_sSensorDisplay_Page + 1, 0,
                                &g_sKentec320x240x16_SSD2119, 30, 35, 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "+", 0, 0, 0, 0, OnButtonPress_Sensor_Page),
        RectangularButtonStruct(g_psPanels + 4, g_psPushButtonIndicators_Sensor_Display, 0,
                                &g_sKentec320x240x16_SSD2119, 30, (35 + 50), 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "-", 0, 0, 0, 0, OnButtonPress_Sensor_Page),
};
#define NUM_PUSH_BUTTONS_SENSOR_DISPLAY (sizeof(g_sSensorDisplay_Page) / \
                                         sizeof(g_sSensorDisplay_Page[0]))
uint32_t g_ui32ButtonState_Sensor_Display;
//  ████  ███  █   █ █   █  ███  █████        █   █ █████ ████   ████ █████ █████        ███  ████  ████   ███  █   █
// █     █   █ ██  █ █   █ █   █ █            █   █   █   █   █ █     █       █         █   █ █   █ █   █ █   █  █ █
// █     █████ █ █ █  █ █  █████ █████        █ █ █   █   █   █ █  ██ ████    █         █████ ████  ████  █████   █
// █     █   █ █  ██  █ █  █   █     █        ██ ██   █   █   █ █   █ █       █         █   █ █   █ █   █ █   █   █
//  ████ █   █ █   █   █   █   █ █████        █   █ █████ ████   ███  █████   █         █   █ █   █ █   █ █   █   █
//*****************************************************************************
//
// An array of canvas widgets, one per panel.  Each canvas is filled with
// black, overwriting the contents of the previous panel.
//
//*****************************************************************************
// notes make sure the correct "g_psPanels + x" is given to the correct numbered panle in this array or else the button will stop working
tCanvasWidget g_psPanels[] =
    {
        CanvasStruct(0, 0, &g_sIntroduction_Page, &g_sKentec320x240x16_SSD2119, 0, 24,
                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        CanvasStruct(0, 0, &g_sHome_Page, &g_sKentec320x240x16_SSD2119, 0, 24,
                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        CanvasStruct(0, 0, &g_psPushButtons_motor, &g_sKentec320x240x16_SSD2119, 0, 24,
                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        CanvasStruct(0, 0, &g_sMotorDisplay_Page, &g_sKentec320x240x16_SSD2119, 0, 24,
                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        CanvasStruct(0, 0, &g_sSensorDisplay_Page, &g_sKentec320x240x16_SSD2119, 0, 24,
                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),

};

//*****************************************************************************
//
// The number of panels.
//
//*****************************************************************************
#define NUM_PANELS (sizeof(g_psPanels) / sizeof(g_psPanels[0]))

//*****************************************************************************
//
// The names for each of the panels, which is displayed at the bottom of the
// screen.
//
//*****************************************************************************
char *g_pcPanei32Names[] =
    {
        "     Introduction     ",
        "     Home     ",
        "     Motor Setup     ",
        "     Motor Display     ",
        "     Sensor Display     ",
};

//*****************************************************************************
//
// The buttons and text across the bottom of the screen.
//
//*****************************************************************************
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 190,
                  50, 50, PB_STYLE_FILL, ClrBlack, ClrBlack, 0, ClrSilver,
                  &g_sFontCm20, "-", g_pui8Blue50x50, g_pui8Blue50x50Press, 0, 0,
                  OnPrevious);

Canvas(g_sTitle, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 50, 190, 220, 50,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 270, 190,
                  50, 50, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrSilver, &g_sFontCm20, "+", g_pui8Blue50x50,
                  g_pui8Blue50x50Press, 0, 0, OnNext);

RectangularButton(g_sHomeButton, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 0,
                  70, 24, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrWhite, &g_sFontCm20, "Home", g_pui8Blue50x50,
                  g_pui8Blue50x50Press, 0, 0, OnNext_static_to_Home);

// The panel that is currently being displayed.
uint32_t g_ui32Panel;

// Handles presses of the previous panel button.
void OnPrevious(tWidget *psWidget)
{
    //
    // There is nothing to be done if the first panel is already being
    // displayed.
    //
    if (g_ui32Panel == 0)
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Decrement the panel index.
    //
    g_ui32Panel--;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if this is the first panel.
    //
    if (g_ui32Panel == 0)
    {
        //
        // Clear the previous button from the display since the first panel is
        // being displayed.
        //
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if the previous panel was the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 2))
    {
        //
        // Display the next button.
        //
        PushButtonImageOn(&g_sNext);
        PushButtonTextOn(&g_sNext);
        PushButtonFillOff(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

// Handles presses of the next panel button.
void OnNext(tWidget *psWidget)
{
    UARTprintf("\n OnNext");

    //
    // There is nothing to be done if the last panel is already being
    // displayed.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Increment the panel index.
    //
    g_ui32Panel++;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if the previous panel was the first panel.
    //
    if (g_ui32Panel == 1)
    {
        //
        // Display the previous button.
        //
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if this is the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        //
        // Clear the next button from the display since the last panel is being
        // displayed.
        //
        PushButtonImageOff(&g_sNext);
        PushButtonTextOff(&g_sNext);
        PushButtonFillOn(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

// it currently appents the current panel index
void OnNext_static_to_Home(tWidget *psWidget)
{
    int people = xTaskGetTickCount();
    UARTprintf("\n OnNext_static_to_Home.. g_ui32Panel:%d, people:%d", g_ui32Panel, people);
    if (g_ui32Panel == 1)
    {
        return;
    }
    // Remove the current panel.
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    g_ui32Panel = 1;

    // Add and draw the new panel.
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 1));
    WidgetPaint((tWidget *)(g_psPanels + 1));

    // Set the title of this panel.
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[1]);
    WidgetPaint((tWidget *)&g_sTitle);
}

void OnNext_static_to_Motor_Set_Up(tWidget *psWidget)
{
    UARTprintf("\n OnNext_static_to_1");
    if (g_ui32Panel == 2)
    {
        return;
    }
    // Remove the current panel.
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    g_ui32Panel = 2;

    // Add and draw the new panel.
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 2));
    WidgetPaint((tWidget *)(g_psPanels + 2));

    // Set the title of this panel.
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[2]);
    WidgetPaint((tWidget *)&g_sTitle);
}

void OnNext_static_to_Motor_Display(tWidget *psWidget)
{
    UARTprintf("\n OnNext_static_to_1");
    if (g_ui32Panel == 3)
    {
        return;
    }
    // Remove the current panel.
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    g_ui32Panel = 3;

    // Add and draw the new panel.
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 3));
    WidgetPaint((tWidget *)(g_psPanels + 3));

    // Set the title of this panel.
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[3]);
    WidgetPaint((tWidget *)&g_sTitle);
}

void OnNext_static_to_Sensor_Display(tWidget *psWidget)
{
    UARTprintf("\n OnNext_static_to_1");
    if (g_ui32Panel == 4)
    {
        return;
    }
    // Remove the current panel.
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    g_ui32Panel = 4;

    // Add and draw the new panel.
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + 4));
    WidgetPaint((tWidget *)(g_psPanels + 4));

    // Set the title of this panel.
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[4]);
    WidgetPaint((tWidget *)&g_sTitle);
}
// Handles paint requests for the introduction canvas widget.
void OnIntroPaint(tWidget *psWidget, tContext *psContext)
{
    //
    // Display the introduction text in the canvas.
    //
    GrContextFontSet(psContext, &g_sFontCm18);
    GrContextForegroundSet(psContext, ClrSilver);
    // GrStringDraw(psContext, "This application demonstrates the", -1,
    //              0, 32, 0);
    // GrStringDraw(psContext, "TivaWare Graphics Library.", -1, 0, 50, 0);
    // GrStringDraw(psContext, "Each panel shows a different feature of", -1, 0,
    //              74, 0);
    // GrStringDraw(psContext, "the graphics library. Widgets on the panels", -1, 0,
    //              92, 0);
    // GrStringDraw(psContext, "are fully operational; pressing them will", -1, 0,
    //              110, 0);
    // GrStringDraw(psContext, "result in visible feedback of some kind.", -1, 0,
    //              128, 0);

    GrStringDraw(psContext, "Press the + and - buttons at the bottom", -1, 0,
                 146, 0);
    GrStringDraw(psContext, "of the screen to move between the panels.", -1, 0,
                 164, 0);
}

// Handles paint requests for the canvas demonstration widget.
void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
{

    // define and draw the scale
    uint32_t xMax = 10;
    uint32_t yMax = 90;
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "90", -1,
                         10, 33, 0);
    GrStringDrawCentered(&sContext, "0", -1,
                         10, 160, 0);
    GrStringDrawCentered(&sContext, "0s", -1,
                         30, 175, 0);
    GrStringDrawCentered(&sContext, "10s", -1,
                         210, 175, 0);

    // define the size of the canvas - Eg: 20, 27, 200, 140
    uint32_t canvas_xMin = 20;
    uint32_t canvas_xRange = 200;
    uint32_t canvas_yMin = 27;
    uint32_t canvas_yRange = 140;

    // define a list of integers
    uint32_t plot_data[10] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512};

    // iterate through the data and draw a rectangle on the screen
    uint32_t ui32Idx;
    tRectangle sRect;
    GrContextForegroundSet(psContext, ClrGoldenrod);
    for (ui32Idx = 0; ui32Idx < 10; ui32Idx += 1)
    {
        // get the value of each element and convert to plotting coordinate
        float display_y = plot_data[ui32Idx];
        display_y = canvas_yMin + canvas_yRange - (display_y / yMax) * canvas_yRange;

        // get the current index, denoting time
        float display_x = ui32Idx;
        display_x = (display_x / xMax) * canvas_xRange + canvas_xMin;

        // draw rectangle here
        sRect.i16XMin = display_x;
        sRect.i16YMin = display_y - 10;
        // sRect.i16XMax = GrContextDpyWidthGet(psContext) - 1;
        sRect.i16XMax = display_x + 10;
        sRect.i16YMax = display_y;
        GrContextForegroundSet(psContext, ClrDarkBlue);
        GrRectFill(psContext, &sRect);
        GrContextForegroundSet(psContext, ClrWhite);
        GrRectDraw(psContext, &sRect);
    }
}

void OnButtonPress_Home(tWidget *psWidget)
{
    uint32_t ui32Idx;
    UARTprintf("\n OnButtonPress");
    // Find the index of this push button.
    for (ui32Idx = 0; ui32Idx < NUM_PUSH_BUTTONS; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_sHome_Page + ui32Idx))
        {
            break;
        }
    }
    UARTprintf("\n number: %d", ui32Idx);
    // Return if the push button could not be found.
    if (ui32Idx == NUM_PUSH_BUTTONS)
    {
        return;
    }

    // Toggle the state of this push button indicator.
    g_ui32ButtonState ^= 1 << ui32Idx;
    switch (ui32Idx)
    {
    case 0:
        UARTprintf("\n OnButtonPress_Home switch 0");
        OnNext_static_to_Motor_Set_Up(psWidget);
        break;
    case 1:
        UARTprintf("\n OnButtonPress_Home switch 1");
        OnNext_static_to_Motor_Display(psWidget);
        break;
    case 2:
        UARTprintf("\n OnButtonPress_Home switch 2");
        OnNext_static_to_Sensor_Display(psWidget);
        break;
    default:
        OnNext(psWidget);
        break;
    }
}

void OnButtonPress_Sensor_Page(tWidget *psWidget)
{
    uint32_t ui32Idx;
    tRectangle sRect_accel;
    UARTprintf("\n OnButtonPress");
    // Find the index of this push button.
    for (ui32Idx = 0; ui32Idx < NUM_PUSH_BUTTONS_SENSOR_DISPLAY; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_sSensorDisplay_Page + ui32Idx))
        {
            break;
        }
    }
    UARTprintf("\n number: %d", ui32Idx);
    // Return if the push button could not be found.
    if (ui32Idx == NUM_PUSH_BUTTONS_SENSOR_DISPLAY)
    {
        return;
    }

    // Toggle the state of this push button indicator.
    g_ui32ButtonState_Sensor_Display ^= 1 << ui32Idx;

    sRect_accel.i16XMin = 90;
    sRect_accel.i16YMin = (35 + 50 - 30);
    sRect_accel.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect_accel.i16YMax = (35 + 50 - 30 + 24);
    switch (ui32Idx)
    {
    case 0:
        UARTprintf("\n OnButtonPress_Sensor_Page switch 0");
        GrContextForegroundSet(&sContext, ClrDarkKhaki); // fill
        GrRectFill(&sContext, &sRect_accel);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        acceleration_limit++;
        usnprintf(acceleration_limit_text, 30, "acceleration limit: %d", acceleration_limit);
        GrStringDrawCentered(&sContext, acceleration_limit_text, 30, ((GrContextDpyWidthGet(&sContext) / 6) * 4), (35 + 50 - 30 + 12), 0);
        break;
    case 1:
        UARTprintf("\n OnButtonPress_Sensor_Page switch 1");
        GrContextForegroundSet(&sContext, ClrDarkKhaki); // fill
        GrRectFill(&sContext, &sRect_accel);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        acceleration_limit--;
        usnprintf(acceleration_limit_text, 30, "acceleration limit: %d", acceleration_limit);
        GrStringDrawCentered(&sContext, acceleration_limit_text, 30, ((GrContextDpyWidthGet(&sContext) / 6) * 4), (35 + 50 - 30 + 12), 0);
        break;
    default:
        OnNext(psWidget);
        break;
    }
}

void OnButtonPress_Motor_Setup(tWidget *psWidget)
{
    uint32_t ui32Idx;
    UARTprintf("\n OnButtonPress");
    // Find the index of this push button.
    for (ui32Idx = 0; ui32Idx < NUM_PUSH_BUTTONS_MOTOR_SETUP; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_psPushButtons_motor + ui32Idx))
        {
            break;
        }
    }
    UARTprintf("\n number: %d", ui32Idx);
    // Return if the push button could not be found.
    if (ui32Idx == NUM_PUSH_BUTTONS_MOTOR_SETUP)
    {
        return;
    }

    // Toggle the state of this push button indicator.
    g_ui32ButtonState_Motor_Setup ^= 1 << ui32Idx;
    switch (ui32Idx)
    {
    case 0:
        UARTprintf("\n OnButtonPress_Motor: Start");
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        SetSpeed = 1000;
        // clear print area
        Clear_Banner_Motor_Text();
        // print new text
        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);

        usnprintf(speed_text, 50, "S: %d", SetSpeed);
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);
        break;
    case 1:
        UARTprintf("\n OnButtonPress_Motor: Stop");
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
        SetSpeed = 0;
        // clear print area
        Clear_Banner_Motor_Text();
        // print new text
        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);

        usnprintf(speed_text, 50, "S: %d", SetSpeed);
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);
        break;
    case 2:
        UARTprintf("\n OnButtonPress_Motor: + Max Current");
        Clear_Banner_Motor_Text();
        current++;
        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);

        // usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        // GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        usnprintf(speed_text, 50, "S: %d", SetSpeed);
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);
        break;
    case 3:
        UARTprintf("\n OnButtonPress_Motor: - Max Curret");
        Clear_Banner_Motor_Text();
        current--;
        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);

        // usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        // GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        usnprintf(speed_text, 50, "S: %d", SetSpeed);
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);

        break;
        // case 4:
        //     UARTprintf("\n OnButtonPress_Motor: + Max Speed");
        //     Clear_Banner_Motor_Text();
        //     speed++;
        //     usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        //     GrContextForegroundSet(&sContext, ClrWhite); // fill
        //     GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        //     usnprintf(current_lim_text, 30, "M.C: %d", current);
        //     GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75+20, 0);

        //     usnprintf(speed_text, 30, "S: %d", SetSpeed);
        //     GrStringDrawCentered(&sContext, speed_text, 30, 270, 75+20, 0);

        //     break;
        // case 5:
        //     UARTprintf("\n OnButtonPress_Motor: - Max Speed");
        //     Clear_Banner_Motor_Text();
        //     speed--;
        //     usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        //     GrContextForegroundSet(&sContext, ClrWhite); // fill
        //     GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        //     usnprintf(current_lim_text, 30, "M.C: %d", current);
        //     GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75+20, 0);

        //     usnprintf(speed_text, 30, "S: %d", SetSpeed);
        //     GrStringDrawCentered(&sContext, speed_text, 30, 270, 75+20, 0);

        //     break;

    case 4:
        UARTprintf("\n OnButtonPress_Motor: + Speed");
        Clear_Banner_Motor_Text();
        SetSpeed += 500;
        usnprintf(speed_text, 30, "S: %d", SetSpeed);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);

        // usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        // GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);
        break;

    case 5:
        UARTprintf("\n OnButtonPress_Motor: - Speed");
        Clear_Banner_Motor_Text();
        SetSpeed -= 500;
        usnprintf(speed_text, 30, "S: %d", SetSpeed);
        GrContextForegroundSet(&sContext, ClrWhite); // fill
        GrStringDrawCentered(&sContext, speed_text, 30, 270, 75 + 20, 0);

        // usnprintf(speed_lim_text, 50, "M.S: %d", speed);
        // GrStringDrawCentered(&sContext, speed_lim_text, 50, 190, 75+20, 0);

        usnprintf(current_lim_text, 30, "M.C: %d", current);
        GrStringDrawCentered(&sContext, current_lim_text, 30, 110, 75 + 20, 0);
        break;
    default:
        OnNext(psWidget);
        break;
    }
}

void Motor_Display(tWidget *psWidget)
{
}

void vDisplayDemoTask(void)
{
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
    xTaskCreate(prvDisplayDemoTask,
                "Hello",
                1024,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvDisplayDemoTask(void *pvParameters)
{
    // tRectangle sRect;

    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                            120000000);

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrOrange);
    GrRectFill(&sContext, &sRect);

    // Put a white box around the banner.
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    // char monkey[30];
    // vTaskDelay(10);
    // usnprintf(monkey, 30, "filtered data %d", xTaskGetTickCount());
    // GrStringDrawCentered(&sContext, monkey, -1,
    //                      GrContextDpyWidthGet(&sContext) / 2, 8, 0);

    sRect_motor_display.i16XMin = 0;
    sRect_motor_display.i16YMin = 22;
    sRect_motor_display.i16XMax = GrContextDpyWidthGet(&sContext);
    sRect_motor_display.i16YMax = 180;

    sRect_motor_display2.i16XMin = 0;
    sRect_motor_display2.i16YMin = 22;
    sRect_motor_display2.i16XMax = GrContextDpyWidthGet(&sContext);
    sRect_motor_display2.i16YMax = 46;

    int eStopBMI = 0;
    int previousRawLux = 0;
    int xPosInit = 0;  // PX position where the graph starts drawing from
    int xPosFin = 320; // PX position where the graph stops drawing and wraps back to xPosInit
    int yPos = 150;    // Px position where 0 lux corresponses (i.e. Lowest part of the graph drawn)
    int scale = 100;   // Scales the lux value to fit on the screen. Header still reads the actual lux value, only the graph is scaled
    int previousFilteredLux = 0;

    //
    // Configure and enable uDMA
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    uDMAControlBaseSet(&psDMAControlTable[0]);
    uDMAEnable();

    //
    // Initialize the touch screen driver and have it route its messages to the
    // widget tree.
    //
    TouchScreenInit(g_ui32SysClock);
    TouchScreenCallbackSet(WidgetPointerMessage);

    //
    // Add the title block and the previous and next buttons to the widget
    // tree.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevious);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitle);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNext);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sHomeButton);
    //
    // Add the first panel to the widget tree.
    //
    g_ui32Panel = 0;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[0]);

    //
    // Issue the initial paint request to the widgets.
    //
    WidgetPaint(WIDGET_ROOT);

    // base strong for sensor page
    usnprintf(acceleration_limit_text, 30, "acceleration limit: %d", acceleration_limit);
    int prev_absAccel = 0;
    //
    // Loop forever handling widget messages.
    //
    for (;;)
    {
        //
        // Process any messages in the widget message queue.
        //
        // vTaskDelay(5000);
        WidgetMessageQueueProcess();

        // code for estop
        if (xQueueReceive(xSensorQueue, &(xData_Read), (TickType_t)10) == pdPASS)
        {
            if (xData_Read.ulFilterData_BMI >= acceleration_limit)
            {
                SetSpeed = 0;
            }
        }

        if (g_ui32Panel == 4) // SENSOR PAGE
        {
            ClearText_Sensor_Page_opt();
            ClearText_Sensor_Page_bmi();
            GrContextForegroundSet(&sContext, ClrWhite);
            usnprintf(lux_val_text, 30, "Lux data %d", xData_Read.ulFilterData);
            GrStringDrawCentered(&sContext, lux_val_text, -1, GrContextDpyWidthGet(&sContext) / 2, 145, 0);
            usnprintf(acceleration_value_text, 30, "acceleration data %d", xData_Read.ulFilterData_BMI);
            GrStringDrawCentered(&sContext, acceleration_value_text, -1, GrContextDpyWidthGet(&sContext) / 2, 165, 0);
    
            stateChange = true;
        }
        if (g_ui32Panel == 0)
        {
            // TIme Start = 10:30am, 30th May 2024
            if (minute >= 60)
            {
                hour++;
                minute = 0;
            }
            if (second >= 60)
            {
                minute++;
                second = 0;
            }

            ClearBanner_intro_timer();
            GrContextForegroundSet(&sContext, ClrWhite);
            usnprintf(date_time, 30, "30th May, %d:%d:%dam", hour, minute, second);

            GrStringDrawCentered(&sContext, date_time, -1, GrContextDpyWidthGet(&sContext) / 2, 35, 0);
            stateChange = true;
        }
        if (g_ui32Panel == 3) // --> PLOTTING PAGE <--
        {
            if (xQueueReceive(xRPMqueue, &(RPM), (TickType_t)10) == pdPASS)
            {

                if ((RPM != RPMPrev))
                { // Only update if the raw value is different or a button press occured

                    // THIS CODE PRINT LUX ON THE TOP OF THE PAGE ON THE PLOTTING PAGE
                    usnprintf(str, 20, "RPM: %d", RPM);
                    GrContextForegroundSet(&sContext, ClrBlack);
                    GrRectFill(&sContext, &sRect_motor_display2);
                    GrContextFontSet(&sContext, &g_sFontCm20);
                    GrContextForegroundSet(&sContext, ClrWhite);
                    GrStringDrawCentered(&sContext, str, -1, GrContextDpyWidthGet(&sContext) / 2, 35, 0);
                }

                // GrContextForegroundSet(&sContext, ClrRed);
                // GrLineDraw(&sContext, xPosInit, yPos - previousRawLux/scale, xPosInit + 1, yPos - xData_Read.ulRawData/scale);

                // plot first line
                GrContextForegroundSet(&sContext, ClrWhite);
                GrLineDraw(&sContext, xPosInit, yPos - previousFilteredLux / scale, xPosInit, yPos - RPM / scale);

                // plot second line
                // GrContextForegroundSet(&sContext, ClrRed);
                // GrLineDraw(&sContext, xPosInit,  yPos  - previousRawLux/scale, xPosInit + 1, yPos - xData_Read.ulRawData/scale - 1);

                if ((xPosInit >= xPosFin) | stateChange)
                { // If the xInterval goes off screen OR if a button is pressed to change state: reset the xInterval and clear screen
                    xPosInit = 0;
                    GrContextForegroundSet(&sContext, ClrBlack);
                    GrRectFill(&sContext, &sRect_motor_display);
                    GrContextForegroundSet(&sContext, ClrWhite);
                }
                else
                { // Increment xInterval otherwise.
                    xPosInit += 1;
                }
                // previousFilteredLux = xData_Read.ulFilterData;
                previousRawLux = xData_Read.ulRawData;
                previousFilteredLux = xData_Read.ulFilterData;
                stateChange = false;
            }
        }
    }
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}

void Timer2IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int people = xTaskGetTickCountFromISR();
    // UARTprintf("\n people:%d", people);
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // second++;

    //
    // Update the interrupt status.
    //
    // MAP_IntMasterDisable();

    // UARTprintf("\n T1 \n");
    // MAP_IntMasterEnable();

    // Give the semaphore to unblock the waiting task
    xSemaphoreGiveFromISR(xTimer2Semaphore, &xHigherPriorityTaskWoken);
    // xSemaphoreGiveFromISR(xBmi160Semaphore, &xHigherPriorityTaskWoken);
    // xSemaphoreGiveFromISR(xDisplaySemaphore, &xHigherPriorityTaskWoken);
    // If a higher priority task was woken, yield
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void Timer4IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // int people = xTaskGetTickCountFromISR();
    //  UARTprintf("\n people:%d", people);
    //
    //  Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    second++;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void ClearBanner(void)
{
    // Construct Banner to wipe previous contents
    sRect.i16XMin = 70;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkKhaki); // fill
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite); // border
    GrRectDraw(&sContext, &sRect);
}

static void ClearText_Sensor_Page_opt(void)
{
    // Construct Banner to wipe previous contents
    sRect_text_for_lux.i16XMin = 0;
    sRect_text_for_lux.i16YMin = (90 + 40);
    sRect_text_for_lux.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect_text_for_lux.i16YMax = 160;
    GrContextForegroundSet(&sContext, ClrBlack); // fill
    GrRectFill(&sContext, &sRect_text_for_lux);
}

static void ClearText_Sensor_Page_bmi(void)
{
    sRect_text_for_lux.i16XMin = 0;
    sRect_text_for_lux.i16YMin = 160;
    sRect_text_for_lux.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect_text_for_lux.i16YMax = 190;
    GrContextForegroundSet(&sContext, ClrBlack); // fill
    GrRectFill(&sContext, &sRect_text_for_lux);
}

// Motor setup page clear banner
static void Clear_Banner_Motor_Text(void)
{
    // Construct Banner to wipe previous contents
    sRect_motor_setup.i16XMin = 0;
    sRect_motor_setup.i16YMin = 75;
    sRect_motor_setup.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect_motor_setup.i16YMax = 75 + 45;
    GrContextForegroundSet(&sContext, ClrBlack); // fill
    GrRectFill(&sContext, &sRect_motor_setup);
}

static void ClearBanner_intro_timer(void)
{
    sRect_motor_setup.i16XMin = 0;
    sRect_motor_setup.i16YMin = 24;
    sRect_motor_setup.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect_motor_setup.i16YMax = 24 + 25;
    GrContextForegroundSet(&sContext, ClrBlack); // fill
    GrRectFill(&sContext, &sRect_motor_setup);
}