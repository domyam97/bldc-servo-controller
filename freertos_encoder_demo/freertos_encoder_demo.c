//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "config/calibrate.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/qei.h"
#include "drivers/ams_AS5047.h"
#include "drivers/drv8323.h"
#include "utils/uartstdio.h"
#include "utils/rs485.h"
#include "comms_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************

#define ID                  2

#ifdef BREAKOUT
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

#endif

//*****************************************************************************
//
// The mutex that protects concurrent access of RS485 from multiple tasks.
//
//*****************************************************************************

xSemaphoreHandle g_pRS485Semaphore;

//*****************************************************************************
//
// The mutex that protects concurrent access of RS485 from multiple tasks.
//
//*****************************************************************************

xSemaphoreHandle g_pEncoderSemaphore;

uint32_t ui32CurrZeroBuff[4];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

#ifdef BREAKOUT

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

#endif

//*****************************************************************************
//
// Configure the encoder SPI and its pins.
//
//*****************************************************************************
void
ConfigureEnc(void)
{

    //
    // Enable the GPIO ports that SSI1 uses
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Enable the GPIO ports for quadrature
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //
    // Enable SSI1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // Enable QEI
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //
    // Configure GPIO Pins for SSI1 mode.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    ROM_GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    ROM_GPIOPinConfigure(GPIO_PD2_SSI1RX);
    ROM_GPIOPinConfigure(GPIO_PD3_SSI1TX);
    ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //
    //Configure pins for QEI
    //
    ROM_GPIOPinConfigure(GPIO_PC5_PHA1);
    ROM_GPIOPinConfigure(GPIO_PC6_PHB1);
    ROM_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);


    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    ROM_SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_PIOSC);

    //
    // Configure Encoder
    //
    AMSConfig(1, 16000000,5000000);
}

//*****************************************************************************
//
// Configure the RS485 and its pins.
//
//*****************************************************************************
void
ConfigureRS485(void)
{
    //
    // Enable the GPIO Peripheral used by the UART1.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


    //
    // Enable UART1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);


    //
    // Enable RS485 Control GPIO
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure GPIO Pins for UART1 mode.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure GPIO Pins for RS485 control mode.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    RS485Config(1, 921600, 16000000, 3, GPIO_PIN_4);
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Error;

    //
    // Set the clocking to run at 80 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

#ifdef BREAKOUT
    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ConfigureUART();

    //
    // Print demo introduction.
    //
    UARTprintf("\n\nWelcome to the EK-TM4C123GXL FreeRTOS Demo!\n");

    g_pUARTSemaphore = xSemaphoreCreateMutex();

#endif

    //
    // Initialize the RS485 and configure it for 115,200, 8-N-1 operation.
    //

    ConfigureRS485();

    //
    // Initialize SPI for the encoder and QEI.
    //
    ConfigureEnc();

    GetDiagnostics(&ui32Error);



    //
    // Create a mutex to guard the UART.
    //

    g_pRS485Semaphore = xSemaphoreCreateMutex();
    g_pEncoderSemaphore = xSemaphoreCreateMutex();

    GetError(&ui32Error);

    //
    // Create the comms task.
    //
    if(CommsTaskInit(ID) != 0)
    {

        while(1)
        {
        }
    }

    //
    // Create the encoder task.
    //
    if(EncoderTaskInit() != 0)
    {

        while(1)
        {
        }
    }


    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
    //UARTprintf("\n\nFailed to init.\n");

    while(1)
    {
    }
}
