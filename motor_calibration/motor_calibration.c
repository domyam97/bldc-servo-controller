/*
 * motor_calibration.c
 *
 *  Created on: Nov 17, 2019
 *      Author: domya
 */

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


uint32_t ui32CurrZeroBuff[4];

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
    RS485Config(1, 115200, 16000000, 3, GPIO_PIN_4);
}

//*****************************************************************************
//
// Configure the DRV8323 SPI and its pins.
//
//*****************************************************************************
void ConfigureDRV(void)
{

    //
    // Enable the GPIO ports that SSI1 uses
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable the GPIO ports for driving and sensing
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Enable SSI0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure GPIO Pins for SSI0 mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE,
                       GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Configure GPIO Pins for DRV enable.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_INT_PIN_6);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

    //
    //Configure GPIO Pins for driving
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    ROM_GPIOPinConfigure(GPIO_PE5_M0PWM5);
    ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
    ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE,
                       GPIO_PIN_5 | GPIO_PIN_7);
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE,
                       GPIO_PIN_5);

    //
    //Setup ADC's
    //
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 );
    //
    // Use the internal 16MHz oscillator as the SSI clock source.
    //
    ROM_SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_PIOSC);

    //
    // Set the PWM clock at SYSCLK/2 = 40 MHz
    //
    ROM_SysCtlPWMClockSet(PWM_SYSCLK_DIV_2);

    //
    // Configure DRV SSI PORT, CLK SPD, BIT_RATE
    //
    DRVConfig(0, 16000000, 1000000);
    ProgramDRVSettings();
}

int
main(void)
{
    uint32_t ui32CurrZeroRead[4];
    uint32_t ui32Throwaway1 = 0, ui32Throwaway2 = 0;
    //
    // Set the clocking to run at 80 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();


    //
    // Initialize the RS485 and configure it for 115,200, 8-N-1 operation.
    //

    //ConfigureRS485();

    //
    // Initialize SPI for the encoder and QEI.
    //
    ConfigureEnc();

    //
    //Initialize DRV chip
    //
    ConfigureDRV();
    ClearFaults();

    //
    //Get ADC Zero offset
    //
    int n = 50;
    int i,j;
    for(i = 0; i < n; i++){
        ZeroCurrent(&ui32CurrZeroRead, SEN_SETS); //should probably get rid of ui32SenSets input. Should just read current register settings.
        for(j = 0; j < 4; j++){
            ui32CurrZeroBuff[j] = (ui32CurrZeroBuff[j] + ui32CurrZeroRead[j])/2;
        }
    }

    ReadRegister(0x03, &ui32Throwaway1);
    ReadRegister(0x05, &ui32Throwaway2);
    GetFaults(&ui32Throwaway1, &ui32Throwaway2);

    int8_t PHASE_ORDER = OrderPhases((uint32_t*)&ui32CurrZeroBuff);


    ROM_SysCtlDelay(1000 * SYS_CLK / 1000000UL);

    float elec_offset = RunCalibration(PHASE_ORDER);

    while(1)
    {
    }
}

