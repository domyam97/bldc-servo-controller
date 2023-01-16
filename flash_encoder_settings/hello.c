#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/qei.h"
#include "ams_AS5047.h"


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

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    ROM_SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_PIOSC);

    //
    //Configure QEI1
    //
    ROM_QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B |
                     QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |
                     QEI_CONFIG_NO_SWAP, 0);
    //
    // Configure Encoder
    //
    AMSConfig(1, 16000000,5000000);
}

/**
 * hello.c
 */
int main(void)
{
	uint32_t ui32Diag, ui32Angle =0;
	uint32_t ui32Mess = 0x8000 | READ | 0x0018;
    uint32_t ui32Throwaway = 0;
    uint32_t ui32Loop = 5;

	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
	                       SYSCTL_OSC_MAIN);

    ConfigureEnc();

/*
    //Settings Check
    //check 1
    ROM_SSIDataPutNonBlocking(SSI1_BASE, ui32Mess);
    while(ROM_SSIBusy(SSI1_BASE)){
    }

    //read error
    ROM_SSIDataPutNonBlocking(SSI1_BASE, READ | 0x0001);
    while(ROM_SSIBusy(SSI1_BASE)){
    }

    //check
    ROM_SSIDataPutNonBlocking(SSI1_BASE, ui32Mess);
    while(ROM_SSIBusy(SSI1_BASE)){
    }

    //check 2
    ROM_SSIDataPut(SSI1_BASE, READ | 0x0019);
    while(ROM_SSIBusy(SSI1_BASE)){
    }

    //check 3
    ROM_SSIDataPut(SSI1_BASE, 0xC000);
    while(ROM_SSIBusy(SSI1_BASE)){
    }


    while(ui32Loop)
    {
        ui32Loop = ROM_SSIDataGetNonBlocking(SSI1_BASE, &ui32Throwaway);
    }

    ui32Loop = ROM_SSIDataGetNonBlocking(SSI1_BASE, &ui32Throwaway);

    GetDiagnostics(&ui32Diag);
    GetZero(&ui32Angle);
    while(1)
    {
    GetAngle(DAE_CORRECTION, &ui32Angle);

    ui32Angle = ui32Angle;
    }
*/

    GetError(&ui32Throwaway);

    //
    // Programming Settings can only be done once per chip
    // Make sure this is the zero value you want to use.
    //

    if(ProgramAMSSettings(DEFAULT_SET_1, DEFAULT_SET_2) != 0)
    {
        while(1){
        }
    }
    GetDiagnostics(&ui32Diag);
    GetAngle(DAE_CORRECTION, &ui32Angle);
    GetZero(&ui32Diag);
    //
    // Programming Zero can only be done once per chip
    // Make sure this is the zero value you want to use.
    //
    /*if(SetZeroPos() != 0)
    {
        while(1){
        }
    }
    GetZero(&ui32Angle);
    */

    while(1)
    {
    GetAngle(DAE_CORRECTION, &ui32Angle);

    ui32Angle = ui32Angle;
    }

}
