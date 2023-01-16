#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "drv8323.h"

//*****************************************************************************
//
// Configure the encoder SPI and its pins.
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
}

/**
 * hello.c
 */
int main(void)
{
    uint32_t ui32HighSide = 0, ui32LowSide = 0, ui32Loop=5, ui32Throwaway;
    uint32_t ui32Fault1, ui32Fault2;
    uint32_t ui32CurrBuff[4];
    uint32_t ui32USeconds = 1000;
    int i;

    //80 MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                               SYSCTL_OSC_MAIN);

    ConfigureDRV();

    //DisableDRV();

    //EnableDRV();

    SysCtlDelay(1200*80);

    //ClearFaults();

    ProgramDRVSettings();

    GetFaults(&ui32Fault1, &ui32Fault2);

    ReadRegister(0x02, &ui32HighSide);

    ReadRegister(0x04, &ui32LowSide);

    DisableDRV();
    //
    //this code cycles the PWM's so that you can check the outputs on an oscilloscope or multimeter.
    //

    /*
    while(1)
    {
        if(i > 2){
            i=0;
        }
        ReadCurrent(&ui32CurrBuff);
        //GetFaults(&ui32Fault1, &ui32Fault2);
        //ClearFaults();

        if(i==0){
            SetPWM(PWMGEN_C, 105);
            SetPWM(PWMGEN_B, 125);
            SetPWM(PWMGEN_A, 125);
            ROM_PWMSyncUpdate(PWM0_BASE,PWM_GEN_BITS);
        }
        else if(i==1){
            SetPWM(PWMGEN_C, 125);
            SetPWM(PWMGEN_B, 105);
            SetPWM(PWMGEN_A, 125);
            ROM_PWMSyncUpdate(PWM0_BASE,PWM_GEN_BITS);
        }
        else if(i==2){
            SetPWM(PWMGEN_C, 125);
            SetPWM(PWMGEN_B, 125);
            SetPWM(PWMGEN_A, 105);
            ROM_PWMSyncUpdate(PWM0_BASE,PWM_GEN_BITS);
        }
        SysCtlDelay(ui32USeconds*80);
        i++;
    }

    */

}
