/*
 * drv8323.c
 *
 *  Created on: Oct 31, 2019
 *      Author: domya
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "drv8323.h"


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The list of possible base addresses for the console SPI.
//
//*****************************************************************************
static const uint32_t g_ui32SSIBase[4] =
{
    SSI0_BASE, SSI1_BASE, SSI2_BASE, SSI3_BASE
};

//*****************************************************************************
//
// The list of SSI peripherals.
//
//*****************************************************************************
static const uint32_t g_ui32SSIPeriph[4] =
{
    SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1, SYSCTL_PERIPH_SSI2,
    SYSCTL_PERIPH_SSI3
};

//*****************************************************************************
//
// The list of possible base addresses for the console SPI.
//
//*****************************************************************************
static uint32_t g_ui32DRVBase = 0;

//*****************************************************************************
//
// The list of possible base addresses for the console SPI.
//
//*****************************************************************************
//static uint32_t g_ui32PWMClock = 0;

//*****************************************************************************
//
//! Configures the SSI driver.
//!
//! \param ui32PortNum is the number of SSI port to use for SSI comms
//! (0-3)
//! \param ui32SSIClock is the frequency of the source clock for the DRV
//! module.
//! \param ui32BitRate is the bit rate that the SSI is to be configured to use.
//!
//! This function will configure the specified SSI port to be used as a
//! for driver comms.  The serial parameters are set to the bit rate
//! specified by the \e ui32BitRate parameter and use 16 bit, with parity, SPI_Mode_1
//!
//! This function must be called prior to using any of the other SPI driver
//! settings.  This function assumes that the
//! caller has previously configured the relevant SSI pins for operation as an
//! SSI rather than as GPIOs. This function also sets up hardware for driving and
//! current sensing.
//!
//! \return None.
//
//*****************************************************************************
void
DRVConfig(uint32_t ui32PortNum, uint32_t ui32SSIClk, uint32_t ui32BitRate)
{
    // Check the arguments.
    //
    ASSERT((ui32PortNum == 0) || (ui32PortNum == 1) ||
           (ui32PortNum == 2) || (ui32PortNum == 3));

    //
    // Check to make sure the SPI peripheral is present.
    //
    if(!MAP_SysCtlPeripheralPresent(g_ui32SSIPeriph[ui32PortNum]))
    {
        return;
    }

    //
    // Select the base address of the SPI bus.
    //
    g_ui32DRVBase = g_ui32SSIBase[ui32PortNum];

    //g_ui32PWMClock = MAP_SysCtlPWMClockGet();


    //
    // Enable the SPI peripheral for use.
    //
    MAP_SysCtlPeripheralEnable(g_ui32SSIPeriph[ui32PortNum]);

    //
    //Configure SPI settings for encoder. SPI_MODE_1, 16 bit data width.
    //
    MAP_SSIConfigSetExpClk(g_ui32SSIBase[ui32PortNum], ui32SSIClk, SSI_FRF_MOTO_MODE_1,
                           SSI_MODE_MASTER, ui32BitRate, 16);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlDelay(1000);
    //
    //Configure the ADC and Read pattern
    //
    MAP_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0); //phase A
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1); //phase B
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2| ADC_CTL_END | ADC_CTL_IE); //phase C and stop, set interrupt flag


    MAP_ADCSequenceEnable(ADC0_BASE, 1);
    MAP_ADCIntClear(ADC0_BASE, 1);

    //
    //Configure PWMs
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    MAP_PWMGenConfigure(PWM0_BASE, PWMGEN_A, PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN |
                        PWM_GEN_MODE_DB_NO_SYNC | PWM_GEN_MODE_DOWN);
    MAP_PWMGenConfigure(PWM0_BASE, PWMGEN_B, PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN |
                        PWM_GEN_MODE_DB_NO_SYNC | PWM_GEN_MODE_DOWN);
    MAP_PWMGenConfigure(PWM0_BASE, PWMGEN_C, PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN |
                        PWM_GEN_MODE_DB_NO_SYNC | PWM_GEN_MODE_DOWN);

    MAP_PWMGenPeriodSet(PWM0_BASE, PWMGEN_A, PWM_GEN_PERIOD); //16kHz PWM output
    MAP_PWMGenPeriodSet(PWM0_BASE, PWMGEN_B, PWM_GEN_PERIOD);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWMGEN_C, PWM_GEN_PERIOD);

    //
    //Enable the PWM Outs
    //
    MAP_PWMGenEnable(PWM0_BASE, PWMGEN_A);
    MAP_PWMGenEnable(PWM0_BASE, PWMGEN_B);
    MAP_PWMGenEnable(PWM0_BASE, PWMGEN_C);

    SetPWM(PWMGEN_A, 0);
    SetPWM(PWMGEN_B, 0);
    SetPWM(PWMGEN_C, 0);
    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);

    MAP_PWMSyncTimeBase(PWM0_BASE, PWM_GEN_BITS);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_3_BIT
                       | PWM_OUT_5_BIT, true);


    MAP_PWMSyncUpdate(PWM0_BASE,PWM_GEN_BITS);




    // Disable Interrupts because we don't want to deal with that
    //
    MAP_SSIIntDisable(g_ui32DRVBase, 0xFFFFFFFF);

    //
    // Enable the SSI operation.
    //
    MAP_SSIEnable(g_ui32DRVBase);
    MAP_IntMasterEnable();

    //
    //Enable the DRV Chip
    //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6,
                     GPIO_PIN_4 | GPIO_PIN_6);
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

}

void
WriteRegister(uint8_t ui8Address, uint32_t ui32Data)
{
    //
    //writes SPI message in the proper format for DRV Chip
    //
    uint32_t ui32Message;
    ui32Message = (ui8Address << 11) | ui32Data;
    MAP_SSIDataPutNonBlocking(g_ui32DRVBase, ui32Message);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }
}

//*****************************************************************************
//
//! Configures the SSI driver.
//!
//! \param ui32PWMGen is the bit representation of the PWM generator.
//! \param ui32Width is the (0-255) duty cycle width for PWM output.
//!
//! This function will...
//!
//! \return None.
//
//*****************************************************************************

void
SetPWM(uint32_t ui32PWMGen, uint32_t ui32Width)
{
    if(ui32Width == 0){
        ui32Width = 1;
    }
    if(ui32PWMGen == PWM_GEN_0)
    {
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui32Width*PWM_GEN_PERIOD/255);
    }
    else if(ui32PWMGen == PWM_GEN_1)
    {
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui32Width*PWM_GEN_PERIOD/255);
    }
    else if(ui32PWMGen == PWM_GEN_2)
    {
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, ui32Width*PWM_GEN_PERIOD/255);
    }
}

void
DisablePWM(void)
{
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_3_BIT
                       | PWM_OUT_5_BIT, false);
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6,
                     0);
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
}

void
ReadRegister(uint8_t ui8Address, uint32_t *pui32Data)
{
    uint32_t ui32Message, ui32Throwaway;
    ui32Message = 0x8000 | (ui8Address << 11);

    MAP_SSIDataPutNonBlocking(g_ui32DRVBase, ui32Message);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32DRVBase, 0x0000);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }
    MAP_SSIDataGetNonBlocking(g_ui32DRVBase, pui32Data);
    MAP_SSIDataGetNonBlocking(g_ui32DRVBase, &ui32Throwaway);
}

void
DisableDRV(void)
{
    //
    //Disable the DRV Chip
    //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    SysCtlDelay(10000);
}

void
EnableDRV(void)
{
    //
    //Enable the DRV Chip
    //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    ProgramDRVSettings();
}

void
ProgramDRVSettings(void)
{
    uint32_t ui32Loop = 5, ui32Throwaway;

    WriteRegister(0x02, 0x020);

    WriteRegister(0x03, 0x77);

    WriteRegister(0x04, 0x377);

    WriteRegister(0x05, 0x155);

    WriteRegister(0x06, SEN_SETS);

    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(SSI0_BASE, &ui32Throwaway);
    }
}
void
ReadCurrent(uint32_t *pui32ReadBuf)
{
    MAP_ADCProcessorTrigger(ADC0_BASE, 1);
    while(!MAP_ADCIntStatus(ADC0_BASE, 1, false))
    {
    }
    MAP_ADCIntClear(ADC0_BASE, 1);
    MAP_ADCSequenceDataGet(ADC0_BASE, 1, pui32ReadBuf);
}

void
ClearFaults(void)
{
    uint32_t ui32Loop = 5, ui32Throwaway;
    WriteRegister(0x02, 0x001);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }
    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(SSI0_BASE, &ui32Throwaway);
    }
}

void
ZeroCurrent(uint32_t *pui32ZeroBuf, uint32_t ui32SenSets)
{
    uint32_t ui32Loop = 5, ui32Throwaway;
    //
    //Write PWM's to Zero
    //
    SetPWM(PWMGEN_A, 0);
    SetPWM(PWMGEN_B, 0);
    SetPWM(PWMGEN_C, 0);

    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);

    //
    //Write Motor Driver Current sense shorted to zero.
    //
    WriteRegister(0x06, ui32SenSets | 0x1C);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }

    //
    //Read the zero current offset
    //
    ReadCurrent(pui32ZeroBuf);

    WriteRegister(0x06, ui32SenSets);
    while(MAP_SSIBusy(g_ui32DRVBase)){
    }

    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32DRVBase, &ui32Throwaway);
    }
}

void
GetFaults(uint32_t *pui32Fault1, uint32_t *pui32Fault2)
{
    ReadRegister(0x00, pui32Fault1);
    ReadRegister(0x01, pui32Fault2);
}
