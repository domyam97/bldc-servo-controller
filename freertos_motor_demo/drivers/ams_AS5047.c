/*
 * ams_AS5047.c
 *
 *  Created on: Oct 8, 2019
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
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"
#include "driverlib/ssi.h"
#include "ams_AS5047.h"

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
// The list of possible base addresses for the console SPI.
//
//*****************************************************************************
static uint32_t g_ui32AMSBase = 0;


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
//! Configures the SSI driver.
//!
//! \param ui32PortNum is the number of UART port to use for SSI comms
//! (0-3)
//! \param ui32SSIClock is the frequency of the source clock for the UART
//! module.
//! \param ui32BitRate is the bit rate that the UART is to be configured to use.
//!
//! This function will configure the specified SSI port to be used as a
//! for encoder comms.  The serial parameters are set to the bit rate
//! specified by the \e ui32BitRate parameter and use 16 bit, with parity, SPI_Mode_1
//!
//! This function must be called prior to using any of the other SPI encoder
//! functions.  This function assumes that the
//! caller has previously configured the relevant SSI pins for operation as an
//! SSI rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
void
AMSConfig(uint32_t ui32PortNum, uint32_t ui32SSIClk, uint32_t ui32BitRate)
{
    //
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
    // Select the base address of the UART.
    //
    g_ui32AMSBase = g_ui32SSIBase[ui32PortNum];


    //
    // Enable the UART peripheral for use.
    //
    MAP_SysCtlPeripheralEnable(g_ui32SSIPeriph[ui32PortNum]);

    //
    //Configure SPI settings for encoder. SPI_MODE_1, 16 bit data width.
    //
    MAP_SSIConfigSetExpClk(g_ui32SSIBase[ui32PortNum], ui32SSIClk, SSI_FRF_MOTO_MODE_1,
                           SSI_MODE_MASTER, ui32BitRate, 16);

    //
    // Set the SSI to interrupt whenever the RX or TX FIFO is half full or
    //
    //MAP_SSIIntEnable(g_ui32AMSBase, SSI_TXOR | SSI_RXFF);

    //
    // Disable Interrupts because we don't want to deal with thats
    //
    MAP_SSIIntDisable(g_ui32AMSBase, 0xFFFFFFFF);

    //
    //Configure QEI1 Max count is 7 revolutions of the output at 2000 CPR and 7:1 gear ratio
    //
    ROM_QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B |
                     QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE |
                     QEI_CONFIG_NO_SWAP, 2000*7*7 );

    MAP_QEIEnable(QEI1_BASE);

    //
    // Enable the UART operation.
    //
    MAP_SSIEnable(g_ui32AMSBase);
    MAP_IntMasterEnable();

}

//*****************************************************************************
//
//! Flushes SSI Rx FIFO
//!
//! \param none
//!
//! \return none
//!
//
//*****************************************************************************

static void
FlushSSIRx(){
    uint32_t ui32Throwaway = 0;
    uint32_t ui32Loop = 5;

    ASSERT(g_ui32AMSBase != 0);
    //
    //Clear Rx FIFO
    //
    while(ui32Loop)
    {
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }

    ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);

}

//*****************************************************************************
//
//! Calculates the parity of a given input
//!
//! \param ui32Byte is the data to examined.
//!
//! \return ui32Parity is the parity bit at position 15.
//!
//
//*****************************************************************************
static uint32_t
CalculateParity(uint32_t ui32Byte){
    bool bParity = 0;
    uint32_t n = ui32Byte;

    while(n)
    {
        bParity = !bParity;
        n = n & (n - 1);
    }

    if(bParity)
        return 0x8000;

    else
        return 0;
}

//*****************************************************************************
//
//! Programs the settings register on the encoder.
//!
//! \param ui32Settings1 is the first Setting register value. It is the bitwise OR
//! for settings values defined in ams_AS5047.h and the AMS AS5047 datasheet
//! \param ui32Settings2 is the second Setting register value. It is the bitwise OR
//! for settings values defined in ams_AS5047.h. and the AMS AS5047 datasheet.
//!
//! This function sets the non-volatile settings registers on the AMS AS5047. Only needs to
//! be called once for the device. Does not need to be called every time on boot.
//! To be used to program device on first time startup.
//!
//! Follows settings programming flow in "Burn and Verification of the OTP Memory" in
//! AMS AS5047 datasheet. Skips the last few steps that involve resetting encoder and
//! checking settings registers again.
//!
//! \return Flag if error (-1)
//!
//
//*****************************************************************************
int
ProgramAMSSettings(uint32_t ui32Settings1, uint32_t ui32Settings2){

    uint32_t ui32CheckSet1 = 0, ui32CheckSet2 = 0, ui32CheckProg = 0, ui32Flush=1,
            ui32Throwaway, ui32Loop = 1;


    //
    // Write first settings register (0x0018)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0018) | 0x0018);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(ui32Settings1) | ui32Settings1);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }


    //
    // Write second settings register (0x0019)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0019) | 0x0019);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(ui32Settings2) | ui32Settings2);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }



    //
    //Flush Rx FIFO
    //
    while(ui32Flush){
        ui32Flush = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway); //should be 0
    }
    ui32Flush = 1;

    //
    //Read Settings Registers
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0018) | READ | 0x0018);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0019) | READ | 0x0019);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    MAP_SSIDataGet(g_ui32AMSBase, &ui32Throwaway); //should be 0
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet1); //Good data?
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet2); //Good data?

    ui32CheckSet1 = ui32CheckSet1 & 0x3FFF;
    ui32CheckSet2 = ui32CheckSet2 & 0x3FFF;


    if(!(ui32CheckSet1 == ui32Settings1 &&
            ui32CheckSet2 == ui32Settings2)){

        //
        // Settings failed. Exit.
        //
        return -1;
    }

    //
    // If everything went okay, keep going.
    // Enable OTP read / write by setting PROGEN = 1 in the PROG register
    //

    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003) | 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0001) | 0x0001);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    // Start the OTP burn procedure by setting PROGOTP = 1 in the PROG register (0x0003)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003) | 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0008) | 0x0008);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    while(ui32Loop)
    {
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }

    ui32Loop = 1;

    //
    // Read the PROG register until it reads 0x0001
    //
    uint32_t ui32ProgComp = CalculateParity(0x0001) | 0x0001;
    while(ui32CheckProg != ui32ProgComp){
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0003) | READ | 0x0003);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataPut(g_ui32AMSBase, SSI_NONOP);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway); //returns read adresss 3 command
        MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32CheckProg);
    }

    //
    // Clear the memory content writing 0x00 in the whole non-volatile memory (0x0016 - 0x0019)
    //
    uint32_t ui32Zeros = CalculateParity(0x00) | 0x00;
    uint32_t i = 0;
    for(i = 0x0016; i <= 0x0019; i++){
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(i) | i);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, ui32Zeros);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    FlushSSIRx();

    //
    // Set the PROGVER = 1 to set the guard band for the guard band test
    // In the PROG register (0x0003)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003)| 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0040)| 0x0040);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    FlushSSIRx();

    //
    // Set the PROGVER = 1 to set the guard band for the guard band test
    // In the PROG register (0x0003)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003)| 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0004)| 0x0004);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //

    ui32Loop = 1;

    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }



    //
    //Read Settings Registers
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0018) | READ | 0x0018);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0019) | READ | 0x0019);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway); //should be 0
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32CheckSet1); //Good data?
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32CheckSet2); //Good data!!

    ui32CheckSet1 = ui32CheckSet1 & 0x3FFF;
    ui32CheckSet2 = ui32CheckSet2 & 0x3FFF;


    if(!(ui32CheckSet1 == ui32Settings1 &&
            ui32CheckSet2 == ui32Settings2)){

        //
        // Settings failed. Exit.
        //
        return -1;
    }
    return 0;
}

//*****************************************************************************
//
//! Reads the SSI angle value from the encoder
//!
//! \param ui8Correction is a flag for corrected position reading or not corrected.
//! \param pui32Angle is a pointer to a place to store angle data.
//!
//! The \e ui8Correction defines the type of data to be read from the encoder.
//! The encoder offers Dynamic Angle Error Compensation. \b DAE_CORRECTION reads
//! corrected data. \b NO_DAE_CORRECTION reads uncorrected data.
//!
//! \return Flag if error.
//!
//*****************************************************************************

int
GetAngle(uint8_t ui8Correction, uint32_t *pui32Angle){
    uint32_t ui32AngleReg = 0x3FFE;
    uint32_t ui32AngleVal, ui32Throwaway;
    if(ui8Correction){
        ui32AngleReg = 0x3FFF;
    }

    //
    // Read angle value from Angle Register (0x3FFE or 0x3FFF)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | ui32AngleReg) | READ | ui32AngleReg);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, pui32Angle);

    //
    // Check for error flag
    //
    if(*pui32Angle & 0x4000){
        return -1;
    }

    //
    //Mask returned angle value to remove Parity Bit
    //
    ui32AngleVal = *pui32Angle;
    *pui32Angle = 0x3FFF & ui32AngleVal;

    return 0;
}

//*****************************************************************************
//
//! Reads the error data from the encoder
//!
//! \param pui32Error is a pointer to a place to store Error data.
//!
//! Possible values for \e pui32Error are parity error (0x4), invalid command (0x2),
//! or frame error (0x1).
//!
//! \return void
//!
//*****************************************************************************

void
GetError(uint32_t *pui32Error){
    uint32_t ui32ErrorReg = 0x0001;
    uint32_t ui32ErrorVal, ui32Throwaway;

    //
    //Flush Rx FIFO
    //
    FlushSSIRx();

    //
    // Read Error Register
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | ui32ErrorReg) | READ | ui32ErrorReg);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, pui32Error);

    //
    //Mask returned value to remove parity bit
    //
    ui32ErrorVal = *pui32Error;
    *pui32Error = ui32ErrorVal & 0x3FFF;
}

//*****************************************************************************
//
//! Reads the zero angle value stored by the encoder
//!
//! \param pui32ZeroAngle is a pointer to a place to store angle data.
//!
//! \return Flag if error.
//!
//*****************************************************************************

int
GetZero(uint32_t *pui32ZeroAngle){
    uint32_t ui32ZeroReg = 0x0016;
    uint32_t ui32ZeroAngleValM, ui32ZeroAngleValL, ui32Throwaway;

    //
    // Read angle value from Angle Register (0x3FFE or 0x3FFF)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | ui32ZeroReg) | READ | ui32ZeroReg);
    ui32ZeroReg ++;
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | ui32ZeroReg) | READ | ui32ZeroReg);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32ZeroAngleValM);
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32ZeroAngleValL);

    //
    // Check for error flag
    //
    if(ui32ZeroAngleValL & 0x4000){
        return -1;
    }
    else if (ui32ZeroAngleValM & 0x4000){
        return -1;
    }

    //
    // Do all the masking and shifting
    //
    ui32ZeroAngleValM = ui32ZeroAngleValM & 0x00FF;
    ui32ZeroAngleValM = ui32ZeroAngleValM << 6;

    ui32ZeroAngleValL = ui32ZeroAngleValL & 0x003F;
    *pui32ZeroAngle = ui32ZeroAngleValM | ui32ZeroAngleValL;

    return 0;
}

//*****************************************************************************
//
//! Reads the diagnostics data from the encoder
//!
//! \param pui32Diag is a pointer to a place to store Error data.
//!
//! Possible values for \e pui32Diag are Field Strength Low (0x800), Field Strength High (0x400),
//! or Cordic Overflow (0x200)
//!
//! \return Flag if error
//!
//*****************************************************************************

int
GetDiagnostics(uint32_t *pui32Diagnostics){
    uint32_t ui32DiagReg = 0x3FFC;
    uint32_t ui32DiagVal, ui32Throwaway;

    //
    // Read Diag Register
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | ui32DiagReg) | READ | ui32DiagReg);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    MAP_SSIDataGetNonBlocking(g_ui32AMSBase, pui32Diagnostics);

    //
    // Check for error flag
    //
    if(*pui32Diagnostics & 0x4000){
        return -1;
    }

    //
    // Mask out parity bit and error flags
    //
    ui32DiagVal = *pui32Diagnostics;
    *pui32Diagnostics = ui32DiagVal & 0xFFF;

    return 0;
}

int
SetZeroPos(void)
{
    uint32_t ui32CheckSet1 = 0, ui32CheckSet2 = 0, ui32CheckProg = 0, ui32Flush=1,
                ui32Throwaway, ui32Loop = 1, ui32AngleRead;
    uint32_t ui32Settings1, ui32Settings2, ui32ZeroMSB, ui32ZeroLSB;


    //
    // Read the angle from the encoder
    //
    GetAngle(NO_DAE_CORRECTION, &ui32AngleRead);

    //
    // Format the zero settings bytes
    //
    ui32ZeroMSB = ui32AngleRead & 0x3FC0; // first 8 bits of encoder read - MSB
    ui32ZeroMSB = 0;//ui32ZeroMSB >> 6;

    ui32ZeroLSB = 0;//ui32AngleRead & 0x03F; // last 6 bits of encoder read - LSB
    //
    // Write first zero register (0x0016)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0016) | 0x0016);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(ui32ZeroMSB) | ui32ZeroMSB);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }


    //
    // Write second zero register (0x0017)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0017) | 0x0017);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(ui32ZeroLSB) | ui32ZeroLSB);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }



    //
    //Flush Rx FIFO
    //
    while(ui32Flush){
        ui32Flush = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway); //should be 0
    }
    ui32Flush = 1;

    //
    //Read Settings Registers
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0016) | READ | 0x0016);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0017) | READ | 0x0017);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    MAP_SSIDataGet(g_ui32AMSBase, &ui32Throwaway); //should be 0
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet1); //Good data?
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet2); //Good data?

    ui32CheckSet1 = ui32CheckSet1 & 0x3FFF;
    ui32CheckSet2 = ui32CheckSet2 & 0x3FFF;


    if(!(ui32CheckSet1 == ui32ZeroMSB &&
            ui32CheckSet2 == ui32ZeroLSB)){

        //
        // Settings failed. Exit.
        //
        return -1;
    }

    //
    // If everything went okay, keep going.
    // Enable OTP read / write by setting PROGEN = 1 in the PROG register
    //

    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003) | 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0001) | 0x0001);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    // Start the OTP burn procedure by setting PROGOTP = 1 in the PROG register (0x0003)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003) | 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0008) | 0x0008);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    while(ui32Loop)
    {
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }

    ui32Loop = 1;

    //
    // Read the PROG register until it reads 0x0001
    //
    uint32_t ui32ProgComp = CalculateParity(0x0001) | 0x0001;
    while(ui32CheckProg != ui32ProgComp){
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0003) | READ | 0x0003);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataPut(g_ui32AMSBase, SSI_NONOP);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway); //returns read adresss 3 command
        MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32CheckProg);
    }

    //
    // Clear the memory content writing 0x00 in the whole non-volatile memory (0x0016 - 0x0019)
    //
    uint32_t ui32Zeros = CalculateParity(0x00) | 0x00;
    uint32_t i = 0;
    for(i = 0x0016; i <= 0x0019; i++){
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(i) | i);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
        MAP_SSIDataPutNonBlocking(g_ui32AMSBase, ui32Zeros);
        while(MAP_SSIBusy(g_ui32AMSBase)){
        }
    }
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    FlushSSIRx();

    //
    // Set the PROGVER = 1 to set the guard band for the guard band test
    // In the PROG register (0x0003)
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003) | 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0040) | 0x0040);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //
    ui32Loop = 1;

    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }

    //
    // Refresh the non-volatile memory content with the OTP
    // content by setting OTPREF = 1
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0003)| 0x0003);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(0x0004)| 0x0004);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    //
    //Flush Rx FIFO
    //

    ui32Loop = 1;

    while(ui32Loop){
        ui32Loop = MAP_SSIDataGetNonBlocking(g_ui32AMSBase, &ui32Throwaway);
    }



    //
    //Read Settings Registers
    //
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0016) | READ | 0x0016);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0017) | READ | 0x0017);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, CalculateParity(READ | 0x0019) | READ | 0x0019);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }
    MAP_SSIDataPutNonBlocking(g_ui32AMSBase, SSI_NONOP);
    while(MAP_SSIBusy(g_ui32AMSBase)){
    }

    MAP_SSIDataGet(g_ui32AMSBase, &ui32Throwaway); //should be 0
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet1); //Good data?
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckSet2); //Good data!!
    MAP_SSIDataGet(g_ui32AMSBase, &ui32CheckProg); //Good data!!

    ui32CheckSet1 = ui32CheckSet1 & 0x3FFF;
    ui32CheckSet2 = ui32CheckSet2 & 0x3FFF;
    ui32CheckProg = ui32CheckProg & 0x3FFF;

    if(ui32CheckProg != DEFAULT_SET_2){
           while(1){
           }
       }


    if(!(ui32CheckSet1 == ui32ZeroMSB &&
            ui32CheckSet2 == ui32ZeroLSB)){

        //
        // Settings failed. Exit.
        //
        return -1;
    }
    return 0;
}
