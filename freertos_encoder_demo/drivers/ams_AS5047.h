/*
 * ams_AS5047.h
 *
 *  Created on: Sep 10, 2019
 *      Author: domya
 */

#ifndef DRIVERS_AMS_AS5047_H_
#define DRIVERS_AMS_AS5047_H_

#include <stdarg.h>

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
// Constants for settings and stuff
//
//*****************************************************************************

#define READ                        0x4000
#define SSI_NONOP                   0xC000

#define DAE_CORRECTION              0x01 //reads corrected angle
#define NO_DAE_CORRECTION           0x00 //reads uncorrected angle

#define USE_ABI                     0x00000000
#define USE_UVW                     0x00000008

#define INVERT_DIR                  0x00000004 //reverses direction the encoder reads

#define DAE_ON                      0x00000000 //default. USE Dynamic Angle Error Compensation
#define DAE_OFF                     0x00000010 //better have a good reason for this

#define ENABLE_PWM                  0x00000010 //requires USE_ABI or USE_UVW to be set


//*****************************************************************************
//
// Constants for UVW Pole Pairs
//
//*****************************************************************************

#define UVW_PP_1                    0x00000000
#define UVW_PP_2                    0x00000001
#define UVW_PP_3                    0x00000002
#define UVW_PP_4                    0x00000003
#define UVW_PP_5                    0x00000004
#define UVW_PP_6                    0x00000005
#define UVW_PP_7                    0x00000006

//*****************************************************************************
//
// Constants for ABI Resolution
//
//*****************************************************************************

#define ABI_DEC                     0x00000000 //decimal increments cpr
#define ABI_BIN                     0x00000020 //binary increments cpr
#define ABI_RES_2000                0x00000000
#define ABI_RES_1600                0x00000020
#define ABI_RES_1200                0x00000040
#define ABI_RES_800                 0x00000060
#define ABI_RES_400                 0x00000080
#define ABI_RES_200                 0x000000a0
#define ABI_RES_100                 0x000000c0
#define ABI_RES_32                  0x000000e0
#define ABI_RES_2048                0x00000000
#define ABI_RES_1024                0x00000020

//*****************************************************************************
//
// Constants for Hysteresis
// These constants are dependent on the value of ABI Res
// The data sheet doesn't talk a whole lot about what changing hysteresis values actually does.
//
//*****************************************************************************

#define HYSTERESIS_1                0x00000000
#define HYSTERESIS_2                0x00000008
#define HYSTERESIS_3                0x00000010
#define HYSTERESIS_4                0x00000018

//*****************************************************************************
//
// Default setting values
//
//*****************************************************************************

#define DEFAULT_SET_1               (USE_ABI | ABI_BIN | 0x0001)
#define DEFAULT_SET_2               (HYSTERESIS_2 | ABI_RES_2048)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern int SetZeroPos(void);
extern int ProgramAMSSettings(uint32_t ui32Settings1, uint32_t ui32Settings2);
extern int GetAngle(uint8_t ui8Correction, uint32_t *pui32Angle);
extern void GetError(uint32_t *pui32Error);
extern int GetDiagnostics(uint32_t *pui32Diagnostics);
extern int GetZero(uint32_t *pui32ZeroAngle);

//Tiva HW config for SSI. mode, protocol, and data width are pre set for encoder hw.
extern void AMSConfig(uint32_t ui32PortNum, uint32_t ui32SSIClk, uint32_t ui32BitRate);


#endif /* DRIVERS_AMS_AS5047_H_*/
