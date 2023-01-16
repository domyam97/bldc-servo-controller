/*
 * drv8323.h
 *
 *  Created on: Oct 31, 2019
 *      Author: domya
 */

#ifndef DRIVERS_DRV8323_H_
#define DRIVERS_DRV8323_H_

#include <stdarg.h>
#include "driverlib/pwm.h"

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

#define PWM_GEN_BITS                PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT

#define SEN_SETS                    0x283

#define PWMGEN_A                    PWM_GEN_0
#define PWMGEN_B                    PWM_GEN_1
#define PWMGEN_C                    PWM_GEN_2
#define PWM_GEN_PERIOD              1333

extern void SetPWM(uint32_t ui32PWMGen, uint32_t ui32Width);
extern void WriteRegister(uint8_t ui8Address, uint32_t ui32Data);
extern void ReadRegister(uint8_t ui8Address, uint32_t *pui32Data);
extern void ReadCurrent(uint32_t *pui32ReadBuf);
extern void DisableDRV(void);
extern void EnableDRV(void);
extern void ProgramDRVSettings(void);
extern void ClearFaults(void);
extern void ZeroCurrent(uint32_t *pui32ZeroBuf, uint32_t ui32SenSets);
extern void GetFaults(uint32_t *pui32Fault1, uint32_t *pui32Fault2);
extern void DisablePWM(void);

//Tiva HW config for SSI. mode, protocol, and data width are pre set for DRV hw.
extern void DRVConfig(uint32_t ui32PortNum, uint32_t ui32SSIClk, uint32_t ui32BitRate);


#endif /* DRIVERS_DRV8323_H_ */
