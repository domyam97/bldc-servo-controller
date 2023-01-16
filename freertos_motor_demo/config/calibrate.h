/*
 * calibrate.h
 *
 *  Created on: Nov 11, 2019
 *      Author: domya
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

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

#define SYS_CLK         800000000UL

extern int8_t OrderPhases(uint32_t *p_ui32CurrentZeroOffset);
extern float RunCalibration(int8_t PHASE_ORDER);


#endif /* CALIBRATE_H_ */
