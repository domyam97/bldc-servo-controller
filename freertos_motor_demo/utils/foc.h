/*
 * foc.h
 *
 *  Created on: Nov 11, 2019
 *      Author: domya
 */

#ifndef UTILS_FOC_H_
#define UTILS_FOC_H_

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

#define DTC_MAX                 0.95
#define DTC_MIN                 0.01

#define DEG_360                  0xFFF00000
#define DEG_90                   DEG_360/4 + 1

extern void abc( float theta, float d, float q, float *a, float *b, float *c);
extern void dq0(float theta, float a, float b, float c, float *d, float *q);
extern void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);



#endif /* UTILS_FOC_H_ */
