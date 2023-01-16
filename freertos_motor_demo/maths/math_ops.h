/*
 * math_ops.h
 *
 *  Created on: Oct 31, 2019
 *      Author: domya
 */

#ifndef MATHS_MATH_OPS_H_
#define MATHS_MATH_OPS_H_

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

#define PI 3.14159265359f
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
float roundf(float x);
void limit_norm(float *x, float *y, float limit);
uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits);
float current_to_float(uint32_t reading, uint32_t offset, float gain);



#endif /* MATHS_MATH_OPS_H_ */
