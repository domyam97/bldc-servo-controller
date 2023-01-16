/*
 * math_ops.cpp
 *
 *  Created on: Oct 31, 2019
 *      Author: Ben Katz
 *      Project: Hobbyking_Cheetah_Compact
 *      Link: https://os.mbed.com/users/benkatz/code/Hobbyking_Cheetah_Compact/file/6cc428f3431d/math_ops.cpp/
 */

#include "maths/math_ops.h"
#include "config/hardware_config.h"

float fmaxf(float x, float y)
{
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y));
}

float fminf(float x, float y)
{
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z)
{
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z)
{
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

float roundf(float x)
{
    /// Returns nearest integer ///

    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
}

void limit_norm(float *x, float *y, float limit)
{
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit)
    {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t) ((x - offset) * ((float) ((uint32_t)((1 << bits) - 1))) / span);
}

float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((uint32_t)((1 << bits) - 1))) + offset;
}

float current_to_float(uint32_t reading, uint32_t offset, float gain)
{
    float f_reading = ((float) reading) / 4095.0f * 3.3f;
    float f_offset = ((float) offset) / 4095.0f * 3.3f;
    /// reading in mV from -Vref/2 to Vref/2 ///
    f_reading = f_offset - f_reading;
    /// reading in A ///
    return (f_reading / R_SENSE / gain);
}
}
