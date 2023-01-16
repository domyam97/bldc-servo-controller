/*
 * foc.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: domya
 */

#include <stdint.h>
#include <stdarg.h>
#include <float.h>
#include "maths/math_ops.h"
#include "config/hardware_config.h"
#include "config/controller_config.h"
#include "utils/sine.h"
#include "utils/foc.h"

//*******************************************************************************
//
//      Author: Ben Katz
//      Project: Hobbyking_Cheetah_Compact
//
//*******************************************************************************
void
abc(float theta, float d, float q, float *a, float *b, float *c){
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    while (theta < 0.0f){
        theta += 360.0f;
    }

    theta = fmodf(theta, 360.0f);

    uint32_t ui32Theta = float_to_uint(theta, 0, 360, 32);

    int32_t sin = sine(ui32Theta);
    int32_t cos = sine(ui32Theta + DEG_90);

    float cf = ((float)cos)/65535.0f;
    float sf = ((float)sin)/65535.0f;

    *a = cf * (d) - sf * (q);                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf) * (d) - (-0.86602540378f*cf-.5f*sf) * (q);
    *c = (-0.86602540378f*sf-.5f*cf) * (d) - (0.86602540378f*cf-.5f*sf) * (q);
}

//*******************************************************************************
//
//      Author: Ben Katz
//      Project: Hobbyking_Cheetah_Compact
//
//*******************************************************************************
void
dq0(float theta, float a, float b, float c, float *d, float *q){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    while (theta < 0.0f){
            theta += 6.28318530718f;
    }
    theta = fmodf(theta, 360.0f);

    uint32_t ui32Theta = float_to_uint(theta, 0, 360, 32);

    int32_t sin = sine(ui32Theta);
    int32_t cos = sine(ui32Theta + DEG_90);

    float cf = ((float)cos)/65535.0f;
    float sf = ((float)sin)/65535.0f;

    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);

}

//*******************************************************************************
//
//      Author: Ben Katz
//      Project: Hobbyking_Cheetah_Compact
//
//*******************************************************************************
void
svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))/2.0f;
    *dtc_u = fminf(fmaxf(((u -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);

}
}
}



