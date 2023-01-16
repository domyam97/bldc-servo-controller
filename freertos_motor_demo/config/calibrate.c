/*
 * calibrate.cpp
 *
 *  Created on: Nov 17, 2019
 *      Author: domya
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <float.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "maths/math_ops.h"
#include "drivers/drv8323.h"
#include "drivers/ams_AS5047.h"
#include "config/hardware_config.h"
#include "config/controller_config.h"
#include "utils/sine.h"
#include "utils/foc.h"
#include "config/calibrate.h"

static int g_n = 128 * NPP; // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
static int g_n2 = 50;

static uint32_t g_ui32CurrentOffset[4];

static float v_d = 0.1f;                             //Put all volts on the D-Axis
static float v_q = 0.0f;

volatile float fv_u;
volatile float fv_v;
volatile float fv_w;
volatile float dtc_u;
volatile float dtc_v;
volatile float dtc_w;

volatile float g_error_f[2688];
volatile float g_error_b[2688];


static
void wait_us(uint32_t ui32USeconds)
{
    unsigned long uLDelay = (unsigned long) ui32USeconds * SYS_CLK / 1000000UL;
    SysCtlDelay((uint32_t) uLDelay);
}

static
void Sample(uint32_t *p_Angle, int *p_PrevAngle, int *p_Rotations)
{
    *p_PrevAngle = *p_Angle;
    GetAngle(DAE_CORRECTION, p_Angle);
    if ((int) *p_Angle - *p_PrevAngle > 0x3FFF / 2)
    {
        *p_Rotations -= 1;
    }
    else if ((int) *p_Angle - *p_PrevAngle < -0x3FFF / 2)
    {
        *p_Rotations += 1;
    }

}

static
void MoveCalib(float *p_fErrorArray, int8_t PHASE_ORDER, uint8_t ui32Dir)
{
    const float delta = 2.0f * PI * NPP / (g_n * g_n2);    // change in angle between samples
    uint32_t Angle;
    int i,j;
    int PrevAngle, Rotations;
    float fDir = 1.0f;
    if(!ui32Dir){
        fDir = -1.0f;
    }

    float theta_ref = 0.0f;
    float fi_a, fi_b, fi_c, fi_q, fi_d;

    for (i = 0; i < g_n; i++)
    {                                                   // rotate forwards
        for (j = 0; j < g_n2; j++)
        {
            theta_ref += fDir*delta;
            abc(theta_ref*180.0f/PI, v_d, v_q, &fv_u, &fv_v, &fv_w); // inverse dq0 transform on voltages
            svm(1.0, fv_u, fv_v, fv_w, &dtc_u, &dtc_v, &dtc_w); // space vector modulation
            SetPWM(PWMGEN_A, PWM_FULL * dtc_u);                    // Set duty cycles
            if (PHASE_ORDER)
            {
                SetPWM(PWMGEN_B, PWM_FULL * dtc_v);
                SetPWM(PWMGEN_C, PWM_FULL * dtc_w);
            }
            else
            {
                SetPWM(PWMGEN_C, PWM_FULL * dtc_v);
                SetPWM(PWMGEN_B, PWM_FULL * dtc_w);
            }
            MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
            wait_us(50);
            Sample(&Angle, &PrevAngle, &Rotations);
        }
        Sample(&Angle, &PrevAngle, &Rotations);
        float theta_actual = 2.0f * PI / (float) 0x3FFF
                * (float) (Angle + (Rotations * 0x3FFF));
        // get mechanical position
        p_fErrorArray[i] = theta_ref / NPP - theta_actual;
    }
}

//ben katz
float RunCalibration(int8_t PHASE_ORDER)
{
    uint32_t Angle;
    int PrevAngle, Rotations;
    uint32_t ui32CurrentRead[4];

    float theta_ref = 0.0f;
    float fi_a, fi_b, fi_c, fi_q, fi_d;

    abc(theta_ref*180.0f/PI, v_d, v_q, &fv_u, &fv_v, &fv_w); // inverse dq0 transform on voltages
    svm(1.0f, fv_u, fv_v, fv_w, &dtc_u, &dtc_v, &dtc_w); // space vector modulation
    int i;
    for (i = 0; i < 2000; i++)
    {
        SetPWM(PWMGEN_A, PWM_FULL * dtc_u);                        // Set duty cycles
        if (PHASE_ORDER)
        {
            SetPWM(PWMGEN_B, PWM_FULL * dtc_v);
            SetPWM(PWMGEN_C, PWM_FULL * dtc_w);
        }
        else
        {
            SetPWM(PWMGEN_C, PWM_FULL * dtc_v);
            SetPWM(PWMGEN_B, PWM_FULL * dtc_w);
        }
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        wait_us(100);
    }
    Sample(&Angle, &PrevAngle, &Rotations);

    wait_us(1000);

    uint32_t ui32ElectricalAngle = Angle & 0x3FFF;
    ui32ElectricalAngle = ui32ElectricalAngle
            * float_to_uint(NPP, 0, 0xFFFFFFFF, 32);
    ui32ElectricalAngle = ui32ElectricalAngle % 0x3FFF;
    float ftheta_elec = uint_to_float(ui32ElectricalAngle, 0.0, 360.0, 14);

    ReadCurrent((uint32_t*)&ui32CurrentRead);
    fi_a = current_to_float(ui32CurrentRead[0], g_ui32CurrentOffset[0], 20);
    fi_b = current_to_float(ui32CurrentRead[1], g_ui32CurrentOffset[1], 20);
    fi_c = current_to_float(ui32CurrentRead[2], g_ui32CurrentOffset[2], 20);
    dq0(ftheta_elec, fi_a, fi_b, fi_c, &fi_d, &fi_q); //dq0 transform on currents
    float current = sqrtf(powf(fi_d, 2) + powf(fi_q, 2));

    MoveCalib(&g_error_f, PHASE_ORDER, 1);

    wait_us(1000);

    MoveCalib(&g_error_b, PHASE_ORDER, 0);

    SetPWM(PWMGEN_A, PWM_FULL * 0);                             // Set duty cycles
    SetPWM(PWMGEN_B, PWM_FULL * 0);
    SetPWM(PWMGEN_C, PWM_FULL * 0);
    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);

    float offset = 0.0f;
    for (i = 0; i < g_n; i++)
    {
        offset += (g_error_f[i] + g_error_b[g_n - 1 - i]) / (2.0f * g_n); // calclate average position sensor offset
    }
    offset = fmodf(offset * NPP, 2.0f * PI);

    return offset;

}

int8_t OrderPhases(uint32_t *p_ui32ZeroCurrentOffset)
{
    float theta_ref = 0.0f;
    float theta_actual = 0.0f;
    float fi_a, fi_b, fi_c, fi_q, fi_d;
    int sample_counter = 0;
    float ftheta_elec;
    uint32_t Angle = 0;
    int PrevAngle = 0, Rotations = 0;
    uint32_t ui32Flags1, ui32Flags2;
    uint32_t ui32CurrentRead[4];
    uint32_t ui32CurrentOffset[4];

    int j;
    for (j = 0; j < 4; j++)
    {
        g_ui32CurrentOffset[j] = p_ui32ZeroCurrentOffset[j];
    }
    Sample(&Angle, &PrevAngle, &Rotations);
    Rotations = 0;
    abc(theta_ref*180.0f/PI, v_d, v_q, &fv_u, &fv_v, &fv_w); //inverse dq0 transform on voltages
    svm(1.0, fv_u, fv_v, fv_w, &dtc_u, &dtc_v, &dtc_w);   //space vector modulation
    int i;
    wait_us(10);
    ClearFaults();
    for (i = 0; i < 2000; i++)
    {
        GetFaults(&ui32Flags1, &ui32Flags2);
        SetPWM(PWMGEN_A, PWM_FULL * dtc_u);                        // Set duty cycles
        SetPWM(PWMGEN_B, PWM_FULL * dtc_v);
        SetPWM(PWMGEN_C, PWM_FULL * dtc_w);
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        wait_us(100);
    }
    Sample(&Angle, &PrevAngle, &Rotations);

    GetFaults(&ui32Flags1, &ui32Flags2);

    wait_us(1000);

    float theta_start;

    Sample(&Angle, &PrevAngle, &Rotations);
    uint32_t ui32ElectricalAngle = Angle & 0x3FFF;
    ui32ElectricalAngle = ui32ElectricalAngle * (uint32_t) NPP;
    ui32ElectricalAngle = ui32ElectricalAngle % 0x3FFF;
    ftheta_elec = uint_to_float(ui32ElectricalAngle, 0.0, 360.0, 14);

    ReadCurrent((uint32_t*)&ui32CurrentRead);
    fi_a = current_to_float(ui32CurrentRead[0], g_ui32CurrentOffset[0], 20);
    fi_b = current_to_float(ui32CurrentRead[1], g_ui32CurrentOffset[1], 20);
    fi_c = current_to_float(ui32CurrentRead[2], g_ui32CurrentOffset[2], 20);
    dq0(ftheta_elec, fi_a, fi_b, fi_c, &fi_d, &fi_q);
    float current = sqrtf(powf(fi_d, 2) + powf(fi_q, 2));
    ClearFaults();
    /// Rotate voltage angle
    while (theta_ref < 6.0f*PI)
    {                                           //rotate for 3 electrical cycles
        //ClearFaults();
        abc((theta_ref*180.0f/PI), v_d, v_q, &fv_u, &fv_v, &fv_w); //inverse dq0 transform on voltages
        svm(1.0, fv_u, fv_v, fv_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation
        wait_us(200);
        SetPWM(PWMGEN_A, PWM_FULL * dtc_u);                        // Set duty cycles
        SetPWM(PWMGEN_B, PWM_FULL * dtc_v);
        SetPWM(PWMGEN_C, PWM_FULL * dtc_w);
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        Sample(&Angle, &PrevAngle, &Rotations);         //sample position sensor
        theta_actual = 2.0f * PI / (float) 0x3FFF
                * (float) (Angle + (Rotations * 0x3FFF));
        if(theta_ref == 0)
        {
            theta_start = theta_actual;
        }
        if (sample_counter > 200)
        {
            sample_counter = 0;
        }
        sample_counter++;
        theta_ref += 0.002f;
    }
    float theta_end = 2.0f * PI / (float) 0x3FFF
            * (float) (Angle + (Rotations * 0x3FFF));
    int8_t direction = (theta_end - theta_start) > 0;

    SetPWM(PWMGEN_A, PWM_FULL * 0);                             // Set duty cycles
    SetPWM(PWMGEN_B, PWM_FULL * 0);
    SetPWM(PWMGEN_C, PWM_FULL * 0);
    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);

    GetFaults(&ui32Flags1, &ui32Flags2);
    return direction;
}
