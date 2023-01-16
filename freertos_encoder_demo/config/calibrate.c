/*
 * calibrate.c
 *
 *  Created on: Nov 11, 2019
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

static
void wait_us(uint32_t ui32USeconds)
{
    unsigned long uLDelay = (unsigned long)ui32USeconds*SYS_CLK/1000000UL;
    SysCtlDelay((uint32_t)uLDelay);
}

static
void Sample(uint32_t *p_Angle, int *p_PrevAngle, int *p_Rotations)
{
   *p_PrevAngle = *p_Angle;
   GetAngle(DAE_CORRECTION, p_Angle);
   if(*p_Angle - *p_PrevAngle > 0x3FFF/2){
       *p_Rotations -= 1;
       }
   else if ((int)*p_Angle - *p_PrevAngle < -0x3FFF/2){
       *p_Rotations += 1;
       }

}

int OrderPhases(uint32_t *p_ui32ZeroCurrentOffset)
{
    float theta_ref = 0.0f;
    float theta_actual = 0.0f;
    float v_d = 0.1f;                                                             //Put all volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0.0f;
    float dtc_u, dtc_v, dtc_w = .5f;
    float fi_a, fi_b, fi_c, fi_q, fi_d;
    int sample_counter = 0;
    float ftheta_elec;
    uint32_t Angle=0;
    int PrevAngle=0, Rotations=0;
    uint32_t ui32Flags1, ui32Flags2;
    uint32_t ui32CurrentRead[4];
    uint32_t ui32CurrentOffset[4];

    int j;
    for(j = 0; j < 4; j++){
        ui32CurrentOffset[j] = *(p_ui32ZeroCurrentOffset + j);
    }
    Sample(&Angle, &PrevAngle, &Rotations);
    Rotations = 0;
    abc(theta_ref, &v_d, &v_q, &v_u, &v_v, &v_w);                                  //inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                            //space vector modulation
    int i;
    wait_us(10);
    for(i = 0; i<20000; i++){
        GetFaults(&ui32Flags1, &ui32Flags2);
        SetPWM(PWMGEN_A,255*dtc_u);                                        // Set duty cycles
        SetPWM(PWMGEN_B,255*dtc_v);
        SetPWM(PWMGEN_C,255*dtc_w);
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        wait_us(100);
        }
    Sample(&Angle, &PrevAngle, &Rotations);

    GetFaults(&ui32Flags1, &ui32Flags2);

    wait_us(1000);

    float theta_start;

    Sample(&Angle, &PrevAngle, &Rotations);
    uint32_t ui32ElectricalAngle = Angle&0x3FFF;
    ui32ElectricalAngle = ui32ElectricalAngle * (uint32_t)NPP;
    ui32ElectricalAngle = ui32ElectricalAngle % 0x3FFF;
    ftheta_elec = uint_to_float(ui32ElectricalAngle, 0.0, 360.0, 14);

    ReadCurrent(&ui32CurrentRead);
    fi_a = current_to_float(ui32CurrentRead[0], ui32CurrentOffset[0], 40);
    fi_b = current_to_float(ui32CurrentRead[1], ui32CurrentOffset[1], 40);
    fi_c = current_to_float(ui32CurrentRead[2], ui32CurrentOffset[2], 40);
    dq0(ftheta_elec, fi_a, fi_b, fi_c, &fi_d, &fi_q);
    float current = sqrt(pow(fi_d, 2) + pow(fi_q, 2));
    ClearFaults();
    /// Rotate voltage angle
    while(theta_ref < 4*PI){                                                    //rotate for 2 electrical cycles
        ClearFaults();
        abc(theta_ref, &v_d, &v_q, &v_u, &v_v, &v_w);                             //inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                        //space vector modulation
        wait_us(100);
        SetPWM(PWMGEN_A,255*dtc_u);                                        // Set duty cycles
        SetPWM(PWMGEN_B,255*dtc_v);
        SetPWM(PWMGEN_C,255*dtc_w);
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        Sample(&Angle, &PrevAngle, &Rotations);                                                           //sample position sensor
        theta_actual = 2.0*PI/(float)0x3FFF * (float) (Angle + (Rotations*0x3FFF))
                       / (float)0x3FFF;
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200){
            sample_counter = 0;
       }
        sample_counter++;
        theta_ref += 0.001f;
    }
    float theta_end = 2.0*PI/(float)0x3FFF * (float) (Angle + (Rotations*0x3FFF))
                           / (float)0x3FFF;
    int direction = (theta_end - theta_start)>0;

    SetPWM(PWMGEN_A,255*0);                                        // Set duty cycles
    SetPWM(PWMGEN_B,255*0);
    SetPWM(PWMGEN_C,255*0);
    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);

    GetFaults(&ui32Flags1, &ui32Flags2);
    return direction;
}
//ben katz
float Calibrate(uint32_t *p_ui32ZeroCurrentOffset, int PHASE_ORDER)
{
    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 50;
    float delta = 2*PI*NPP/(n*n2);                                              // change in angle between samples
    float error_f[2688];                                                     // error vector rotating forwards
    float error_b[2688];
    float fi_b, fi_a, fi_c, fi_q, fi_d;
    int raw_f[2688];
    int raw_b[2688];
    float theta_ref = 0.0f;
    float theta_actual = 0.0f;
    float v_d = 0.1f;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w;
    float dtc_u = .5f, dtc_v = .5f, dtc_w = .5f;
    float ftheta_elec;
    int Angle;
    int PrevAngle, Rotations;
    uint32_t ui32CurrentRead[4];
    uint32_t ui32CurrentOffset[4];

    int j;
    for(j = 0; j < 4; j++){
        ui32CurrentOffset[j] = *(p_ui32ZeroCurrentOffset + j);
    }



    abc(theta_ref, &v_d, &v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    int i;
    for(i = 0; i<10000; i++){
        SetPWM(PWMGEN_A, 255*dtc_u);                                        // Set duty cycles
        if(PHASE_ORDER){
            SetPWM(PWMGEN_B,255*dtc_v);
            SetPWM(PWMGEN_C,255*dtc_w);
            }
        else{
            SetPWM(PWMGEN_C,255*dtc_v);
            SetPWM(PWMGEN_B,255*dtc_w);
            }
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        wait_us(100);
        }
    Sample(&Angle, &PrevAngle, &Rotations);
    uint32_t ui32ElectricalAngle = Angle&0x3FFF;
    ui32ElectricalAngle = ui32ElectricalAngle * float_to_uint(NPP, 0, 0xFFFFFFFF, 32);
    ui32ElectricalAngle = ui32ElectricalAngle % 0x3FFF;
    ftheta_elec = uint_to_float(ui32ElectricalAngle, 0.0, 360.0, 14);

    ReadCurrent(&ui32CurrentRead);
    fi_a = current_to_float(ui32CurrentRead[0], ui32CurrentOffset[0], 40);
    fi_b = current_to_float(ui32CurrentRead[1], ui32CurrentOffset[1], 40);
    fi_c = current_to_float(ui32CurrentRead[2], ui32CurrentOffset[2], 40);
    dq0(ftheta_elec, fi_a, fi_b, fi_c, &fi_d, &fi_q);    //dq0 transform on currents
    float current = sqrt(pow(fi_d, 2) + pow(fi_q, 2));

    for(i = 0; i<n; i++){                                                   // rotate forwards
       for(j = 0; j < n2; j++){
        theta_ref += delta;
        abc(theta_ref, &v_d, &v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
        SetPWM(PWMGEN_A, 255*dtc_u);                                        // Set duty cycles
        if(PHASE_ORDER){
           SetPWM(PWMGEN_B,255*dtc_v);
           SetPWM(PWMGEN_C,255*dtc_w);
           }
        else{
           SetPWM(PWMGEN_C,255*dtc_v);
           SetPWM(PWMGEN_B,255*dtc_w);
           }
        MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
        wait_us(100);
        Sample(&Angle, &PrevAngle, &Rotations);
       }
       Sample(&Angle, &PrevAngle, &Rotations);
       theta_actual = 2.0*PI/(float)0x3FFF * (float) (Angle + (Rotations*0x3FFF))
               / (float)0x3FFF;
       // get mechanical position
       error_f[i] = theta_ref/NPP - theta_actual;
       raw_f[i] = Angle;
    }

    for(i = 0; i<n; i++){                                                   // rotate forwards
           for(j = 0; j < n2; j++){
            theta_ref -= delta;
            abc(theta_ref, &v_d, &v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
            svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
            SetPWM(PWMGEN_A, 255*dtc_u);                                        // Set duty cycles
            if(PHASE_ORDER){
               SetPWM(PWMGEN_B,255*dtc_v);
               SetPWM(PWMGEN_C,255*dtc_w);
               }
            else{
               SetPWM(PWMGEN_C,255*dtc_v);
               SetPWM(PWMGEN_B,255*dtc_w);
               }
            MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_BITS);
            wait_us(100);
            Sample(&Angle, &PrevAngle, &Rotations);
           }
           Sample(&Angle, &PrevAngle, &Rotations);
           theta_actual = 2.0*PI/(float)0x3FFF * (float) (Angle + (Rotations*0x3FFF))
                   / (float)0x3FFF;
           // get mechanical position
           error_b[i] = theta_ref/NPP - theta_actual;
           raw_b[i] = Angle;
        }

       float offset = 0;
       for(i = 0; i<n; i++){
           offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
           }
       offset = fmod(offset*NPP, 2*PI);

       return offset;

}

