/*
 * controller_task.c
 *
 *  Created on: Nov 7, 2019
 *      Author: domya
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <float.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "maths/math_ops.h"
#include "drivers/drv8323.h"
#include "config/hardware_config.h"
#include "config/controller_config.h"
#include "utils/sine.h"
#include "utils/foc.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "controller_task.h"

//*****************************************************************************
//
// The stack size for the Cont task.
//
//*****************************************************************************
#define CONTTASKSTACKSIZE       128         // Stack size in words

//*****************************************************************************
//
// Control loop delay value in us. CONT loop frequency is 10^6 divided by this number.
//
//*****************************************************************************
#define CONT_DELAY              400

#define TARGET_VEL              0.0f

static int PHASE_ORDER;

static float g_felec_offset;


//*****************************************************************************
//
// current mechanical angle
//
//*****************************************************************************
volatile int32_t g_i32Mech_Angle;

//*****************************************************************************
//
// Target rots
//
//*****************************************************************************
volatile int8_t g_i8Rots = 0;

//*****************************************************************************
//
// Electrical Angle for calcs. Update every loop.
//
//*****************************************************************************
volatile float g_ftheta_elec;

//*****************************************************************************
//
// Electrical rotational velocity for calcs. Update every loop
//
//*****************************************************************************
volatile float g_fdtheta_elec;


volatile float g_fi_q_filt = 0.0f;

volatile float g_fi_d_ref;

volatile float g_fi_q_ref;

volatile float g_fprev_vel[3] = {0.0f, 0.0f, 0.0f};

//*****************************************************************************
//
// d and q voltage vector integrators. For torque control PID loop.
//
//*****************************************************************************
volatile float g_fd_int;
volatile float g_fq_int;


static uint32_t g_ui32CurrentOffset[4];

//*****************************************************************************
//
// The item size and queue size for the COMMS message queue.
//
//*****************************************************************************
#define ANGLE_ITEM_SIZE             sizeof(uint16_t)
#define TORQUE_ITEM_SIZE            sizeof(uint8_t)
#define FLAG_ITEM_SIZE              sizeof(uint8_t)
#define VELOCITY_ITEM_SIZE          sizeof(float)
#define CONT_QUEUE_SIZE             1

#define CPR                         0x3FFF

//*****************************************************************************
//
// The queue that holds messages sent to the Comms task.
//
//*****************************************************************************
extern xQueueHandle g_pCOMMSContFlagQueue;   //uint8_t
xQueueHandle g_pCONTPosQueue;               //uint16_t
xQueueHandle g_pCONTTorqueQueue;            //uint8_t
xQueueHandle g_pCONTCurrPosQueue;           //uint16_t
xQueueHandle g_pCONTCurrVelQueue;               //float

extern xSemaphoreHandle g_pDRV_Semaphore;


//*****************************************************************************
//
//! Makes the motor do the spin
//!
//! \param ui32Theta is the current electrical angle value from the encoder task
//! left shifted so that the MSB is bit 31.
//!
//! This function will...
//!
//! \return None.
//
//      Author: Ben Katz
//      Project: Hobbyking_Cheetah_Compact
//
//*****************************************************************************

static void
Commutate(uint32_t ui32Theta){

       uint32_t ui32ReadCurrentBuff[4];
       float fi_a = 0.0f, fi_b = 0.0f, fi_c = 0.0f;
       float fi_d, fi_q, fv_d, fv_q;
       float fv_u, fv_v, fv_w;
       float fdtc_u,fdtc_v, fdtc_w;

       /// Commutation Loop ///
       //
       //read and calculate controller phase currents.
       //
       ReadCurrent(&ui32ReadCurrentBuff);
       fi_a = current_to_float(ui32ReadCurrentBuff[0], g_ui32CurrentOffset[0], 20.0f);
       fi_b = current_to_float(ui32ReadCurrentBuff[1], g_ui32CurrentOffset[1], 20.0f);
       fi_c = current_to_float(ui32ReadCurrentBuff[2], g_ui32CurrentOffset[2], 20.0f);

       int32_t sin = sine(ui32Theta);
       int32_t cos = sine(ui32Theta + DEG_90);
       float s = ((float)sin)/65535.0f;
       float c = ((float)cos)/65535.0f;

       dq0(g_ftheta_elec, fi_a, fi_b, fi_c, &fi_d, &fi_q);    //dq0 transform on currents

       g_fi_q_filt = 0.95f*g_fi_q_filt + 0.05f*fi_q;
       //observer->i_d_m = controller->i_d;
       //observer->i_q_m = controller->i_q;

       //observer->e_d = observer->i_d_m - observer->i_d_est;
       //observer->e_q = observer->i_q_m - observer->i_q_est;
       //observer->e_d_int += observer->e_d;
       //observer->e_q_int += observer->e_q;

       //observer->i_d_est +=  K_O*observer->e_d + .001f*observer->e_d_int;
       //observer->i_q_est += K_O*observer->e_q + .001f*observer->e_q_int;


       int32_t sincog12 = sine((12*ui32Theta) % DEG_360);
       float scog12 = ((float)sincog12)/65535.0f;
       float scog1 = s;
       float cogging_current = 0.25f*scog1 - 0.3f*scog12;

       /// PI Controller ///
       float i_d_error = g_fi_d_ref - fi_d;
       float i_q_error = g_fi_q_ref - fi_q; //  + cogging_current;

       float v_d_ff = 2.0f*(g_fi_d_ref*R_PHASE - g_fdtheta_elec*L_Q*g_fi_q_ref);   //feed-forward voltages
       float v_q_ff =  2.0f*(g_fi_q_ref*R_PHASE  + g_fdtheta_elec*(L_D*g_fi_d_ref + WB));

       g_fd_int += i_d_error;
       g_fq_int += i_q_error;

       //v_d_ff = 0;
       //v_q_ff = 0;

       limit_norm(&g_fd_int, &g_fq_int, V_BUS/(K_SCALE*I_BW*KI_Q));        // Limit integrators to prevent windup
       fv_d = K_SCALE*I_BW*i_d_error + K_SCALE*I_BW*KI_D*g_fd_int;// + v_d_ff;
       fv_q = K_SCALE*I_BW*i_q_error + K_SCALE*I_BW*KI_Q*g_fq_int;// + v_q_ff;

       //controller->v_q = 4.0f;
       //controller->v_d = 0.0f;

       //controller->v_d = v_d_ff;
       //controller->v_q = v_q_ff;

       limit_norm(&fv_d, &fv_q, OVERMODULATION*V_BUS);       // Normalize voltage vector to lie within curcle of radius v_bus
       abc(g_ftheta_elec, fv_d, fv_q, &fv_u, &fv_v, &fv_w); //inverse dq0 transform on voltages

        //controller->v_u = c*controller->v_d - s*controller->v_q;                // Faster Inverse DQ0 transform
        //controller->v_v = (0.86602540378f*s-.5f*c)*controller->v_d - (-0.86602540378f*c-.5f*s)*controller->v_q;
        //controller->v_w = (-0.86602540378f*s-.5f*c)*controller->v_d - (0.86602540378f*c-.5f*s)*controller->v_q;
       svm(V_BUS, fv_u, fv_v, fv_w, &fdtc_u, &fdtc_v, &fdtc_w); //space vector modulation

       //observer->i_d_dot = 0.5f*(controller->v_d - 2.0f*(observer->i_d_est*R_PHASE - controller->dtheta_elec*L_Q*observer->i_q_est))/L_D;   //feed-forward voltage
       //observer->i_q_dot =  0.5f*(controller->v_q - 2.0f*(observer->i_q_est*R_PHASE  + controller->dtheta_elec*(L_D*observer->i_d_est + WB)))/L_Q;

       if(PHASE_ORDER){                                                         // Check which phase order to use,
           SetPWM(PWMGEN_A, PWM_FULL * (fdtc_u));
           SetPWM(PWMGEN_B, PWM_FULL * (fdtc_v));
           SetPWM(PWMGEN_C, PWM_FULL * (fdtc_w));
        }
        else{
            SetPWM(PWMGEN_A, PWM_FULL * (fdtc_u));
            SetPWM(PWMGEN_C, PWM_FULL * (fdtc_v));
            SetPWM(PWMGEN_B, PWM_FULL * (fdtc_w));
        }
       MAP_PWMSyncUpdate(PWM0_BASE,PWM_GEN_BITS);

       g_ftheta_elec = uint_to_float(ui32Theta>>20, 0.0f, 360.0f, 12) - g_felec_offset;                                          //For some reason putting this at the front breaks thins

}

static void
ContTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32ContDelay = CONT_DELAY;
    uint8_t ui8Flags = 0, ui8FlagRec;
    uint16_t ui16CmdPos, ui16CurrPos;
    uint8_t ui8CmdTorque = 0;
    int32_t i32Target_Angle = 0 ;
    int8_t i8TargetRots;
    float fCurrent_Vel, fTorque_ff, fFilt_Vel;
    int8_t bDat;

    //
    // Get the current tick count.
    //
    ui16LastTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        fFilt_Vel = 0.0f;
        //
        // Get new position data and transform to electrical angle
        //
        if(xQueueReceive(g_pCONTCurrPosQueue, &ui16CurrPos, 0) != pdPASS){
            ui8Flags = ui8Flags | 0x01; //flag for no new encoder data
            bDat = 0;
        }
        else{
            g_i8Rots = (int8_t)(((ui16CurrPos >> 12) & 0x07) * (int8_t)(powf(-1.0,(ui16CurrPos>>15))));
            g_i32Mech_Angle = ui16CurrPos & 0xFFF;
            g_i32Mech_Angle = g_i32Mech_Angle + (g_i8Rots * 0xFFF);

            g_ftheta_elec = uint_to_float((ui16CurrPos & 0xFFF), 0, 360, 12);
            g_ftheta_elec = fmodf(g_ftheta_elec*NPP,360.0) - g_felec_offset;
            bDat = 1;
        }

        //
        // Get current velocity
        //
        if(xQueueReceive(g_pCONTCurrVelQueue, &fCurrent_Vel, 0) != pdPASS){
            ui8Flags = ui8Flags | 1<<1; //flag for no new velocity data
        }
        else{
            int i;
            for(i=0; i <= 1; i++){
                fFilt_Vel += g_fprev_vel[2-i];
                g_fprev_vel[2-i] = g_fprev_vel[1-i];
            }
            fFilt_Vel += fCurrent_Vel;
            fFilt_Vel = fFilt_Vel/4.0f;
            g_fprev_vel[0] = fCurrent_Vel;

            g_fdtheta_elec = fFilt_Vel*NPP;
        }

        //
        // Get new position target
        //
        xQueuePeek(g_pCONTPosQueue, &ui16CmdPos, 0);
        i8TargetRots = (int8_t)(((ui16CmdPos >> 12) & 0x07) * (int8_t)(powf(-1.0,(ui16CmdPos>>15))));
        i32Target_Angle = ui16CmdPos & 0xFFF;
        i32Target_Angle = i32Target_Angle + (i8TargetRots * 0xFFF);

        //
        // Get new Torque target
        //
        xQueuePeek(g_pCONTTorqueQueue, &ui8CmdTorque, 0);
        fTorque_ff = uint_to_float((ui8CmdTorque & 0x7F), 0, 14, 7);
        fTorque_ff = ((-1.0f) * (ui8CmdTorque >> 7)) * fTorque_ff;

        //
        //Do PD control on position with ff torque
        //
        float fMechAngle = (float)g_i32Mech_Angle/(float)0xFFF;
        float fTargetAngle = (float)i32Target_Angle/(float)0xFFF;

        //float fTargetTorque = KP*(fTargetAngle-fMechAngle) + KD*(TARGET_VEL - fCurrent_Vel);
        float fTargetTorque = 0.2f;
        if(fTargetTorque < 0)
        {
            fTargetTorque = fmaxf(fTargetTorque, (-1.0*MAX_TORQUE));
        }
        else
        {
            fTargetTorque = fminf(fTargetTorque, MAX_TORQUE);
        }


        g_fi_d_ref = 0.0f;
        g_fi_q_ref = fTargetTorque/KT;

        /*
         if(fabsf(fTargetTorque) < 0.1f )
        {
            vTaskDelay(1);
        }
        */

        uint32_t ui32ElectricalAngle = ui16CurrPos & 0xFFF;
        ui32ElectricalAngle = ui32ElectricalAngle * (uint32_t)NPP;
        ui32ElectricalAngle = ui32ElectricalAngle % 0xFFF;
        ui32ElectricalAngle = ui32ElectricalAngle << 20;

        if(bDat)
        {
            Commutate(ui32ElectricalAngle);
        }

        vTaskDelayUntil(&ui16LastTime, ui32ContDelay / portTICK_RATE_US);
    }
}

uint32_t
ContTaskInit(uint32_t *p_ui32CurrentOffset, int PHASE_ORDER_set, float elec_offset)
{

    g_pCONTPosQueue = xQueueCreate(CONT_QUEUE_SIZE, ANGLE_ITEM_SIZE);
    g_pCONTTorqueQueue = xQueueCreate(CONT_QUEUE_SIZE, TORQUE_ITEM_SIZE);
    g_pCONTCurrPosQueue = xQueueCreate(CONT_QUEUE_SIZE, ANGLE_ITEM_SIZE);
    g_pCONTCurrVelQueue = xQueueCreate(CONT_QUEUE_SIZE, VELOCITY_ITEM_SIZE);

    int i;
    for(i = 0; i<4; i++){
        g_ui32CurrentOffset[i] = *(p_ui32CurrentOffset + i);
    }

    g_felec_offset = elec_offset;
    PHASE_ORDER = PHASE_ORDER_set;

    BaseType_t xReturned = xTaskCreate(ContTask, (const portCHAR *)"Controller",
                                     CONTTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                                     PRIORITY_CONT_TASK, NULL);
    if(xReturned != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}


