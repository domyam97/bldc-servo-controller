/*
 * encoder_task.c
 *
 *  Created on: Oct 19, 2019
 *      Author: domya
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <float.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/qei.h"
#include "drivers/ams_AS5047.h"
#include "maths/math_ops.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the encoder task.
//
//*****************************************************************************
#define ENCTASKSTACKSIZE        128         // Stack size in words

#define CPR                     0x3FFF


extern xSemaphoreHandle g_pEncoderSemaphore;

extern xQueueHandle g_pCOMMSPosQueue;           //uint16_t
extern xQueueHandle g_pCONTCurrPosQueue;        //uint16_t
extern xQueueHandle g_pCONTCurrVelQueue;            //float
extern xQueueHandle g_pCOMMSEncFlagQueue;       //uint8_t

//*****************************************************************************
//
// Previous 14-bit encoder count
//
//*****************************************************************************
volatile int16_t g_i16PrevAngle = 0;

//*****************************************************************************
//
// Stores number of rotations - not sure how to actually use this data.
// Need con ops for startup.
//
//*****************************************************************************
volatile int8_t g_i8RotCount=0;

//*****************************************************************************
//
// Stores number of rotations - not sure how to actually use this data.
// Need con ops for startup.
//
//*****************************************************************************
//static uint16_t g_ui16Offset=0;



//*****************************************************************************
//
// This task reads the encoder values and prints them
//
//*****************************************************************************
static void
EncoderTask(void *pvParameters){

    portTickType ui16LastTime;
    uint32_t ui32EncoderDelay = 200; //in useconds
    uint32_t ui32AngleRead;
    int16_t i16Angle;
    uint8_t ui8Flags;
    uint16_t ui16AngleForm = 0;
    int8_t bDir,bRot = 0;
    float fpVelocity,fpAngle,fpPrevAngle;

    //
    // Loop forever.
    //
    while(1)
    {
        ui16AngleForm = 0;
        bRot = 0;
        bDir = 0;
        ui8Flags = 0;

        GetAngle(DAE_CORRECTION, &ui32AngleRead);

        i16Angle = (int16_t) ui32AngleRead;
        //i16Angle = i16Angle - g_ui16Offset

        //
        //check if rotation
        //
        if(i16Angle - g_i16PrevAngle < -CPR/2){
            g_i8RotCount++;
            bRot = 1;
        }

        else if(i16Angle - g_i16PrevAngle > CPR/2){
            g_i8RotCount--;
            bRot = -1;
        }

        else{
            bRot = 0;
        }

        //
        //check direction
        //
        if(i16Angle + bRot*CPR - g_i16PrevAngle < 0){
            bDir = 1;
        }

        else{
            bDir = 0;
        }


        //
        //check if Rots is out of range. If it is, raise flag. 00000100b
        //
        if(g_i8RotCount > 7 || g_i8RotCount < -7){
            ui8Flags = 1 << 2;
        }

        //
        //Format rotations part of message for negative Rots.
        //
        if(g_i8RotCount < 0){
            ui16AngleForm = 1 << 15;
            ui16AngleForm = ui16AngleForm | ((-1*g_i8RotCount & 0x07) << 12);
        }
        //
        //Format rotations part of message for positive Rots.
        //-This is all because I'm not clever with two's compliment.
        //
        else{
            ui16AngleForm = ui16AngleForm | ((g_i8RotCount & 0x07) << 12);
        }

        //
        //Format angle part of message.
        //
        ui16AngleForm = ui16AngleForm | (i16Angle >> 2);

        //
        //write data for COMMS
        //
        xQueueOverwrite(g_pCOMMSPosQueue, &ui16AngleForm);

        //
        //write position to Controller - probably similar to comms
        //
        xQueueOverwrite(g_pCONTCurrPosQueue, &ui16AngleForm);

        //
        //calculate and send current velocity (assume 200 usecond period for test)
        //
        fpAngle = uint_to_float((uint32_t) i16Angle + bRot*CPR, 0.0f, 2.0f*PI, 14);
        fpPrevAngle = uint_to_float((uint32_t) g_i16PrevAngle, 0.0f, 2.0f*PI, 14);

        fpVelocity = (fpAngle-fpPrevAngle)/(200.0f/1000000.0f);
        xQueueOverwrite(g_pCONTCurrVelQueue, &fpVelocity);

        //
        //write flags to comms - probably similar to comms
        //
        xQueueOverwrite(g_pCOMMSEncFlagQueue, &ui8Flags);


        g_i16PrevAngle = i16Angle;

        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&ui16LastTime, ui32EncoderDelay / portTICK_RATE_US);
    }
}

uint32_t
EncoderTaskInit(void/*uint16_t ui16Offset*/)
{
    //g_ui16Offset = ui16Offset;
    uint32_t ui32AngleRead;

    GetAngle(DAE_CORRECTION, &ui32AngleRead);
    g_i16PrevAngle = (int16_t)ui32AngleRead;

    //
    // Create the switch task.
    //
    BaseType_t xReturned = xTaskCreate(EncoderTask, (const portCHAR *)"Encoder",
                                 ENCTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                                 PRIORITY_ENC_TASK, NULL);

    if(xReturned != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
