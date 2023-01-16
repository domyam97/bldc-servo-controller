/*
 * comms_task.c
 *
 *  Created on: Sep 10, 2019
 *      Author: domya
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/rs485.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "comms_task.h"

//*****************************************************************************
//
// The stack size for the COMMS task.
//
//*****************************************************************************
#define COMMSTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// COMMS transmit loop delay value in us. COMMS transmit frequency is 10^6 divided by this number.
// Must be a multiple of 100
//
//*****************************************************************************
#define COMMS_DELAY                 500

//*****************************************************************************
//
// The item size and queue size for the COMMS message queue.
//
//*****************************************************************************
#define ANGLE_ITEM_SIZE             sizeof(uint16_t)
#define TORQUE_ITEM_SIZE            sizeof(uint8_t)
#define FLAG_ITEM_SIZE              sizeof(uint8_t)
#define COMMS_QUEUE_SIZE            1

//*****************************************************************************
//
// The queue that holds messages sent to the Comms task.
//
//*****************************************************************************

static uint8_t g_ui8ID;

//*****************************************************************************
//
// The queue that holds messages sent to the Comms task.
//
//*****************************************************************************
xQueueHandle g_pCOMMSPosQueue, g_pCOMMSTorqueQueue, g_pCOMMSEncFlagQueue, g_pCOMMSContFlagQueue;

extern xQueueHandle g_pCONTPosQueue;
extern xQueueHandle g_pCONTTorqueQueue;

extern xSemaphoreHandle g_pRS485Semaphore;
extern xSemaphoreHandle g_pUARTSemaphore;

volatile char g_pcResp[6];
volatile char g_pcRead[5]; //max receive message size is 8 bytes

static void
CommsTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32CommsDelay = COMMS_DELAY;
    uint8_t ui8Flags = 0, ui8FlagRec;
    uint16_t ui16CmdPos, ui16CurrPos;
    uint8_t ui8CmdTorque, ui8CurrTorque;
    bool bTransmit = true;

    //
    // Get the current tick count.
    //
    ui16LastTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        //
        //Read a message from the RS485 Rx buffer, if available
        //
        if(RS485avail()){

            RS485read(g_pcRead, 5);
            bTransmit = true;
        }

        // Set up dummy message
        //
        // Read the message, see what message type it is
        //
        if(g_pcRead[0] == 0x00){              //Request Message - prep response

            //
            //Check ID
            //
            if(g_pcRead[1] == g_ui8ID){

                //
                //set message type RESP = 0x02
                //
                g_pcResp[0] = 0x02;

                //
                //set device ID
                //
                g_pcResp[1]  = g_ui8ID;

                //
                //set pos Value
                //
                if(xQueueReceive(g_pCOMMSPosQueue, &ui16CurrPos, 0) != pdPASS){
                    ui8Flags = ui8Flags | 0x01;
                }
                g_pcResp[2] = (ui16CurrPos & 0xFF00) >> 8;
                g_pcResp[3] = (ui16CurrPos & 0xFF);

                //
                //Set Torque Value
                //
                if(xQueueReceive(g_pCOMMSTorqueQueue, &ui8CurrTorque, 0) != pdPASS){
                    ui8Flags = ui8Flags | 0x02;
                    ui8CurrTorque = 0;
                }
                g_pcResp[4] = ui8CurrTorque;

                //
                //Receive flags from encoder
                //
                xQueueReceive(g_pCOMMSEncFlagQueue, &ui8FlagRec, 0);
                ui8Flags = ui8Flags | ui8FlagRec;
                //
                //Receive flags from controller
                //
                xQueueReceive(g_pCOMMSContFlagQueue, &ui8FlagRec, 0);
                ui8Flags = ui8Flags | ui8FlagRec;
                g_pcResp[5] = ui8Flags;


            }
        }


        else if(g_pcRead[0] == 0x01){         //Command Message - take in data, prep response

            //
            //Check ID
            //
            if(g_pcRead[1] == g_ui8ID){

                //
                //Write command pos to controller task
                //
                ui16CmdPos = g_pcRead[2] << 8;
                ui16CmdPos = ui16CmdPos | g_pcRead[3];

                //
                //Write command torque to controller task
                //
                ui8CmdTorque = g_pcRead[4];


                //
                //write response message
                //

                //
                //set message type RESP = 0x02
                //
                g_pcResp[0] = 0x02;

                //
                //set device ID
                //
                g_pcResp[1]  = g_ui8ID;

                //
                //set pos Value
                //
                if(xQueueReceive(g_pCOMMSPosQueue, &ui16CurrPos, 0) != pdPASS){
                    ui8Flags = ui8Flags | 0x01;
                }
                g_pcResp[2] = (ui16CurrPos & 0xFF00) >> 8;
                g_pcResp[3] = (ui16CurrPos & 0xFF);

                //
                //Set Torque Value
                //
                if(xQueueReceive(g_pCOMMSTorqueQueue, &ui8CurrTorque, 0) != pdPASS){
                    ui8Flags = ui8Flags | 0x02;
                }
                g_pcResp[4] = ui8CurrTorque;

                g_pcResp[5] = ui8Flags;
            }
        }

        // RS485write(0xAB, 1);

        if(bTransmit){
            xSemaphoreTake(g_pRS485Semaphore, portMAX_DELAY);
            RS485write(&g_pcResp, 6);
            xSemaphoreGive(g_pRS485Semaphore);
            bTransmit = false;
        }
        vTaskDelayUntil(&ui16LastTime, ui32CommsDelay / portTICK_RATE_US);
    }
}

uint32_t
CommsTaskInit(uint8_t ui8ID)
{

        g_pCOMMSPosQueue = xQueueCreate(COMMS_QUEUE_SIZE, ANGLE_ITEM_SIZE);
        g_pCOMMSTorqueQueue = xQueueCreate(COMMS_QUEUE_SIZE, TORQUE_ITEM_SIZE);
        g_pCOMMSEncFlagQueue = xQueueCreate(COMMS_QUEUE_SIZE, FLAG_ITEM_SIZE);
        g_pCOMMSContFlagQueue = xQueueCreate(COMMS_QUEUE_SIZE, FLAG_ITEM_SIZE);
        g_ui8ID = ui8ID;

        //
        // Create the switch task.
        //
        BaseType_t xReturned = xTaskCreate(CommsTask, (const portCHAR *)"Comms",
                                     COMMSTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                                     PRIORITY_COMMS_TASK, NULL);
        if(xReturned != pdTRUE)
        {
            return(1);
        }

        //
        // Success.
        //
        return(0);
}
