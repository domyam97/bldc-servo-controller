/*
 * hardware_config.h
 *
 *  Created on: Nov 9, 2019
 *      Author: domya
 */

#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_

#define R_SENSE             1.5E-3f
#define V_REF               3.3f
#define V_BUS               25.0f
#define NPP                 21.0f
#define KT                  0.111f
#define MAX_TORQUE          0.95f


#define PWM_FULL            1000


//
//Motor Constants from Ben Katz
//
#define L_D 0.00003f            //Henries
#define L_Q 0.00003f            //Henries
#define R_PHASE 0.1601f         //Ohms
#define WB KT/NPP        //Webers.



#endif /* HARDWARE_CONFIG_H_ */
