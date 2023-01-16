/*
 * controller_config.h
 *
 *  Created on: Nov 11, 2019
 *      Author: domya
 */

#ifndef CONFIG_CONTROLLER_CONFIG_H_
#define CONFIG_CONTROLLER_CONFIG_H_

//
//Torque control constants
//
#define KD                  0.5f
#define KP                  10.0f

//
//FOC PID Controller Costants - from Ben Katz of course
//
#define K_D                 .05f             // Loop gain,  Volts/Amp
#define K_Q                 .05f             // Loop gain,  Volts/Amp
#define K_SCALE             0.0001f          // K_loop/Loop BW (Hz) 0.0042
#define KI_D                0.0255f          // PI zero, in radians per sample
#define KI_Q                0.0255f          // PI zero, in radians per sample
#define OVERMODULATION      1.2f             //1.0 means no overmoduation
#define I_BW                20.0f



#endif /* CONFIG_CONTROLLER_CONFIG_H_ */
