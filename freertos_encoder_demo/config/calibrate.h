/*
 * calibrate.h
 *
 *  Created on: Nov 11, 2019
 *      Author: domya
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#define SYS_CLK         800000000UL

extern int OrderPhases(uint32_t *p_ui32CurrentZeroOffset);
extern float Calibrate(uint32_t *p_ui32CurrentZeroOffset, int PHASE_ORDER);


#endif /* CALIBRATE_H_ */
