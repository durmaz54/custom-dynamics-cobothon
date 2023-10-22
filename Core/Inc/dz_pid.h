/*
 * dz_pid.h
 *
 *  Created on: Oct 22, 2023
 *      Author: durmaz
 */

#ifndef INC_DZ_PID_H_
#define INC_DZ_PID_H_

#define PIDMAX	999
#define PIDMIN	350
#define DELTAT (double)0.01

#include "stdint.h"

struct pid{
	double Kp;
	double Ki;
	double Kd;
	double integral;
	int16_t prev_error;
};

int16_t pidCalculate(struct pid* pidx,int16_t setpoint, int16_t sensor);

#endif /* INC_DZ_PID_H_ */
