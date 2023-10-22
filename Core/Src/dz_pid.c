/*
 * dz_pid.c
 *
 *  Created on: Oct 22, 2023
 *      Author: durmaz
 */


#include "dz_pid.h"

int16_t pidCalculate(struct pid* pidx,int16_t setpoint, int16_t sensor){
	int16_t hata = setpoint - sensor;

	double pid_p = pidx->Kp * (double)hata;

	double pid_d = pidx->Kd * (double)(hata - pidx->prev_error) / DELTAT;

	pidx->integral += (double)hata * DELTAT;
	if(pidx->integral > 200){
		pidx->integral = 200;
	}
	else if(pidx->integral < -200){
		pidx->integral = -200;
	}

	int16_t rtr = (int16_t)(pid_p + (pidx->integral * pidx->Ki) + pid_d);

	pidx->prev_error = hata;

	if(rtr > PIDMAX){
		rtr = PIDMAX;
	}
	else if(rtr < PIDMIN){
		rtr = PIDMIN;
	}

	return rtr;
}

