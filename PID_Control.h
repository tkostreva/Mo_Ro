#ifndef _PID_Control_
#define _PID_Control_

#include <time.h>
#include <stdio.h>

typedef struct _PID_ {//keeps track of a specific PID control's parameters and data
	int	i;
	double	errSum[8],
		lastInput,
		kp,
		ki,
		kd;
	struct timespec lastTime;
} PID;

/* Set tunable proportionality constants */
void init_PID(PID *p, double Kp, double Ki, double Kd);//initializes the p i and d constants

void reset_PID(PID *p);//zero out last readings.

/* Compute output of PID control based on input and setpoint. must be called often for greater accuracy */
double Compute(PID *p, double Input, double Setpoint);

#endif