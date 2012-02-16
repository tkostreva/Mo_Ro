#ifndef _PID_Control_
#define _PID_Control_

#include <time.h>
#include <stdio.h>

typedef struct _PID_ {
	int	i;
	double	errSum[8],
		lastInput,
		kp,
		ki,
		kd;
	struct timespec lastTime;
} PID;

/* Set tunable proportionality constants */
void init_PID(PID *p, double Kp, double Ki, double Kd);

void reset_PID(PID *p);

/* Compute output of PID control based on input and setpoint */
double Compute(PID *p, double Input, double Setpoint);

#endif