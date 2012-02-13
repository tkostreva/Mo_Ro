#ifndef _PID_Control_
#define _PID_Control_

#include <time.h>

typedef struct _PID_ {
	int 	SampleTime;
	double	errSum,
		lastErr,
		kp,
		ki,
		kd;	
	struct timespec lastTime;
} PID;

/* Set tunable proportionality constants */
void SetTunings(PID *p, int InitialSampleTime, double Kp, double Ki, double Kd);

/* Compute output of PID control based on input and setpoint */
double Compute(PID *p, double Input, double Setpoint);

#endif