#include "PID_Control.h"

/* GLOBAL */
struct timespec now;
		
// Set tunable proportionality constants
void SetTunings(PID *p, int InitSampleTime, double Kp, double Ki, double Kd) {
	double SampleTimeInSec = ((double)InitSampleTime) / 1000000000.0;  // because SampleTimes are in nanoseconds
	
	p->SampleTime = InitSampleTime;
		
	p->kp = Kp;
	p->ki = Ki * SampleTimeInSec;
	p->kd = Kd / SampleTimeInSec;
}

double Compute(PID *p, double Input, double Setpoint) {
	double 	error,
		dErr,
		Output;
	int 	timeChange;
	
	clock_gettime(CLOCK_REALTIME, &now);
   
	timeChange = (now.tv_sec - p->lastTime.tv_sec) * 1000000000 + (now.tv_nsec - p->lastTime.tv_nsec);
	
	if( timeChange >= p->SampleTime ) {
		/*Compute all the working error variables*/
		error = Setpoint - Input;
		p->errSum += error;
		dErr = (error - p->lastErr);
 
		/*Compute PID Output*/
		Output = p->kp * error + p->ki * p->errSum + p->kd * dErr;
 
		/*Remember some variables for next time*/
		p->lastErr = error;
		p->lastTime.tv_sec = now.tv_sec;
		p->lastTime.tv_nsec = now.tv_nsec;		
	}
	
	return Output;
}