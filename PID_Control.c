#include "PID_Control.h"

/* GLOBAL */
struct timespec now;
		
// Set tunable proportionality constants
void init_PID(PID *p, double Kp, double Ki, double Kd) {
	p->kp = Kp;
	p->ki = Ki;
	p->kd = Kd;
}

void reset_PID(PID *p){
	int j;
	p->lastErr = 0.0;
	
	for(j = 0; j < 8; j++) p->errSum[j] = 0.0;
	
	p->i = 0;
	clock_gettime(CLOCK_REALTIME, &(p->lastTime));
}

double Compute(PID *p, double Input, double Setpoint) {
	double 	error,
		dErr,
		sum,
		Output,
		dt;
	int	j;
	
	clock_gettime(CLOCK_REALTIME, &now);
   
	dt = (double)(now.tv_sec - p->lastTime.tv_sec) + (double)(now.tv_nsec - p->lastTime.tv_nsec)/1000000000.0;
	
	/* catch any timer errors, normalize to 1 second */
	if(dt > 1.0) dt = 1.0;
	if(dt < 0.0) dt = 1.0;
	
	/*Compute all the working error variables*/
	error = Setpoint - Input;
	p->errSum[p->i] = error * dt;
	dErr = (error - p->lastErr) / dt;
	
	sum = 0.0;
	for(j = 0; j < 8; j++) sum += p->errSum[j];

	printf("dt = %f\terror = %f\terrSum = %f\tdErr = %f\n", dt, error, sum, dErr);
	
	/*Compute PID Output*/
	Output = (p->kp * error) + (p->ki * sum) + (p->kd * dErr);

	/*Remember some variables for next time*/
	p->lastErr = error;
	p->lastTime.tv_sec = now.tv_sec;
	p->lastTime.tv_nsec = now.tv_nsec;	
	p->i++;
	if(p->i == 8) p->i = 0;
	
	return Output;
}