#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "position.h"

// Set DATA_COLLECT to 1 supress normal output and dump data for csv files
#define DATA_COLLECT 1

/*working variables*/
struct timespec *lastTime;
struct timespec *now;
double kp, ki, kd;
int SampleTime = 500000; // timespec has resolution of nanoseconds --> 0.5e6 ns

double Compute(double Input, double Setpoint) {
	static double errSum;
	static double lastErr;
	double 	error,
		dErr;
	int 	timeChange;
	
	clock_gettime(CLOCK_REALTIME, now);
   
	timeChange = (now->tv_sec - lastTime->tv_sec) * 1000000 + (now->tv_usec - lastTime->tv_usec);
	
	if(timeChange>=SampleTime) {
		/*Compute all the working error variables*/
		error = Setpoint - Input;
		errSum += error;
		dErr = (error - lastErr);
 
		/*Compute PID Output*/
		Output = kp * error + ki * errSum + kd * dErr;
 
		/*Remember some variables for next time*/
		lastErr = error;
		lastTime->tv_sec = now->tv_sec;
		lastTime->tv_usec = now->tv_usec;
	}
}

// Set tunable proportionality constants
void SetTunings(double Kp, double Ki, double Kd) {
	double SampleTimeInSec = ((double)SampleTime)/1000000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
}

// Change Sample Time
void SetSampleTime(int NewSampleTime) {
	double ratio;
  
	if (NewSampleTime > 0) {
		ratio  = (double)NewSampleTime / (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

float get_euclidian_distance(int start_x, int start_y, int end_x, int end_y){
 	return sqrt( (float)( pow((start_x-end_x),2) + pow((start_y-end_y),2) ) );//check me
}

float get_theta_to_target(){//current_theta-get_theta_to_target() = theta to correct
 float theta_to_target = arctan( ((float)(start_y-end_y)) / ((float)(end_x-start_x)));
 if(end_x<start_x){//its gonna be outside of the range of arctan
   if(end_y>start_y)
     theta_to_target = PI - theta_to_target;//turn right > pi/2
   else if(start_y>end_y)
     theta_to_target = PI + theta_to_target;//turn left >pi/2
   else//it needs to make a 180
     theta_to_target = PI;//180 degrees
 }
}
char go_to_position(int end_x, int end_y, float end_theta, float tolerance){//returns boolean to say if it made it
 int start_x, start_y, current_x, current_y;
 float start_theta, current_theta, distance_to_target;

 //initialize start_x, start_y, start_theta


 while( (distance_to_target = get_euclidian_distance(current_x,
current_y, end_x, end_y) ) > tolerance){
   //point robot at destination using PID

   //move and close gap to target using PID

   //find currentx and current y and current theta

   //refresh current_x = ....
 }


 //point robot to end theta using PID
}


/* Not functional, would like to add it for awareness of battery levels */
void battery_check( robot_if_t *ri ) {
	if( ri_getBattery(ri) > RI_ROBOT_BATTERY_HOME ) {
		printf("Charge me please!!!");
		exit(11);
	}	
}

int main(int argv, char **argc) {
	//int i;
	robot_if_t ri;
	vector *location = (vector *)calloc(1, sizeof(vector));
	float target_x,
	      target_y;
	
	// initialize memory for timing of the PID controller
	lastTime = malloc(sizeof(timespec));
	now = malloc(sizeof(timespec));
	
        // Make sure we have a valid command line argument
        if(argv <= 3) {
                printf("Usage: robot_test <address of robot> <distance to travel in X in cm> <distance to travel in Y in cm>\n");
                exit(-1);
        }
        
        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");
	
	// Check condition of battery, exit if not enough charge
	battery_check(&ri);

	// Retrieve target distance from command line
	target_X = (float) atoi(argc[2]);
	target_Y = (float) atoi(argc[3]);
			
	// Retrieve initial position, initailize current and last
	init_pos(&ri);

#if (DATA_COLLECT)
	print_stance_csv();
#endif
	
        // Action loop
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
		// Bad attempt at PID
		/*	// Straigten out robot if neccessary
			if ( delta_theta() > 0.175 ) {
				ri_move(&ri, RI_TURN_RIGHT, 6);
				ri_move(&ri, RI_MOVE_FWD_RIGHT, RI_SLOWEST);
			}
			else if ( delta_theta() < -0.175 ) {
				ri_move(&ri, RI_TURN_LEFT, 6);
				ri_move(&ri, RI_MOVE_FWD_LEFT, RI_SLOWEST);
			}
			else {*/
				if(location->v[1] < 0.8 * target_dist) ri_move(&ri, RI_MOVE_FORWARD, RI_FASTEST);
				else ri_move(&ri, RI_MOVE_FORWARD, RI_SLOWEST);
		//	}
		}
		else {
			//printf("I found an obstacle!  Stopping!\n\n");
			break;
		}
		
						
		get_Position(&ri, location);
#if (DATA_COLLECT)
		print_stance_csv();
#else
		printf("Location:  X = %f cm\tY = %f cm\n", location->v[0], location->v[1]);
#endif
        } while(location->v[0] < target_dist);

	free(location);
	free(lastTime);
	free(now);
	
	exit_pos();
	
	return 0;
}
