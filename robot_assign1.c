/*  
 * Filename: robot_assign2.c  
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause  
 * Date: 03-20-2012  
 * Purpose: use both image processing and room waypoints to guide the robot through the corrider
 */


#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "position.h"
#include "PID_Control.h"
#include "robot_vision.h"

/* DEFINES */
//#define WAYPOINT_COORDS {{342.9, 0.0},{243.84, 182.88},{297.18, 182.88},{406.400, 302.26},{060.96, 403.86},{0,0}}
//#define NUMBER_OF_WAYPOINTS 6 /*6 {342.9, 0.0}*/
//#define WAYPOINT_COORDS {{150.0,0.0},{150.0,0.0}}
#define WAYPOINT_COORDS {{64.0, 0.0},{64.0, 0.0},{64.0, 0.0},{64.0, 0.0},{64.0, 0.0},{0.0, 64.0},{0.0, 64.0},{0.0, 64.0}}
#define NUMBER_OF_WAYPOINTS 8

#define F_Kp 1.0
#define F_Ki 0.1
#define F_Kd 0.01

#define R_Kp 10.0/M_PI
#define R_Ki 0.5
#define R_Kd 0.20

#define NS_RADIUS  9.5   /* radius from center of bot to NS sensor */

/* GLOBALS */
PID 	*fwdPID,
	*rotPID;

float rot_speed[] = {  // Rotation Speeds in [rad/s]
	3.0,
	3.0,
	2.55,
	2.55,
	2.1,
	2.0,
	1.2,
	1.17,
	1.06,
	1.06
};

float fwd_speed[] = {  // Forward speeds in [cm/s]
	33.94,
	33.94,
	32.5,
	32.5,
	31.6,
	31.6,
	22.2,
	22.2,
	20.2,
	20.2
};

/* FUNCTIONS */
int fwdSpeedScaling(float PIDout) {
	int 	temp,
		speed;
	
	temp = (int) PIDout;
	temp = abs(temp);
	
	if (temp >= 80) speed = 1;
	else if (temp >= 60 && temp < 80) speed = 3;
	else if (temp >= 40 && temp < 60) speed = 5;
	else if (temp >= 20 && temp < 40) speed = 7;
	else if (temp < 20) speed = 9;
	
	if(PIDout < 0) speed *= -1;
	
	return speed;
}

int rotSpeedScaling(float PIDout) {
	float 	temp;
	int	speed;
	
	temp = fabs(PIDout);
	
	if (temp >= 10.0) speed = 3;
	else if (temp >= 9.0 && temp < 10.0) speed = 3;
	else if (temp >= 8.0 && temp < 9.0) speed = 3;
	else if (temp >= 7.0 && temp < 8.0) speed = 3;
	else if (temp >= 6.0 && temp < 7.0) speed = 3;
	else if (temp >= 5.0 && temp < 6.0) speed = 3;
	else if (temp >= 4.0 && temp < 5.0) speed = 3;
	else if (temp >= 3.0 && temp < 4.0) speed = 3;
	else if (temp >= 2.0 && temp < 3.0) speed = 3;
	else if (temp < 2.0) speed = 3;
	
	if(PIDout < 0) speed *= -1;
	
	return speed;
}

float get_euclidian_distance(float start_x, float start_y, float end_x, float end_y){
	float	diff1,
		diff2;
	diff1 = end_x - start_x;
	diff2 = end_y - start_y;
 	return sqrt( diff1 * diff1 + diff2 * diff2 );
}

/* current_theta-get_theta_to_target() = theta to correct */
float get_theta_to_target(float start_x, float start_y, float end_x, float end_y){
	//this needs to be made more robust... what happens after it turns around?
  
 	float theta_to_target = atan( (end_y - start_y) / (end_x-start_x) );
	if( end_x < start_x ){//its gonna be outside of the range of arctan//this heuristic isnt perfect--> consider negative motion
   		if( end_y  > start_y){
			printf("theta > 90 case1 -> left  \n\n");
     			theta_to_target = M_PI + theta_to_target; //turn left
		}else if(start_y>end_y){
    			printf("theta > 90 case2 -> right \n\n");
			theta_to_target = M_PI - theta_to_target;//turn right
		}else//it needs to make a 180
     			theta_to_target = M_PI;//180 degrees
 	}
 	printf("  get theta to target called: start x = %f,end x = %f, start x = %f,end x = %f, deduced theta = %f\n\n", start_x, end_x, start_y, end_y, theta_to_target);
 	
 	return theta_to_target;
}

void rotate_to_theta(robot_if_t *ri, float target_theta, vector *current_location){
	float	output,
		rot_amount,
		sf;		/* scaling factor for windup */
	int  	ang_vel;
	vector *expected_vel;
	
	/* reset PID control for this rotation */
	reset_PID(rotPID);
	
	expected_vel = (vector *)calloc(1, sizeof(vector));
	
	sf = 0.1;
	
	do {
		printf("\n *********************  ROT PID ENABLED  ********************\n\n");
		printf("Curr Theta = %f\tTarget Theta = %f\n", current_location->v[2], target_theta);
		output = Compute(rotPID, current_location->v[2], target_theta);
		printf("Rotate PID Output = %f\n", output);
		
		// correlate output to an angular velocity
		ang_vel = rotSpeedScaling(output);
		 
		if(ang_vel > 0) {
			ri_move(ri, RI_TURN_LEFT, ang_vel);
			ri_move(ri, RI_STOP, 1);
			
			expected_vel->v[0] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = rot_speed[ang_vel - 1];
		 }
		else {
			ang_vel *= -1;
			ri_move(ri, RI_TURN_RIGHT, ang_vel);
			ri_move(ri, RI_STOP, 1);
			
			expected_vel->v[0] = 0.0;//-1 * NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = 0.0;//-1 * NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = -1.0 * rot_speed[ang_vel - 1];
		 }
		
		/* factor in windup time with scaling factor */
		expected_vel->v[0] *= sf;
		expected_vel->v[1] *= sf;
		expected_vel->v[2] *= sf;
		
		/* increment scaling factor */
		if(sf < 1.0) sf += 0.1;
		 
		get_Position(ri, current_location, expected_vel, ROTATE);
		
		printf("Kalmann filtered result = %f\t%f\t%f\n\n", current_location->v[0], current_location->v[1], current_location->v[2]);
		
		rot_amount = fabs(target_theta - current_location->v[2]);
	} while (rot_amount > 0.075);  // found the granularity of turning is roughly 0.45 radians per turn (single plug...  speed = 6
	
	free(expected_vel);
}

void go_to_position(robot_if_t *ri, float end_x, float end_y){
 	float	setpoint,
		x_i,
		y_i,
		current_distance,
		output,
		sf,
		tolerance;
	int	i,
		bot_speed;
	vector 	*current_location,
		*expected_vel;
	
	current_location = (vector *)calloc(1, sizeof(vector));
	expected_vel = (vector *)calloc(1, sizeof(vector));
 	
	tolerance = 7.5;//adjustable.  How close should I be to the waypoint before moving onto the next one?
	
	/* reset PID control for this move */
	reset_PID(fwdPID);
		
 	//initialize start_x, start_y, start_theta
	get_Position(ri, current_location, expected_vel, FORWARD);
	x_i = current_location->v[0];
	y_i = current_location->v[1];
	
	sf = 0.1;
	i = 0;
	
	setpoint = get_euclidian_distance(current_location->v[0], current_location->v[1], 
		current_location->v[0] + end_x, current_location->v[1] + end_y);
	
	do {
		current_distance = get_euclidian_distance(x_i, y_i, current_location->v[0], current_location->v[1]);		
		
		printf("\n *********************  FWD PID ENABLED  ********************\n\n");
		printf("CurrDist = %f\tSetPoint = %f\n", current_distance, setpoint);
		output = Compute(fwdPID, current_distance, setpoint );
		printf("FWD PID Output = %f\n", output);
		
		// correlate output to a bot_speed NEGATIVE SPEEDS MOVE THE BOT BACKWARDS
		
		bot_speed = fwdSpeedScaling(output);
	
		// move the bot based on bot_speed and define expected velocities for kalman filter
		if(bot_speed > 0) {
			ri_move(ri, RI_MOVE_FORWARD, bot_speed);
			expected_vel->v[0] = fwd_speed[bot_speed - 1] * cos(current_location->v[2]);
			expected_vel->v[1] = fwd_speed[bot_speed - 1] * sin(current_location->v[2]);
		}
		else {
			bot_speed *= -1;
			ri_move(ri, RI_MOVE_BACKWARD, bot_speed);
			expected_vel->v[0] = -1.0 * fwd_speed[bot_speed - 1] * cos(current_location->v[2]);
			expected_vel->v[1] = -1.0 * fwd_speed[bot_speed - 1] * sin(current_location->v[2]);
		}
		
		expected_vel->v[0] *= sf;
		expected_vel->v[1] *= sf;
		expected_vel->v[2] = 0.0;
		
		/* incriment scaling factor for expected velocites during wind up */
		if (sf < 1.0) sf += 0.2;
		i++;
		
		//refresh current position values and see if bot changed rooms.  If it does, reset scaling factor
   		if( get_Position(ri, current_location, expected_vel, FORWARD) ) sf = 0.1;
		
		printf("Kalmann filtered result = %f\t%f\t%f\n", current_location->v[0], current_location->v[1], current_location->v[2]);		
 	} while(current_distance <= (setpoint - tolerance) || current_distance >= (setpoint + tolerance) ); // && (!ri_IR_Detected(ri)));
 	
 	ri_move(ri, RI_STOP, 1);

 	//point robot to end theta using PID //code me
 	free(current_location);
	free(expected_vel);
}


/* Not functional, would like to add it for awareness of battery levels */
void battery_check( robot_if_t *ri ) {
	if( ri_getBattery(ri) < RI_ROBOT_BATTERY_HOME ) {
		printf("Charge me please!!!");
		exit(11);
	}	
}

/* MAIN */

int main(int argv, char **argc) {
	robot_if_t ri;
	vector 	*loc,
		*vel;
	float 	target_x,
		target_y;
	   
	float waypoints[NUMBER_OF_WAYPOINTS][2] = WAYPOINT_COORDS;//WAYPOINT_COORDS;
	int numWayPoints = NUMBER_OF_WAYPOINTS, index;
	        
        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0)) printf("Failed to setup the robot!\n");
	
	if(ri_getHeadPosition(&ri) != RI_ROBOT_HEAD_LOW ) ri_move(&ri, RI_HEAD_DOWN , 1);
	
	// Check condition of battery, exit if not enough charge
	//battery_check(&ri);

	loc = (vector *)calloc(1, sizeof(vector));
	vel = (vector *)calloc(1, sizeof(vector));
	
	// Initialize PID controllers
	fwdPID = calloc(1, sizeof(PID));
	rotPID = calloc(1, sizeof(PID));
	init_PID(fwdPID, F_Kp, F_Ki, F_Kd);
	init_PID(rotPID, R_Kp, R_Ki, R_Kd);
	
	// Retrieve initial position, initailize current and last
	init_pos(&ri);
	
	//cvNamedWindow("Rovio Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Square Display", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded", CV_WINDOW_AUTOSIZE);

	//waypoint nav:
	for(index = 0; index < numWayPoints; index++){
		target_x = waypoints[index][0];
		target_y = waypoints[index][1];
		go_to_position(&ri, target_x, target_y);
		
		printf("\n *********************  Waypoint %d Reached  ********************\n\n", (index+1));
		
		center(&ri);
		
		if(index == 4) {
			get_Position(&ri, loc, vel, FORWARD);
			rotate_to_theta(&ri, -M_PI/2.0, loc);
		}
	}
	
	free(fwdPID);
	free(rotPID);
	free(loc);
	free(vel);
	
	exit_pos();
	
	cvDestroyWindow("Square Display");
	cvDestroyWindow("Thresholded");
	
	return 0;
}
