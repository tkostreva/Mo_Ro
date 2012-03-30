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

/* DEFINES */
//#define WAYPOINT_COORDS {{342.9, 0.0},{243.84, 182.88},{297.18, 182.88},{406.400, 302.26},{060.96, 403.86},{0,0}}
//#define NUMBER_OF_WAYPOINTS 6 /*6 {342.9, 0.0}*/
//#define WAYPOINT_COORDS {{150.0,0.0},{150.0,0.0}}
#define WAYPOINT_COORDS {{0.0,-5.0}}
#define NUMBER_OF_WAYPOINTS 1

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

float fwd_speed[] = {  // Forward speeds in [cm/s]  (Halved in code to compensate for stop / lag in bot )
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

/* FUNCTIONS */
int fwdSpeedScaling(float PIDout) {
	float 	fwdScale;
	int	speed;
	
	fwdScale = fabs(PIDout);
		
	if (fwdScale >= 80.0) speed = 1;
	else if (fwdScale >= 60.0 && fwdScale < 80.0) speed = 3;
	else if (fwdScale >= 40.0 && fwdScale < 60.0) speed = 5;
	else if (fwdScale >= 20.0 && fwdScale < 40.0) speed = 7;
	else if (fwdScale < 20.0) speed = 9;
	
	if(PIDout < 0.0) speed *= -1;
	
	return speed;
}

int rotSpeedScaling(float PIDout) {
	float 	rotScale;
	int	speed;
	
	rotScale = fabs(PIDout);
	
	if (rotScale >= 10.0) speed = 4;
	else if (rotScale >= 9.0 && rotScale < 10.0) speed = 4;
	else if (rotScale >= 8.0 && rotScale < 9.0) speed = 5;
	else if (rotScale >= 7.0 && rotScale < 8.0) speed = 5;
	else if (rotScale >= 6.0 && rotScale < 7.0) speed = 5;
	else if (rotScale >= 5.0 && rotScale < 6.0) speed = 5;
	else if (rotScale >= 4.0 && rotScale < 5.0) speed = 6;
	else if (rotScale >= 3.0 && rotScale < 4.0) speed = 6;
	else if (rotScale >= 2.0 && rotScale < 3.0) speed = 6;
	else if (rotScale < 2.0) speed = 6;
	
	if(PIDout < 0.0) speed *= -1;
	
	return speed;
}

float get_euclidian_distance(float start_x, float start_y, float end_x, float end_y){
	float	diff1,
		diff2;
	diff1 = end_x - start_x;
	diff2 = end_y - start_y;
 	return sqrt( diff1 * diff1 + diff2 * diff2 );
}


float calcAngle(int p1_x, int p1_y, int p2_x, int p2_y) {
	
	return atan2( (float)(p2_y - p1_y), (float)(p2_x - p1_x) );
}

void rotate_to_theta(robot_if_t *ri, float target_theta, vector *current_location){
	float	output,
		rot_amount;
	int  	ang_vel;		
	vector *expected_vel;
	
	/* reset PID control for this rotation */
	reset_PID(rotPID);
	
	expected_vel = (vector *)calloc(1, sizeof(vector));
	
	do {
		printf("\n *********************  ROT PID ENABLED  ********************\n\n");
		printf("Curr Theta = %f\tTarget Theta = %f\n", current_location->v[2], target_theta);
		output = Compute(rotPID, current_location->v[2], target_theta);
		printf("Rotate PID Output = %f\n", output);
		
		// correlate output to an angular velocity
		ang_vel = rotSpeedScaling(output);
		
		if(ang_vel > 0) {
			if(ang_vel < 6) ri_move(ri, RI_TURN_LEFT_20DEG, ang_vel);
			else ri_move(ri, RI_TURN_LEFT, ang_vel);
			
			if(fabs(output) < 2.0) ri_move(ri, RI_STOP, ang_vel);
						
			expected_vel->v[0] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = rot_speed[ang_vel - 1] * 0.75;
		 }
		else {
			ang_vel *= -1;
			
			if(ang_vel < 6) ri_move(ri, RI_TURN_RIGHT_20DEG, ang_vel);
			else ri_move(ri, RI_TURN_RIGHT, ang_vel);
			
			if(fabs(output) < 2.0) ri_move(ri, RI_STOP, ang_vel);
						
			expected_vel->v[0] = 0.0;//-1 * NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = 0.0;//-1 * NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = -1.0 * rot_speed[ang_vel - 1] * 0.75;
		}
		
				 
		get_Position(ri, current_location, expected_vel, ROTATE);
		
		printf("Kalmann filtered result = %f\t%f\t%f\n\n", current_location->v[0], current_location->v[1], current_location->v[2]);
		
		rot_amount = fabs(target_theta - current_location->v[2]);		
	} while (rot_amount > 0.15);
	
	ri_move(ri, RI_STOP, 1);
	
	free(expected_vel);
}

void go_to_position(robot_if_t *ri, float end_x, float end_y){
 	float	setpoint,
		x_i,
		y_i,
		current_distance,
		output,
		theta_target,
		error,
		tolerance;
	int	i,
		bot_speed;
	tolerance = 10.0;//adjustable.  How close should I be to the waypoint before moving onto the next one?
	vector 	*current_location,
		*expected_vel;
		
	current_location = (vector *)calloc(1, sizeof(vector));
	expected_vel = (vector *)calloc(1, sizeof(vector));
 	
	/* reset PID control for this move */
	reset_PID(fwdPID);
		
 	//initialize start_x, start_y, start_theta
	get_Position(ri, current_location, expected_vel, FORWARD);
	x_i = current_location->v[0];
	y_i = current_location->v[1];
	
	// find initial theta to target in case we need to rotate immediately
	theta_target = calcAngle(x_i, y_i, end_x, end_y);
	
	// point robot at destination using PID 
	if( fabs(theta_target - current_location->v[2]) > 0.15) rotate_to_theta(ri, theta_target, current_location);	
	  
	i = 10;
	// setpoint is the exact number of cm required to move to target.
	setpoint = get_euclidian_distance(current_location->v[0], current_location->v[1], end_x, end_y);
	
	do {
		current_distance = get_euclidian_distance(x_i, y_i, current_location->v[0], current_location->v[1]);
		error = setpoint - current_distance;
		//setpoint = current_distance + error;
		
		/*
		theta_target = calcAngle(current_location->v[0], current_location->v[1], end_x, end_y);
		
		// ROTATE DURING FORWARD RUN ? 
		if(fabs(theta_target - current_location->v[2]) > 0.4) {
			
			rotate_to_theta(ri, theta_target, current_location);
			
			// reset PID control for rest of move
			reset_PID(fwdPID);
			sf = 0.1;
			i = 0;
		} */
		
		// For the first 10 iterations of action loop, pull out slowly 
		if ( i > 0 ) {
			bot_speed = i;  
		}
		else bot_speed = RI_FASTEST;
		
		// At the FWD PID Setpoint, turn on PID control to reduce speed and stop.
		if( error <= 40 ) {
			  output = Compute(fwdPID, current_distance, setpoint );
		  
			  printf("\n *********************  FWD PID ENABLED  ********************\n\n");
			  printf("CurrDist = %f\tError = %f\n", current_distance, error);			  
			  printf("FWD PID Output = %f\n", output);
			  
			  // correlate output to a bot_speed NEGATIVE SPEEDS MOVE THE BOT BACKWARDS
			    
			  bot_speed = fwdSpeedScaling(output);
		}
		
		// move the bot based on bot_speed and define expected velocities for kalmann filter
		if(bot_speed > 0) {
			ri_move(ri, RI_MOVE_FORWARD, bot_speed);
			/* expected velocities now scaled in half to compensate for network lag */
			expected_vel->v[0] = fwd_speed[bot_speed - 1] * cos(current_location->v[2]) * 0.5;
			expected_vel->v[1] = fwd_speed[bot_speed - 1] * sin(current_location->v[2]) * 0.5;
		}
		else {
			bot_speed *= -1;
			ri_move(ri, RI_MOVE_BACKWARD, bot_speed);
			/* expected velocities now scaled in half to compensate for network lag */
			expected_vel->v[0] = -1.0 * fwd_speed[bot_speed - 1] * cos(current_location->v[2]) * 0.5;
			expected_vel->v[1] = -1.0 * fwd_speed[bot_speed - 1] * sin(current_location->v[2]) * 0.5;
		}
		
		expected_vel->v[2] = 0.0;
		
		/* decriment i for wind up */
		i--;
		
		//refresh current position values and see if bot changed rooms.  If it does, reset scaling factor
   		get_Position(ri, current_location, expected_vel, FORWARD);
		
		printf("Kalmann filtered result = %f\t%f\t%f\n", current_location->v[0], current_location->v[1], current_location->v[2]);		
	} while( error > tolerance );
	
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
	//vector 	*location = (vector *)calloc(1, sizeof(vector));
	float 	target_x,
		target_y;
	    
	float waypoints[NUMBER_OF_WAYPOINTS][2] = WAYPOINT_COORDS;//WAYPOINT_COORDS;
	int numWayPoints = NUMBER_OF_WAYPOINTS, index;
	        
        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0)) printf("Failed to setup the robot!\n");
	
	if(ri_getHeadPosition(&ri) != RI_ROBOT_HEAD_LOW ) ri_move(&ri, RI_HEAD_DOWN , 1);
	
	// Check condition of battery, exit if not enough charge
	//battery_check(&ri);

	// Initialize PID controllers
	fwdPID = calloc(1, sizeof(PID));
	rotPID = calloc(1, sizeof(PID));
	init_PID(fwdPID, F_Kp, F_Ki, F_Kd);
	init_PID(rotPID, R_Kp, R_Ki, R_Kd);
	
	// Retrieve initial position, initailize current and last
	init_pos(&ri);

	//waypoint nav:
	for(index = 0; index < numWayPoints; index++){
		target_x = waypoints[index][0];
		target_y = waypoints[index][1];
		go_to_position(&ri, target_x, target_y);
		/*
		if(!ri_IR_Detected(&ri)) {
			printf("I found an obstacle!  Stopping!\n\n");
			exit(-10);
		}
		*/
		printf("\n *********************  Waypoint %d Reached  ********************\n\n", (index+1));
		//ri_move(&ri, RI_HEAD_MIDDLE , 1);
		//ri_move(&ri, RI_HEAD_DOWN , 1);
		
		getc(stdin);
	}
	
	free(fwdPID);
	free(rotPID);
	
	exit_pos();
	
	return 0;
}
