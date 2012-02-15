#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "position.h"
#include "PID_Control.h"

/* DEFINES */
#define F_Kp 3.0
#define F_Kd 0.3
#define F_Ki 0.03

#define R_Kp 3.0
#define R_Kd 0.3
#define R_Ki 0.03

#define LANE_LIMIT 14.5  /* currently the radius of the bot in [cm] */
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

float get_euclidian_distance(float start_x, float start_y, float end_x, float end_y){
	float	diff1,
		diff2;
	diff1 = end_x - start_x;
	diff2 = end_y - start_y;
 	return sqrt( diff1 * diff1 + diff2 * diff2 );
}

/* find slope of line to be traveled in our coordinate system */
float get_slope(float start_x, float start_y, float end_x, float end_y){
	return ( (end_y - start_y) / (end_x - start_x) );
}

/* find y intercept of line to be traved in our coordinate system */
float get_intercept(float m, float end_x, float end_y){
	return ( end_y - m * end_x );
}

/*  
 *  current_theta-get_theta_to_target() = theta to correct
 */
float get_theta_to_target(float start_x, float start_y, float end_x, float end_y){
 	float theta_to_target = atan( (end_y - start_y) / (end_x-start_x) );
	
 	if( end_x < start_x ){//its gonna be outside of the range of arctan
   		if( end_y  > start_y)
     			theta_to_target = M_PI + theta_to_target; //turn left
   		else if(start_y>end_y)
    			theta_to_target = M_PI - theta_to_target;//turn right
   		else//it needs to make a 180
     			theta_to_target = M_PI;//180 degrees
 	}
 	
 	return theta_to_target;
}

void rotate_to_theta(robot_if_t *ri, float target_theta, vector *current_location){
	float	rot_amount,
		output,
		test;
	int  	ang_vel;
	vector *expected_vel;
	
	/* reset PID control for this rotation */
	reset_PID(rotPID);
	
	expected_vel = (vector *)calloc(1, sizeof(vector));
	
	rot_amount = target_theta - current_location->v[2];
	
	do {
		 output = Compute(rotPID, current_location->v[2], rot_amount);
		 printf("Rotate PID Output = %f\n", output);
		 
		 // correlate output to an angular velocity this is a crappy substitute
		 if(output > 0) ang_vel = 6;
		 else ang_vel = -6;
		 
		 if(ang_vel > 0) {
			ri_move(ri, RI_TURN_LEFT, ang_vel);
			expected_vel->v[0] = NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = rot_speed[ang_vel - 1];
		 }
		 else {
			ang_vel *= -1;
			ri_move(ri, RI_TURN_RIGHT, ang_vel);
			expected_vel->v[0] = -1 * NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = -1 * NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = -1 * rot_speed[ang_vel - 1];
		 }
		 
		 get_Position(ri, current_location, expected_vel); 
		 
		 if(output < 0.0) test = -1.0 * output;
		 else test = output;
	} while (test > 10);  // once again, very very arbitrary
	
	free(expected_vel);
}

void go_to_position(robot_if_t *ri, float end_x, float end_y){
 	float	distance_to_target,
		current_distance,
		output,
		slope,
		intercept,
		upper_limit,
		lower_limit,
		theta_target;
	int	bot_speed;
	vector 	*current_location,
		*expected_vel;
		
	current_location = (vector *)calloc(1, sizeof(vector));
	expected_vel = (vector *)calloc(1, sizeof(vector));
 	
	/* reset PID control for this move */
	reset_PID(fwdPID);
		
 	//initialize start_x, start_y, start_theta
	get_Position(ri, current_location, expected_vel);
	
	distance_to_target = get_euclidian_distance(current_location->v[0], current_location->v[1], end_x, end_y);
	
	// find initial theta to target in case we need to rotate immediately
	theta_target = get_theta_to_target(current_location->v[0], current_location->v[1], end_x, end_y);
	
	// point robot at destination using PID 
	if( (theta_target - current_location->v[2]) > 0.1) rotate_to_theta(ri, theta_target, current_location);	
	  
	// find slope and intercept of line to be traveled to define limits on path
	slope = get_slope(current_location->v[0], current_location->v[1], end_x, end_y);
	intercept = get_intercept(slope, end_x, end_y);
	
	do {
		current_distance = get_euclidian_distance(current_location->v[0], current_location->v[1], end_x, end_y);
		
		upper_limit = slope * current_location->v[0] + intercept + LANE_LIMIT;
		lower_limit = slope * current_location->v[0] + intercept - LANE_LIMIT;
		
		if((current_location->v[1] >= upper_limit) || (current_location->v[1] <= lower_limit)) {
			printf("upper_limit = %f\t lower_limit = %f\n",upper_limit,lower_limit);
			theta_target = get_theta_to_target(current_location->v[0], current_location->v[1], end_x, end_y);
			rotate_to_theta(ri, theta_target, current_location);
			
			/* reset PID control for rest of move */
			reset_PID(fwdPID);
		}
		
		//move to reduce error at fill speed, then using PID in final quarter
		//if( current_distance <= 0.25 * distance_to_target ) {
		if(1) {
			  output = Compute(fwdPID, current_distance, distance_to_target);
			  printf("FWD PID Output = %f\n", output);
			  
			  // correlate output to a bot_speed NEGATIVE SPEEDS MOVE THE BOT BACKWARDS
			  
			  // shitty patch until we figure out correlation
			  if(output < 0) bot_speed = -RI_SLOWEST;
			  else bot_speed = RI_SLOWEST;
		}
		else bot_speed = RI_FASTEST;
		
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
		
   		//refresh current position values
   		get_Position(ri, current_location, expected_vel);
		
		printf("Kalman Data = %f\t%f\tcurrent_distance = %f\n",current_location->v[0], current_location->v[1], current_distance);
		printf("Excepted velcoities = %f\t%f\n", expected_vel->v[0], expected_vel->v[1]);
 	}while( output < 1000  );  // chose a VERY ARBITRARY NUMBER FOR THE MOMENT 
 	//while( current_distance > (0.05 * distance_to_target) || output < 400  );  // chose a VERY ARBITRARY NUMBER FOR THE MOMENT

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
		target_y,
		scalar_target_dist,
		d_theta;
	    
	// Make sure we have a valid command line argument
        if(argv <= 3) {
                printf("Usage: robot_test <address of robot> <distance to travel in X in cm> <distance to travel in Y in cm>\n");
                exit(-1);
        }
        
        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");
	
	// Check condition of battery, exit if not enough charge
	//battery_check(&ri);

	// Initialize PID controllers
	fwdPID = calloc(1, sizeof(PID));
	rotPID = calloc(1, sizeof(PID));
	SetTunings(fwdPID, F_Kp, F_Ki, F_Kd);
	SetTunings(rotPID, R_Kp, R_Ki, R_Kd);
	
	// Retrieve target distance from command line
	target_x = (float) atoi(argc[2]);
	target_y = (float) atoi(argc[3]);
	//scalar_target_dist = get_euclidian_distance(0.0, 0.0, target_x, target_y);
			
	// Retrieve initial position, initailize current and last
	init_pos(&ri);

#if (DATA_COLLECT)
	print_stance_csv();
#endif
	go_to_position(&ri, target_x, target_y);
	/*
        // Action loop
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
		// Bad attempt at PID
			d_theta = turn_to();
			//printf("Delta Theta = %f\n", d_theta);
			
			// Straigten out robot if neccessary
			if ( d_theta > 0.07 ) {
				ri_move(&ri, RI_TURN_RIGHT, 6);
				//ri_move(&ri, RI_MOVE_FWD_RIGHT, RI_SLOWEST);
				ri_move(&ri, RI_MOVE_FORWARD, 5);
				printf("-----------------Robot Turning Right---------------------------\n");
				while (1){
					get_Position(&ri, location);
					
					//check right rotation
					if (check_rotation(0) == 1)
						break;
					ri_move(&ri, RI_TURN_RIGHT, 6);
					d_theta = turn_to();
				}
					
				update_theta("Right");
			}
			else if ( d_theta < -0.07) {
				ri_move(&ri, RI_TURN_LEFT, 6);
				//ri_move(&ri, RI_MOVE_FWD_LEFT, RI_SLOWEST);
				ri_move(&ri, RI_MOVE_FORWARD, 5);
				printf("-----------------Robot Turning Left---------------------------\n");
				while (1){
					get_Position(&ri, location);
					
					//check left rotation
					if (check_rotation(1) == 1)
						break;
					ri_move(&ri, RI_TURN_LEFT, 6);
					d_theta = turn_to();
				}
				update_theta("Left");
			}
			else {
				if(location->v[0] < 0.8 * scalar_target_dist) ri_move(&ri, RI_MOVE_FORWARD,1);
				else ri_move(&ri, RI_MOVE_FORWARD, RI_SLOWEST);
			}
		}
		else {
			printf("I found an obstacle!  Staying Safe!\n\n");
			exit(-1);
			
			printf("I found an obstacle!  Turning Right!\n\n");
			while(ri_IR_Detected(&ri)){
				ri_move(&ri, RI_TURN_RIGHT, 6);
				ri_move(&ri, RI_MOVE_FORWARD, 5);
				update_theta("Right");
			}
			
		}
		
					
		get_Position(&ri, location);
#if (DATA_COLLECT)
		print_stance_csv();
#else
		//printf("Location:  X = %f cm\tY = %f cm\ttarget = %f\n", location->v[0], location->v[1], target_x);
		printf("Kalmann filtered result = %f\t%f\t%f\n", location->v[0], location->v[1], location->v[2]);
#endif
        } while(location->v[1] > target_x);

	
	free(location);
	
	*/
	free(fwdPID);
	free(rotPID);
	
	exit_pos();
	
	return 0;
}
