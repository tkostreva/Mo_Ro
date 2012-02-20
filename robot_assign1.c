#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "position.h"
#include "PID_Control.h"

/* DEFINES */
#define WAYPOINT_COORDS {{342.9, 0.0},{243.84, 182.88},{297.18, 182.88},{406.400, 302.26},{060.96, 403.86},{0,0}}
#define NUMBER_OF_WAYPOINTS 6 /*6 {342.9, 0.0}*/
//#define WAYPOINT_COORDS {{150.0,0.0},{150.0,0.0}}
//#define NUMBER_OF_WAYPOINTS 1

#define F_Kp 1.0
#define F_Ki 0.1
#define F_Kd 0.01

#define R_Kp 10.0/M_PI
#define R_Ki 0.5
#define R_Kd 0.20

#define LANE_LIMIT 25.0
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
	
	if (temp >= 10.0) speed = 5;
	else if (temp >= 9.0 && temp < 10.0) speed = 5;
	else if (temp >= 8.0 && temp < 9.0) speed = 5;
	else if (temp >= 7.0 && temp < 8.0) speed = 5;
	else if (temp >= 6.0 && temp < 7.0) speed = 5;
	else if (temp >= 5.0 && temp < 6.0) speed = 6;
	else if (temp >= 4.0 && temp < 5.0) speed = 7;
	else if (temp >= 3.0 && temp < 4.0) speed = 7;
	else if (temp >= 2.0 && temp < 3.0) speed = 7;
	else if (temp < 2.0) speed = 7;
	
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

/* find slope of line to be traveled in our coordinate system */
float get_slope(float start_x, float start_y, float end_x, float end_y){
	return ( (end_y - start_y) / (end_x - start_x) );
}

/* find y intercept of line to be traved in our coordinate system */
float get_intercept(float m, float end_x, float end_y){
	return ( end_y - m * end_x );
}

/* current_theta-get_theta_to_target() = theta to correct */
float get_theta_to_target(float start_x, float start_y, float end_x, float end_y){
	//this needs to be made more robust... what happens after it turns around?
  
 	float theta_to_target = atan( (end_y - start_y) / (end_x-start_x) );
	if( end_x < start_x ){//its gonna be outside of the range of arctan//this heuristic isnt perfect--> consider negative motion
   		if( end_y  > start_y){
			printf("theta > 90 case1 -> left  ");
     			theta_to_target = M_PI + theta_to_target; //turn left
		}else if(start_y>end_y){
    			printf("theta > 90 case2 -> right ");
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
		rot_amount = fabs(target_theta - current_location->v[2]);
		
		printf("\n *********************  ROT PID ENABLED  ********************\n\n");
		printf("Curr Theta = %f\tTarget Theta = %f\n", current_location->v[2], target_theta);
		output = Compute(rotPID, current_location->v[2], target_theta);
		printf("Rotate PID Output = %f\n", output);
		
		// correlate output to an angular velocity this is a crappy substitute
		ang_vel = rotSpeedScaling(output);
		 
		if(ang_vel > 0) {
			ri_move(ri, RI_TURN_LEFT, ang_vel);
			expected_vel->v[0] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * sin(current_location->v[2]);
			expected_vel->v[1] = 0.0;//NS_RADIUS * rot_speed[ang_vel - 1] * cos(current_location->v[2]);
			expected_vel->v[2] = rot_speed[ang_vel - 1];
		 }
		else {
			ang_vel *= -1;
			ri_move(ri, RI_TURN_RIGHT, ang_vel);
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
	} while (rot_amount > 0.25);  // found the granularity of turning is roughly 0.45 radians per turn (single plug...  speed = 6
	
	free(expected_vel);
}

void go_to_position(robot_if_t *ri, float end_x, float end_y){
 	float	setpoint,
		x_i,
		y_i,
		current_distance,
		output,
		slope,
		intercept,
		sf,
		upper_limit,
		lower_limit,
		theta_target,
		distance_to_target,
		tolerance;
	int	i,
		bot_speed;
	tolerance = 20.0;//adjustable.  How close should I be to the waypoint before moving onto the next one?
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
	theta_target = get_theta_to_target(x_i, y_i, end_x, end_y);
	
	// point robot at destination using PID 
	if( (theta_target - current_location->v[2]) > 0.1) rotate_to_theta(ri, theta_target, current_location);	
	  
	// find slope and intercept of line to be traveled to define limits on path
	slope = get_slope(current_location->v[0], current_location->v[1], end_x, end_y);
	intercept = get_intercept(slope, end_x, end_y);
	tolerance = 5.0;
	sf = 0.1;
	i = 0;
	
	do {
		current_distance = get_euclidian_distance(x_i, y_i, current_location->v[0], current_location->v[1]);
		distance_to_target = get_euclidian_distance(current_location->v[0], current_location->v[1], end_x, end_y);
		setpoint = current_distance + distance_to_target;
		
		upper_limit = slope * current_location->v[0] + intercept + LANE_LIMIT;
		lower_limit = slope * current_location->v[0] + intercept - LANE_LIMIT;
		
		//printf("Upper Lim = %f\tLower Lim = %f\n", upper_limit, lower_limit);
		
		if(((current_location->v[1] >= upper_limit) || (current_location->v[1] <= lower_limit )) && i >=10) {
			theta_target = get_theta_to_target(current_location->v[0], current_location->v[1], end_x, end_y);
			rotate_to_theta(ri, theta_target, current_location);
			
			/* reset PID control for rest of move */
			reset_PID(fwdPID);
			sf = 0.1;
			i = 0;
		}
		
		//move to reduce error at fill speed, then using PID in last 75 cm
		if( distance_to_target <= 75 ) {
			  printf("\n *********************  FWD PID ENABLED  ********************\n\n");
			  printf("CurrDist = %f\tDist to Target = %f\n", current_distance, distance_to_target);
			  output = Compute(fwdPID, current_distance, setpoint );
			  printf("FWD PID Output = %f\n", output);
			  
			  // correlate output to a bot_speed NEGATIVE SPEEDS MOVE THE BOT BACKWARDS
			  
			  // shitty patch until we figure out correlation
			  /*if(output < 0) bot_speed = -RI_SLOWEST;
			  else bot_speed = RI_SLOWEST;*/
			  bot_speed = fwdSpeedScaling(output);
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
		
		expected_vel->v[0] *= sf;
		expected_vel->v[1] *= sf;
		
		/* incriment scaling factor for expected velocites during wind up */
		if (sf < 1.0) sf += 0.1;
		i++;
		
		//refresh current position values and see if bot changed rooms.  If it does, reset scaling factor
   		if( get_Position(ri, current_location, expected_vel, FORWARD) ) sf = 0.1;
		
		printf("Kalmann filtered result = %f\t%f\t%f\n", current_location->v[0], current_location->v[1], current_location->v[2]);		
		
		
 	} while((fabs(distance_to_target) > tolerance) ); // && (!ri_IR_Detected(ri)));

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
	}
	
	free(fwdPID);
	free(rotPID);
	
	exit_pos();
	
	return 0;
}
