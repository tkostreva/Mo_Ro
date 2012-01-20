#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

// Set DATA_COLLECT to 1 to spin the bot (ELSE ZERO)
#define DATA_COLLECT 0


void battery_check( robot_if_t *ri ) {
	if( ri_getBattery(ri) > RI_ROBOT_BATTERY_OFF ) {
		printf("Charge me please!!!");
		exit(11);
	}	
}

int main(int argv, char **argc) {
	int i;
	robot_if_t ri;
	robot_stance initial, current;
	float dist_y, d_theta, target_dist;
	
        // Make sure we have a valid command line argument
        if(argv <= 1) {
                printf("Usage: robot_test <address of robot> <distance to travel in cm>\n");
                exit(-1);
        }

	// Make sure we have a valid command line argument
        if(argv <= 2) {
                printf("Usage: robot_test <address of robot> <distance to travel in cm>\n");
                exit(-1);
        }

        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");
	
	// Check condition of battery, exit if not enough charge
	battery_check(&ri);

	// Retrieve target distance from command line
	target_dist = (float) atoi(argc[2]);
	printf("Target Distance = %f\n, target_dist
			
	// Retrieve initial position from poistion.h
	init_pos(&ri);

// preprocessor will skip this if DATA_COLLECT is defined as ZERO
#if (DATA_COLLECT)
	print_rs_csv(&initial);
	
	i = 0;
        // Action loop
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
			ri_move(&ri, RI_TURN_RIGHT, 5);			
		}
		else {
			printf("I found an obstacle!  Stopping!\n\n");
			break;
		}
		
		print_stance_csv();
		i++;
        } while(i < 25);

// Normal Code
#else
	
	dist_y = 0.0;
		
	// Action loop
	printf("Moving into action loop.\n");
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
			d_theta = delta_theta();
			// Straigten out robot if neccessary
			if ( d_theta > ( initial.theta + 0.175 ) ) {
				printf("I want to turn right.\n");
				//ri_move(&ri, RI_TURN_RIGHT, 6);
				//ri_move(&ri, RI_MOVE_FWD_RIGHT, RI_SLOWEST);
			}
			else if ( d_theta < ( initial.theta - 0.175 ) ) {
				printf("I want to turn left.\n");
				//ri_move(&ri, RI_TURN_LEFT, 6);
				//ri_move(&ri, RI_MOVE_FWD_LEFT, RI_SLOWEST);
			}
			else {
				if(dist_y < 0.8 * target_dist) ri_move(&ri, RI_MOVE_FORWARD, RI_FASTEST);
				else ri_move(&ri, RI_MOVE_FORWARD, RI_SLOWEST);
			}
		}
		else {
			printf("I found an obstacle!  Stopping!\n\n");
			break;
		}
		
		dist_y = get_Distance(ri);
		printf("Distance from start = %f cm\n", dist_y);
        } while(dist_y < target_dist);
#endif
	
	exit_pos();
	
	return 0;
}