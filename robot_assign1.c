#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

// Set DATA_COLLECT to 1 to spin the bot
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
	vector *location = (vector *)calloc(1, sizeof(vector));
	float target_dist;
	
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
			
	// Retrieve initial position, initailize current and last
	init_pos(&ri);

// preprocessor will skip this if DATA_COLLECT is defined as ZERO
#if (DATA_COLLECT)
	print_ns_csv(&initial);
	
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
		
		/*
		// update the sensor data from the bot
		update_sensor_data(&ri);
                
                // Pass current position to last position for comparison
                copy_ns(&last, &current);
		
		// Get updated position
		get_ns(&current, &ri);
		get_we_delta(&we_current, &ri);
		we_sum.right += we_current.right;
		we_sum.left += we_current.left;
		we_sum.back += we_current.back;
		
		print_ns_csv(&current);
		//print_we(&we_current);
		//print_we(&we_sum);
		*/
		i++;
        } while(i < 25);

// Normal Code
#else
	
	//print_stance_csv();
	
        // Action loop
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
			// Straigten out robot if neccessary
			//if ( delta_theta() > 0.175 ) {
			//	ri_move(&ri, RI_TURN_RIGHT, 6);
			//	ri_move(&ri, RI_MOVE_FWD_RIGHT, RI_SLOWEST);
			//}
			//else if ( delta_theta() < -0.175 ) {
			//	ri_move(&ri, RI_TURN_LEFT, 6);
		//		ri_move(&ri, RI_MOVE_FWD_LEFT, RI_SLOWEST);
		//	}
		//	else {
				if(location->v[1] < 0.8 * target_dist) ri_move(&ri, RI_MOVE_FORWARD, RI_FASTEST);
				else ri_move(&ri, RI_MOVE_FORWARD, RI_SLOWEST);
		//	}
		}
		else {
			printf("I found an obstacle!  Stopping!\n\n");
			break;
		}
		
						
		get_Position(&ri, location);
		print_stance_csv();
		//printf("Location:  X = %f cm\tY = %f cm\n", location->v[0], location->v[1]);
        } while(location->v[1] < target_dist);
#endif
	free(location);
	
	exit_pos();
	
	return 0;
}
