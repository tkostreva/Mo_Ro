#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

// Set DATA_COLLECT to 1 to dump data for csv files
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
	
	exit_pos();
	
	return 0;
}
