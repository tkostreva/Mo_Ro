#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "northstar.h"
#include "wheel_encoder.h"
#include "position.h"

// Set DATA_COLLECT to 1 to spin the bot
#define DATA_COLLECT 1


// Update the robot's sensor information
void update_sensor_data( robot_if_t *ri ) {
	// If first sensor update fails to respond, try one more time before giving up
	if(ri_update(ri) != RI_RESP_SUCCESS) {
		if(ri_update(ri) != RI_RESP_SUCCESS) {
			printf("Failed to update sensor information!\n");
			exit(10);
		}
        }
}

void battery_check( robot_if_t *ri ) {
	if( ri_getBattery(ri) > RI_ROBOT_BATTERY_OFF ) {
		printf("Charge me please!!!");
		exit(11);
	}
	
}

int main(int argv, char **argc) {
	int i;
	robot_if_t ri;
	ns_stance initial, current, last;
	we_stance we_current, we_sum;
	float dist_y, dist_x, target_dist, d_theta;
	
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
	update_sensor_data(&ri);
	get_ns(&initial, &ri);
	copy_ns(&current, &initial);
	copy_ns(&last, &initial);
	
	get_we_totals(&we_current, &ri);
	we_sum.right = we_sum.left = we_sum.back = 0;

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
		i++;
        } while(i < 25);

// Normal Code
#else
	
	dist_y = 0.0;
	get_we_totals(&we_current, &ri);
	we_sum.right = we_sum.left = we_sum.back = 0;
	
	print_ns(&initial);
	
        // Action loop
        do {
                // Move forward unless there's something in front of the robot
                if(!ri_IR_Detected(&ri)) {
			d_theta = ( current.theta + last.theta ) / 2.0;
			// Straigten out robot if neccessary
			if ( d_theta > ( initial.theta + 0.175 ) ) {
				ri_move(&ri, RI_TURN_RIGHT, 6);
				ri_move(&ri, RI_MOVE_FWD_RIGHT, RI_SLOWEST);
			}
			else if ( d_theta < ( initial.theta - 0.175 ) ) {
				ri_move(&ri, RI_TURN_LEFT, 6);
				ri_move(&ri, RI_MOVE_FWD_LEFT, RI_SLOWEST);
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
		
		print_ns(&current);
		print_we(&we_current);
		print_we(&we_sum);
		
		dist_y += get_we_dist_FB(&we_current);
		printf("Distance from start = %fcm\n", dist_y);
        } while(dist_y < target_dist);
#endif
	
	exit_ns();
	
	return 0;
}
