#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

int main(int argv, char **argc) {
	robot_if_t ri;
	vector 	*u,
		*w;
	int 	room_id_u,
		room_id_w;
	float 	d_x, 
		d_y, 
		d_theta,
		result;
	
	u = (vector *)calloc(1, sizeof(vector));
	w = (vector *)calloc(1, sizeof(vector));
	
        // Make sure we have a valid command line argument
        if(argv <= 1) {
                printf("Usage: robot_test <address of robot>\n");
                exit(-1);
        }

        // Setup the robot with the address passed in
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");
	
	// Retrieve initial position, initailize current and last
	printf("Please place the robot facing positive Y for initial position.\nHit the any key to continue!");
	getc(stdin);
	printf("\n\n");
	
	init_pos(&ri);

	room_id_u = NS_theta_cal(&ri, u);
	
	printf("Pleae move the robot forward ONE meter without changing its orientation.\nHit the any key to continue!");
	getc(stdin);
	printf("\n\n");
	init_pos(&ri);
	
	room_id_w = NS_theta_cal(&ri, w);
	
	if(room_id_u != room_id_w) {
		printf("You crossed rooms, try again!");
		exit(0);
	}
	
	d_theta = w->v[2] - u->v[2];
	if( d_theta < 0.0 ) d_theta *= -1.0;
	
	if( d_theta > 0.25 ) {
		printf("Difference in theta orientations too large, try again!");
		exit(1);
	}
	
	printf("U = ");
	PrintVector(u);
	printf("W = ");
	PrintVector(w);
	
	d_x = w->v[0] - u->v[0];
	d_y = w->v[1] - u->v[1];
	
	result = atan(d_y / d_x);
	
	printf("Theta Correction for room %d = %f radians\n", room_id_u, result);
	
	free(u);
	free(w);
	
	exit_pos();
	
	return 0;
}
