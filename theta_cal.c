/*  Filename:	theta_cal.c
 *  Author:	Tim Kostreva
 *  Date:	2/2/12
 *  Purpose:	To identify the theta value used for northstar coordiante transformations.
 */

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "position.h"

int main(int argv, char **argc) {
	robot_if_t ri;		/* robot object */
	vector 	*u,		/* Initial position vector */
		*w;		/* Final position vector */
	int 	room_id_u,	/* Room ID's of respective vectors */
		room_id_w;
	float 	d_x, 		/* variables for computation of theta */
		d_y, 
		d_theta,
		result;
	
	/* Initialize memory for vectors */
	u = (vector *)calloc(1, sizeof(vector));
	w = (vector *)calloc(1, sizeof(vector));
	
        /* Make sure we have a valid command line argument */
        if(argv <= 1) {
                printf("Usage: robot_test <address of robot>\n");
                exit(-1);
        }

        /* Setup the robot with the address passed in */
        if(ri_setup(&ri, argc[1], 0))
                printf("Failed to setup the robot!\n");
	
	/* Place bot, prompt, then retrieve initial position */
	printf("Please place the robot facing positive X for initial position.\nHit the any key to continue!");
	getc(stdin);
	printf("\n\n");
	
	init_pos(&ri);

	/* Get Room ID of first postion */
	room_id_u = NS_theta_cal(&ri, u);
	
	/* Prompt to move the bot, retrieve final position */
	printf("Pleae move the robot forward APPROX 1 meter without changing its orientation.\nHit the any key to continue!");
	getc(stdin);
	printf("\n\n");
	init_pos(&ri);
	
	/* Get Room ID of final postion */
	room_id_w = NS_theta_cal(&ri, w);
	
	/* Check to see if crossed rooms */
	if(room_id_u != room_id_w) {
		printf("You crossed rooms, try again!");
		exit(0);
	}
	
	/* Check to see if orientation of the robot was changed too much during the move */
	d_theta = fabs(w->v[2] - u->v[2]);
	
	if( d_theta > 0.25 ) {
		printf("Difference in theta orientations too large, try again!");
		exit(1);
	}
	
	/* Display vectors */
	printf("U = ");
	PrintVector(u);
	printf("W = ");
	PrintVector(w);
	
	/* Calculate Angle with inverse tangent of change in x over change in y */
	d_x = w->v[0] - u->v[0];
	d_y = w->v[1] - u->v[1];
	
	result = atan(d_y / d_x);
	
	printf("Theta Correction for room %d = %f radians\n", room_id_u, result);
	
	/* clean up memory */
	free(u);
	free(w);
	
	exit_pos();
	
	return 0;
}
