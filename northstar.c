/*
 * 	Now only reports raw NorthStar Data....  filtering will be done in position.c
 */

#include "northstar.h"

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri ) {
	s->x = ri_getX(ri);
	s->y = ri_getY(ri);
	s->theta = ri_getTheta(ri);
	s->sig =  ri_getNavStrengthRaw(ri);
	s->room = ri_getRoomID(ri);
	
}

float get_ns_dist(ns_stance *current, ns_stance *last) {
	float x_factor, y_factor, avg_sig;
	float x1, x2, y1, y2;
	
	// find average signal strength between current and last stance
	avg_sig = (float)(current->sig + last->sig) / 2.0;
	
	// Initialize factors
	x_factor = y_factor = 1.0;
	
	// update factors based on signal strength
	if ( avg_sig < 13000.0 ) {
		// some arbitrary equations that need work to scale NS to signal strength
		x_factor = ( -0.222 * avg_sig + 3524 ) / 425.0;
		y_factor = ( -0.417 * avg_sig + 6251 ) / 486.0;
	}
	
	// scale current coords and previous coords by current factors to feed them to euclidean distance function
	x2 = (float) current->x / x_factor;
	y2 = (float) current->y / y_factor;
	x1 = (float) last->x / x_factor;
	y1 = (float) last->y / y_factor;
	
	// get distance between two points, divide by ticks to cm to get distance traveled
	return euclidean_distance(x1, y1, x2, y2) / NS_TICKS_TO_CM;
}

// get euclidean distance between two points ( point 2 - point 1 )
//  THIS NEEDS DOUBLE CHECKED!--> its good - Spencer
float euclidean_distance(float x1, float y1, float x2, float y2) {
	float sum;
	// sum squares of differences
	sum = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	// return square root of sum
	return sqrt( (float) sum );
	  
}

// Print out a northstar stance structure
void print_ns(ns_stance *s) {
	printf("X = %d\tY = %d\tTheta = %f\tRaw Signal = %d\tRoomID = %d\n", 
	       s->x, s->y, s->theta, s->sig, s->room);
}

// Print routine for data dump to CSV file
void print_ns_csv(ns_stance *s){
	static char init = 0;
	
	// print out header for CSV file on first pass
	if ( init == 0 ) {
		printf("x, y, Theta, RawSig, Room\n");
		init = 1;
	}
	
	printf("%d, %d, %f, %d, %d\n", s->x, s->y, s->theta, s->sig, s->room);
}

int ns_ticksToCM(int ticks){
    return ticks/45;//this is going off of the C API, In reality this conversion should be non-linear
}
