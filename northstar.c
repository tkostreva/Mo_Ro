#include "northstar.h"

ns_filter ns_f;

// Initialize a northstar filter structure
void init_ns_filter() {
	ns_f.x_filter = fir_Filter_Create();
	ns_f.y_filter = fir_Filter_Create();
	ns_f.t_filter = fir_Filter_Create();
	ns_f.s_filter = fir_Filter_Create();
}

// Free northstar filter structure on exit from program
void exit_ns() {
	free(ns_f.x_filter);
	free(ns_f.y_filter);
	free(ns_f.t_filter);
	free(ns_f.s_filter);
}

float delta_theta(robot_if_t *ri) {
	int sign;
	static float last_theta = 0.0;
	float d_theta;
	float curr_theta = ri_getTheta(ri);
	
	if(abs(curr_theta - last_theta) > M_PI){
		if(lastTheta<0)
			sign = -1;
		else
			sign = 1;
		d_theta = sign * ((M_PI-abs(curr_theta)) + (M_PI-abs(last_theta)));
	}
	else d_theta = curr_theta - last_theta;
	
	last_theta = curr_theta;
	
	return d_theta;	
}

void ns_filter_flush(robot_if_t *ri) {
	int i;
	
	for(i = 0; i < DEEP_TAPS - 1; i++) {
		fir_Filter(ns_f.x_filter, (float)ri_getX(ri), DEEP_FILTER);
		fir_Filter(ns_f.y_filter, (float)ri_getY(ri), DEEP_FILTER);
		fir_Filter(ns_f.t_filter, ri_getTheta(ri), SHALLOW_FILTER);
		fir_Filter(ns_f.s_filter, (float)ri_getNavStrengthRaw(ri), SHALLOW_FILTER);
		update_sensor_data(ri);
	}
}

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri ) {
	static char initialized = 0;
			
	// initialize filters on first retrieval of north star info
	if (initialized == 0) {
		//printf("INITIALIZING NORTH STAR FILTERS\n");
		init_ns_filter();
		initialized = 1;
	}
	
	s->x = (int)fir_Filter(ns_f.x_filter, ri_getX(ri), DEEP_FILTER);
	s->y = (int)fir_Filter(ns_f.y_filter, ri_getY(ri), DEEP_FILTER);
	s->theta =fir_Filter(ns_f.t_filter, ri_getTheta(ri), SHALLOW_FILTER);
	s->sig =  (int)fir_Filter(ns_f.s_filter, (float)ri_getNavStrengthRaw(ri), SHALLOW_FILTER);
	s->room = ri_getRoomID(ri);
	
}

// S is a deep copy of R  (S gets copy, R is original)
void copy_ns(ns_stance *s, ns_stance *r) {
	s->x = r->x;
	s->y = r->y;
	s->theta = r->theta;
	s->room = r->room;
	s->sig = r->sig;
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
