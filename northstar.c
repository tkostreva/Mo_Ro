/*
 * 	Now only reports raw NorthStar Data....  filtering will be done in position.c
 */

#include "northstar.h"

vector *shift_vector;
matrix *clockwise_matrix;
matrix *scale_matrix;

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri ) {
	s->x = ri_getX(ri);
	s->y = ri_getY(ri);
	s->theta = ri_getTheta(ri);
	s->sig =  ri_getNavStrengthRaw(ri);
	s->room = ri_getRoomID(ri);
	
}

void init_transforms(ns_stance *s) {
	// free pointers in case previously declared
	free(shift_vector);
	free(clockwise_matrix);
	free(scale_matrix);
  
	// get memory for pointers
	shift_vector = calloc(1, sizeof(vector));
	clockwise_matrix = calloc(1, sizeof(matrix));
	scale_matrix = calloc(1, sizeof(matrix));
	
	// initialize shift vector
	shift_vector->v[0] = (-1)*(s->x);
	shift_vector->v[1] = (-1)*(s->y);
	shift_vector->v[2] = (-1)*(s->theta);
	
	//initialize clockwise_matrix
	
	clockwise_matrix->v[0][0] = (double)cos(s->theta);
	clockwise_matrix->v[0][1] = (double)sin(s->theta);
	clockwise_matrix->v[0][2] = 0.0;
	
	clockwise_matrix->v[1][0] = (double)( -1 * sin(s->theta) );
	clockwise_matrix->v[1][1] = (double) cos(s->theta);
	clockwise_matrix->v[1][2] = 0.0;
	
	clockwise_matrix->v[2][0] = 0.0;
	clockwise_matrix->v[2][1] = 0.0;
	clockwise_matrix->v[2][2] = 1.0;
	
	//initialize scale_matrix
	scale_matrix->v[0][0] = 1/NS_TICKS_PER_CM;
	scale_matrix->v[0][1] = 0.0;
	scale_matrix->v[0][2] = 0.0;
	
	scale_matrix->v[1][0] = 0.0;
	scale_matrix->v[1][1] = 1/NS_TICKS_PER_CM;
	scale_matrix->v[1][2] = 0.0;
	
	scale_matrix->v[2][0] = 0.0;
	scale_matrix->v[2][1] = 0.0;
	scale_matrix->v[2][2] = 1.0;	//convert to degrees? do it here if needed
}


vector *transformNS(ns_stance *s){//in progress
	//use clockwise rotation matrix //(i think since initial theta represents ccw)
	//shift + --> rotate * --> scale *
	
	//initialize shift_vector
	vector currentns_vector;
	vector working_vector; 
	vector working_vector_2;
	vector *ns_vector = calloc(1, sizeof(vector));
	
	//initialize currentns_vector
	ns_vector->v[0] = (s->x);
	ns_vector->v[1] = (s->y);
	ns_vector->v[2] = (s->theta);
	PrintVector(ns_vector);//diagnostic
	
	//shift
	AddVectors(ns_vector, shift_vector, &working_vector);
	PrintVector(&working_vector);//diagnostic
	
	//rotate
	MultMatVec(clockwise_matrix, &working_vector, &working_vector_2);
	PrintVector(&working_vector_2);//diagnostic
	
	//scale
	MultMatVec(scale_matrix, &working_vector_2, ns_vector);
	PrintVector(current->nsTranslated);//diagnostic
	// free all working vectors
	
	return ns_vector;
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

// Print out a northstar stance structure
void print_ns(ns_stance *s) {
	printf("X = %d\tY = %d\tTheta = %f\tRaw Signal = %d\tRoomID = %d\n", 
	       s->x, s->y, s->theta, s->sig, s->room);
}

// Print routine for data dump to CSV file
void print_ns_csv(ns_stance *s){
	printf("%d, %d, %f, %d, %d", s->x, s->y, s->theta, s->sig, s->room);
}

int ns_ticksToCM(int ticks){
    return ticks/45;//this is going off of the C API, In reality this conversion should be non-linear
}
