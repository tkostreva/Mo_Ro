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

// Setup the transformation matrices with values stored in NS stance s
void setup_NS_transforms(ns_stance *s) {
	// free pointers in case previously declared
	free(shift_vector);
	free(clockwise_matrix);
	free(scale_matrix);
  
	// get memory for pointers
	shift_vector = calloc(1, sizeof(vector));
	clockwise_matrix = calloc(1, sizeof(matrix));
	scale_matrix = calloc(1, sizeof(matrix));
	
	// initialize shift vector
	shift_vector->v[0] = (-1.0)*(s->x);
	shift_vector->v[1] = (-1.0)*(s->y);
	shift_vector->v[2] = (-1.0)*(s->theta);
	
	//initialize clockwise_matrix
	
	/*
	clockwise_matrix->v[0][0] = (double)cos(s->theta);
	clockwise_matrix->v[0][1] = (double)sin(s->theta);
	clockwise_matrix->v[0][2] = 0.0;
	
	clockwise_matrix->v[1][0] = (double)( -1.0 * sin(s->theta) );
	clockwise_matrix->v[1][1] = (double) cos(s->theta);
	clockwise_matrix->v[1][2] = 0.0;
	*/
	//ccw test://seems better but translation is still off
	clockwise_matrix->v[0][0] = (double)cos(-1.0*s->theta);
	clockwise_matrix->v[0][1] = (double)sin(-1.0*s->theta);
	clockwise_matrix->v[0][2] = 0.0;
	
	clockwise_matrix->v[1][0] = (double)sin(-1.0*s->theta);
	clockwise_matrix->v[1][1] = (double)cos(-1.0*s->theta);
	clockwise_matrix->v[1][2] = 0.0;
	
	clockwise_matrix->v[2][0] = 0.0;
	clockwise_matrix->v[2][1] = 0.0;
	clockwise_matrix->v[2][2] = 1.0;
	
	//initialize scale_matrix
	scale_matrix->v[0][0] = 1.0/NS_TICKS_PER_CM;
	scale_matrix->v[0][1] = 0.0;
	scale_matrix->v[0][2] = 0.0;
	
	scale_matrix->v[1][0] = 0.0;
	scale_matrix->v[1][1] = 1.0/NS_TICKS_PER_CM;
	scale_matrix->v[1][2] = 0.0;
	
	scale_matrix->v[2][0] = 0.0;
	scale_matrix->v[2][1] = 0.0;
	scale_matrix->v[2][2] = 1.0;  //180.0 / M_PI to convert to degrees
}

/* use clockwise rotation matrix //(i think since initial theta represents ccw)
 * shift + --> rotate * --> scale */
vector *transform_NS(ns_stance *s){
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
	printf("Add result = ");
	PrintVector(&working_vector);//diagnostic
	
	//rotate
	MultMatVec(clockwise_matrix, &working_vector, &working_vector_2);
	printf("Rotate Result = ");
	PrintVector(&working_vector_2);//diagnostic
	
	//scale
	// Update Scaling Matrix based on current signal strength
#if 0
	if(s->sig > 13000) {
		scale_matrix->v[0][0] = 1/NS_TICKS_PER_CM;
		scale_matrix->v[1][1] = 1/NS_TICKS_PER_CM;
	}
	else {  // currently keeping scaling factor at 45 per cm when signal strength is high and scaling down to 60 per cm when signal strength is low
		  scale_matrix->v[0][0] = 1.0 / ( -0.001875 * s->sig + 69.375 );
		  scale_matrix->v[1][1] = 1.0 / ( -0.001875 * s->sig + 69.375 );
		  // possibly scale X and Y seperately?
	}
#endif
	
	MultMatVec(scale_matrix, &working_vector_2, ns_vector);
	printf("Scaling Result = ");
	PrintVector(ns_vector);//diagnostic
	return ns_vector;
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

void exit_ns(){
	free(shift_vector);
	free(clockwise_matrix);
	free(scale_matrix);  
}