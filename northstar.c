/*  
* Filename: northstar.c  
* Authors: Tim Kostreva, Junchao Hua, Spencer Krause  
* Date: 02-24-2012  
* Purpose: Now only reports raw NorthStar Data....  filtering will be done in position.c
*/

#include "northstar.h"

vector *ns_shift_vector;
matrix *ns_rot_matrix;
matrix *scale_matrix;

float theta_cor[] = {
	-0.25, /*ROOM TWO THETA COR*/
	-1.556130, /*ROOM THREE THETA COR*/
	-0.020485, /*ROOM FOUR THETA COR*/
	-1.502438  /*ROOM FIVE THETA COR*/
};

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri ) {
	s->x = ri_getX(ri);
	s->y = ri_getY(ri);
	s->theta = ri_getTheta(ri);
	s->sig =  ri_getNavStrengthRaw(ri);
	s->room = ri_getRoomID(ri);	
}

float rot_theta;
// Setup the transformation matrices with values stored in NS stance s
void setup_NS_transforms(ns_stance *s) {
	rot_theta = theta_cor[(s->room - 2)];  // rotation theta is defined by theta correctection matrix, values defined during calibration
	
	// free pointers in case previously declared
	free(ns_shift_vector);
	free(ns_rot_matrix);
	free(scale_matrix);
  
	// get memory for pointers
	ns_shift_vector = calloc(1, sizeof(vector));
	ns_rot_matrix = calloc(1, sizeof(matrix));
	scale_matrix = calloc(1, sizeof(matrix));
	
	//diagnostic
	//print_ns(s);	
	
	// initialize shift vector
	ns_shift_vector->v[0] = (-1.0)*((float)s->x);
	ns_shift_vector->v[1] = (-1.0)*((float)s->y);
	ns_shift_vector->v[2] = (-1.0)*(s->theta);
	//diagnostic
	//printf("Shift Vector is ");
	//PrintVector(ns_shift_vector);
	
	//initialize ns_rot_matrix
	ns_rot_matrix->v[0][0] = cos(rot_theta);
	ns_rot_matrix->v[0][1] = sin(rot_theta);
	ns_rot_matrix->v[0][2] = 0.0;
	
	ns_rot_matrix->v[1][0] = -1.0 * sin(rot_theta);
	ns_rot_matrix->v[1][1] = cos(rot_theta);
	ns_rot_matrix->v[1][2] = 0.0;
	
	ns_rot_matrix->v[2][0] = 0.0;
	ns_rot_matrix->v[2][1] = 0.0;
	ns_rot_matrix->v[2][2] = 1.0;
	
	//printf("Rotation Matrix = \n");
	//PrintMatrix( ns_rot_matrix );
	
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

/* Take current data in s, make it the intial inputs for the transformed vector ns
 * return the updated values in NS to calling function */
void transform_NS(ns_stance *s, vector *ns){
	vector working_vector; 
	vector working_vector_2;
		
	//initialize current ns_vector
	ns->v[0] = (s->x);
	ns->v[1] = (s->y);
	ns->v[2] = (s->theta);
	//PrintVector(ns);//diagnostic
	
	//shift
	AddVectors(ns, ns_shift_vector, &working_vector);
	//diagnostic
	//printf("Shift result = ");
	//PrintVector(&working_vector);
	
	//rotate
	MultMatVec(ns_rot_matrix, &working_vector, &working_vector_2);
	//diagnostic
	//printf("Rotate Result = ");
	//PrintVector(&working_vector_2);
	
	//scale
	// Update Scaling Matrix based on current signal strength [NOT WORKING RIGHT]

	if(s->sig > 15000) {
		scale_matrix->v[0][0] = 1.0 / (NS_TICKS_PER_CM - 15);
		scale_matrix->v[1][1] = scale_matrix->v[0][0];
	}
	else if(s->sig > 4000) {  
		  scale_matrix->v[0][0] = 1.0 / ( ((-3.0 * s->sig) / 1100.0) + 86.0 );
		  scale_matrix->v[1][1] = scale_matrix->v[0][0];
	}
	else {
		scale_matrix->v[0][0] = 1.0 / (NS_TICKS_PER_CM + 15);
		scale_matrix->v[1][1] = scale_matrix->v[0][0];
	}
	
	MultMatVec(scale_matrix, &working_vector_2, ns);
	//diagnostic
	//printf("Scaling Result = ");
	//PrintVector(ns);	
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
	free(ns_shift_vector);
	free(ns_rot_matrix);
	free(scale_matrix);  
}