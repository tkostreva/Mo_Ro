/*
 * 	Now only reports raw NorthStar Data....  filtering will be done in position.c
 */

#include "northstar.h"

vector *ns_shift_vector;
matrix *ns_rot_matrix;
matrix *scale_matrix;

float theta_cor[] = {
	-0.45, /*ROOM TWO THETA COR*/
	-1.456696, /*ROOM THREE THETA COR*/
	 0.035983, /*ROOM FOUR THETA COR*/
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
	/*float newX, newY, newT;
	
	//initialize current ns_vector
	ns->v[0] = (s->x);
	ns->v[1] = (s->y);
	ns->v[2] = (s->theta);
	//PrintVector(ns);//diagnostic
	
	
	//non- matrix transforms:
	//rotate:
	newX = s->v[0]*cos(rot_theta) − s->v[1]*sin(rot_theta);
	newY = s->v[1]*cos(rot_theta) + s->v[0]*sin(rot_theta);
	//shift:
	newX += ns_shift_vector[0];
	newY += ns_shift_vector[1];
	newT += ns_shift_vector[2];
	//scale:
	newX *= (1/52);
	newY *= (1/52);
	//move:
	ns->v[0]=newX;
	ns->v[1]=newY;
	ns->v[2]=newT;
	*/
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

	if(s->sig > 0) {  // currently always skips dynamic scaling
		scale_matrix->v[0][0] = 1/NS_TICKS_PER_CM;
		scale_matrix->v[1][1] = 1/NS_TICKS_PER_CM;
	}
	else {  // currently keeping scaling factor at 45 per cm when signal strength is high and scaling down to 60 per cm when signal strength is low
		  scale_matrix->v[0][0] = 1.0 / ( 0.001875 * s->sig + 16.0742 );
		  scale_matrix->v[1][1] = 1.0 / ( -0.0048 * s->sig + 84.6228 );
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