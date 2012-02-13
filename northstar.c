/*
 * 	Now only reports raw NorthStar Data....  filtering will be done in position.c
 */

#include "northstar.h"

vector *shift_vector;
matrix *rotation_matrix;
matrix *scale_matrix;

float theta_cor[] = {
	-0.559143, /*ROOM TWO THETA COR*/
	 0.136328, /*ROOM THREE THETA COR*/
	-1.545036, /*ROOM FOUR THETA COR*/
	 0.052755  /*ROOM FIVE THETA COR*/
};

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri ) {
	s->x = ri_getX(ri);
	s->y = ri_getY(ri);
	s->theta = ri_getTheta(ri);
	s->sig =  ri_getNavStrengthRaw(ri);
	s->room = ri_getRoomID(ri);
	
}
//updates ns transform vecotrs to be accurate after a room change
void update_NS_transforms_after_room_change(ns_stance *before, ns_stance *after){
	vector working_vector; //temp
	vector room_shift_vector; //temp will be added to shift_vector
	float rot_theta;
	int i;//counter
	
	room_shift_vector.v[0] = (float)((before->x)-(after->x));
	room_shift_vector.v[1] = (float)((before->y)-(after->y));
	room_shift_vector.v[2] = (before->theta)-(after->theta);
	
	//add room_shift_vector to shift vector
	AddVectors(shift_vector, &room_shift_vector, &working_vector);//did i do the pointers right here?
	
	for(i=0; i<3; i++){//copy results
		shift_vector->v[i] = working_vector.v[i];
	}
		
	rot_theta = theta_cor[after->room - 2];//is this the best way to do this? might be better to use initial ns theta from each room to calirate on the fly
	
	//overwrite rotation_matrix
	rotation_matrix->v[0][0] = cos(rot_theta);
	rotation_matrix->v[0][1] = -1.0 * sin(rot_theta);
	rotation_matrix->v[0][2] = 0.0;
	
	rotation_matrix->v[1][0] = sin(rot_theta);
	rotation_matrix->v[1][1] = cos(rot_theta);
	rotation_matrix->v[1][2] = 0.0;
	
	rotation_matrix->v[2][0] = 0.0;
	rotation_matrix->v[2][1] = 0.0;
	rotation_matrix->v[2][2] = 1.0;
}

// Setup the transformation matrices with values stored in NS stance s
void setup_NS_transforms(ns_stance *s) {
	float rot_theta = theta_cor[s->room - 2];
  
	// free pointers in case previously declared
	free(shift_vector);
	free(rotation_matrix);
	free(scale_matrix);
  
	// get memory for pointers
	shift_vector = calloc(1, sizeof(vector));
	rotation_matrix = calloc(1, sizeof(matrix));
	scale_matrix = calloc(1, sizeof(matrix));
	
	//diagnostic
	//print_ns(s);
	
	// initialize shift vector
	shift_vector->v[0] = (-1.0)*((float)s->x);
	shift_vector->v[1] = (-1.0)*((float)s->y);
	shift_vector->v[2] = (-1.0)*(s->theta);
	//diagnostic
	//printf("Shift Vector is ");
	//PrintVector(shift_vector);
	
	//initialize rotation_matrix
	rotation_matrix->v[0][0] = cos(rot_theta);
	rotation_matrix->v[0][1] = -1.0 * sin(rot_theta);
	rotation_matrix->v[0][2] = 0.0;
	
	rotation_matrix->v[1][0] = sin(rot_theta);
	rotation_matrix->v[1][1] = cos(rot_theta);
	rotation_matrix->v[1][2] = 0.0;
	
	rotation_matrix->v[2][0] = 0.0;
	rotation_matrix->v[2][1] = 0.0;
	rotation_matrix->v[2][2] = 1.0;
	
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
	AddVectors(ns, shift_vector, &working_vector);
	//diagnostic
	//printf("Add result = ");
	//PrintVector(&working_vector);
	
	//rotate
	MultMatVec(rotation_matrix, &working_vector, &working_vector_2);
	//diagnostic
	//printf("Rotate Result = ");
	//PrintVector(&working_vector_2);
	
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
	
	MultMatVec(scale_matrix, &working_vector_2, ns);
	//diagnostic
	//printf("Scaling Result = ");
	//PrintVector(ns);
	//free(&working_vector);
	//free(&working_vector_2);
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
	free(rotation_matrix);
	free(scale_matrix);  
}