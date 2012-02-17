#include "wheel_encoder.h"

vector *we_shift_vector;
matrix *we_rot_matrix;

// Populate Wheel Encoder Stance Object from sensor data
void get_we(we_stance *s, robot_if_t *ri ) {
	s->left_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_LEFT );
	s->left_delta = ri_getWheelEncoder( ri, RI_WHEEL_LEFT );
	s->right_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_RIGHT );
	s->right_delta = ri_getWheelEncoder( ri, RI_WHEEL_RIGHT );
	s->back_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_REAR );
	s->back_delta = ri_getWheelEncoder( ri, RI_WHEEL_REAR );
}

void setup_WE_transforms(vector *v){
	// free pointers in case previously declared
	free(we_rot_matrix);
  
	// get memory for pointers
	we_shift_vector = (vector *)malloc(sizeof(vector));
	we_rot_matrix = (matrix *)malloc(sizeof(matrix));
		
	// store v as the shift vector
	we_shift_vector->v[0] = v->v[0];
	we_shift_vector->v[1] = v->v[1];
	we_shift_vector->v[2] = v->v[2];
	
	//initialize we_rot_matrix
	we_rot_matrix->v[0][0] = cos(v->v[2]);
	we_rot_matrix->v[0][1] = sin(v->v[2]);
	we_rot_matrix->v[0][2] = 0.0;
	
	we_rot_matrix->v[1][0] = -1.0 * sin(v->v[2]);
	we_rot_matrix->v[1][1] = cos(v->v[2]);
	we_rot_matrix->v[1][2] = 0.0;
	
	we_rot_matrix->v[2][0] = 0.0;
	we_rot_matrix->v[2][1] = 0.0;
	we_rot_matrix->v[2][2] = 1.0;
}

// Currently Reports FRONT/BACK distance from TOTALS
float get_we_X(we_stance *s) {
	float avg;
	//print_we(s);
	avg = (float)s->right_delta * sin(60.0 / 180.0 * M_PI);
	avg += (float)s->left_delta * sin(120.0 / 180.0 * M_PI);
	avg /= 2.0;
	avg /= WE_TICKS_PER_CM;
	//printf("Avg = %f\n", avg);
	
	return avg;
}

// Currently Reports LEFT/RIGHT distance from TOTALS
float get_we_Y(we_stance *s) {
	float avg;
	//printf("r_delta = %d\tl_delta = %d\n", s->right_tot, s->left_tot);
	avg = (float)s->right_delta * cos(60.0 / 180.0 * M_PI);
	avg += (float)s->left_delta * cos(120.0 / 180.0 * M_PI);
	avg /= 2.0;
	
	avg /= WE_TICKS_PER_CM;
	
	return avg;
}

// Report Theta in radians for total change of back WE since last update
float get_we_Theta(we_stance *s) {
	float theta;
	
	/* get transformed difference between wheel encoders */
	/*theta = (float)s->right_tot * sin(60.0 / 180.0 * M_PI);
	theta -= (float)s->left_tot * sin(120.0 / 180.0 * M_PI);
	theta /= WE_TICKS_PER_CM;
	*/
	theta = s->back_tot / -48.0;
	
	//theta = s->back_tot / WE_TICKS_PER_CM;
	
	//theta /= -(5.0 * M_PI);
  
	return theta;
}

float get_turning_theta(we_stance *s) {
	float theta;
	printf("Back Total = %d\n", s->back_tot);
	
	theta = s->back_tot / -48.0;
	
	//theta = s->back_tot / WE_TICKS_PER_CM;
	
	//theta /= -(5.0 * M_PI);
  
	return theta;
}

/* set up a waypoint for wheel encoder at turn location */
void prepare_to_turn(robot_if_t *ri, vector *v){
	// store v as the shift vector
	we_shift_vector->v[0] = v->v[0];
	we_shift_vector->v[1] = v->v[1];
	we_shift_vector->v[2] = v->v[2];
	
	/* reset we totals */
	ri_reset_state(ri);
}

/* WE NEED A TRANSFORM WE SPECIFICALLY FOR TURNING !!!! */

/* update the shift and rotation vectors for wheel encoders from waypoint */
void finish_turn(robot_if_t *ri, vector *v) {
	we_rot_matrix->v[0][0] = cos(v->v[2]);
	we_rot_matrix->v[0][1] = sin(v->v[2]);
		
	we_rot_matrix->v[1][0] = -1.0 * sin(v->v[2]);
	we_rot_matrix->v[1][1] = cos(v->v[2]);
	
	/* update shift vector with kalman data????*/
	
	ri_reset_state(ri);
}

void transform_WE(we_stance *s, vector *ws){
	float theta,
	      update_t,
	      cos_t,
	      sin_t;
	vector 	working_vector,
		result;    

	theta = get_we_Theta(s);
	update_t = theta + we_shift_vector->v[2];
	cos_t = cos(update_t);
	sin_t = sin(update_t);
	
	/* update rotation matrix based on bots current wheel encoder theta */
	we_rot_matrix->v[0][0] = cos_t;
	we_rot_matrix->v[0][1] = sin_t;
	we_rot_matrix->v[1][0] = -1.0 * sin_t;
	we_rot_matrix->v[1][1] = cos_t;
	
	/* get wheel encoder reported distances in prep for rotation */
	working_vector.v[0] = get_we_X(s);
	working_vector.v[1] = get_we_Y(s);
	working_vector.v[2] = theta;
	
	/* rotate current vector */
	MultMatVec(we_rot_matrix, &working_vector, &result);
	
	/* add to shift vector */
	AddVectors(we_shift_vector, &result, ws);
	
	/* update shift vector with current ws as running total */
	we_shift_vector->v[0] = ws->v[0];
	we_shift_vector->v[1] = ws->v[1];
	we_shift_vector->v[2] = ws->v[2];	
}

void print_we(we_stance *s) {
	printf("L_tot = %d\tL_dlt = %d\tR_tot = %d\tR_dlt = %d\tB_tot = %d\tB_dlt = %d\n", 
	       s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

void print_we_csv(we_stance *s) {
      printf("%d, %d, %d, %d, %d, %d",s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

void exit_we(){
      free(we_rot_matrix);
      free(we_shift_vector);
}