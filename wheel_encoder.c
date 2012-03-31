/*
 * Filename: wheel_encoder.c
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause
 * Date: 02-24-2012
 * Purpose:
 *	Been trying to update the wheel encoder section to use WE deltas instead of totals.  The totals are messing us up.
 *	Our old transform kept looking at the total distance traveled by the wheel encoders straigh ahead and then rotating it
 *  	from the START POINT, which gave us very very very inaccurate wheel encoder results if we were anywhere off the straight ahead
 *	"X Axis."  I am trying to make it so that wheel encoders keep a running total of vectors between each sensor update to track
 *	a more accurate wheel encoder path.  I am also trying to migrate turning into it's own fuction so that turns to do not royally screw
 *	the wheel encoder results (get to turn -> update the shift vector -> reset the wheel encoder totals -> turn -> go from that point)
 */

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
	
	// compute the weighted average of X for the left and right wheels
	avg = (float)s->right_delta * sin(60.0 / 180.0 * M_PI);
	avg += (float)s->left_delta * sin(120.0 / 180.0 * M_PI);
	avg /= 2.0;
	avg /= WE_TICKS_PER_CM;
	
	return avg;
}

// Currently Reports LEFT/RIGHT distance from TOTALS
float get_we_Y(we_stance *s) {
	float avg;
	
	// compute the weighted average of Y for the left and right wheels
	avg = (float)s->right_delta * cos(60.0 / 180.0 * M_PI);
	avg += (float)s->left_delta * cos(120.0 / 180.0 * M_PI);
	avg /= 2.0;
	
	avg /= (WE_TICKS_PER_CM * 4.0);
	
	return avg;
}

// Using difference between front wheel encoders to track the deviation from straight-ish theta for use in "go to" portion
float get_we_Theta(we_stance *s) {
	float 	theta;	
	
	theta = ( (float)s->back_tot / ROTATION_SCALING ) / 14.5;
	
	return theta * -1.0;
}

// return a transformed WE vector for use in the Kalman filter 
void transform_WE(we_stance *s, vector *ws){
	float theta,
	      update_t,
	      cos_t,
	      sin_t;
	vector 	working_vector,
		result;    

	// compute the cos and sin of the update wheel encoder theta
	theta = get_we_Theta(s);
	update_t = theta + we_shift_vector->v[2];
	cos_t = cos(update_t);
	sin_t = sin(update_t);
	
	// update rotation matrix based on bots current wheel encoder theta
	we_rot_matrix->v[0][0] = cos_t;
	we_rot_matrix->v[0][1] = -1.0 * sin_t;
	we_rot_matrix->v[1][0] = sin_t;
	we_rot_matrix->v[1][1] = cos_t;
	
	// get wheel encoder reported distances in prep for rotation
	working_vector.v[0] = get_we_X(s);
	working_vector.v[1] = get_we_Y(s);
	working_vector.v[2] = theta;
	
	// rotate current vector 
	MultMatVec(we_rot_matrix, &working_vector, &result);
	
	// add result to shift vector, put results in reporting vector ws 
	AddVectors(we_shift_vector, &result, ws);
	
	// update shift vector with current ws as running total
	we_shift_vector->v[0] = ws->v[0];
	we_shift_vector->v[1] = ws->v[1];
	we_shift_vector->v[2] = ws->v[2];	
}

// set up a waypoint for wheel encoder at turn location 
void prepare_to_turn(robot_if_t *ri, vector *v){
	// store v as the shift vector
	we_shift_vector->v[0] = v->v[0];
	we_shift_vector->v[1] = v->v[1];
	we_shift_vector->v[2] = v->v[2];
	
	// reset we totals 
	ri_reset_state(ri);
}

// Using totals from all three wheel encoders ONLY when we are ordering a "turn to" 
// Left and right WE are 13.5 cm from center of rotation, back WE is 15 cm from center 
void get_turning_theta(we_stance *s, vector *ws) {
	float	l_theta,  /* left, right, and back thetas */
		r_theta,
		b_theta,
		avg_theta;
	
	// theta a wheel travels through is the distance traveled along outside diameter [in cm] / radius in cm
	// Original Measurements:  Front wheels radius 13.5  Rear Wheel 15.0
	l_theta = ( (float)s->left_tot / ROTATION_SCALING ) / 13.5;
	r_theta = ( (float)s->right_tot / ROTATION_SCALING ) / 13.5;
	b_theta = ( (float)s->back_tot / ROTATION_SCALING ) / 15.0;
	
	// for right rotation, left WE increases, right WE decreases, back WE increases;  opposite for left rotation
	// following formula properly sums thetas */
	avg_theta = 0.3 * l_theta - 0.3 * r_theta + 0.4 * b_theta;
	
	// make avg_theta conform to our coordinate system
	avg_theta *= -1.0;
  
	//set ws vector values 
	ws->v[0] = we_shift_vector->v[0];
	ws->v[1] = we_shift_vector->v[1];
	ws->v[2] = we_shift_vector->v[2] + avg_theta;
}

// update the shift and rotation vectors for wheel encoders from waypoint
void finish_turn(robot_if_t *ri, vector *v) {
	we_rot_matrix->v[0][0] = cos(v->v[2]);
	we_rot_matrix->v[0][1] = -1.0 * sin(v->v[2]);
		
	we_rot_matrix->v[1][0] = sin(v->v[2]);
	we_rot_matrix->v[1][1] = cos(v->v[2]);
	
	/* update shift vector with final value from turn */
	we_shift_vector->v[0] = v->v[0];
	we_shift_vector->v[1] = v->v[1];
	we_shift_vector->v[2] = v->v[2];
	
	ri_reset_state(ri);
}

// print WE data
void print_we(we_stance *s) {
	printf("L_tot = %d\tL_dlt = %d\tR_tot = %d\tR_dlt = %d\tB_tot = %d\tB_dlt = %d\n", 
	       s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

//output WE data in .csv format
void print_we_csv(we_stance *s) {
      printf("%d, %d, %d, %d, %d, %d",s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

// free pointers
void exit_we(){
      free(we_rot_matrix);
      free(we_shift_vector);
}