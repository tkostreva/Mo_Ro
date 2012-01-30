/*
 * 	Abstract all wheel encoder and northstar transforms into position.c so that 
 * 	robot_assign1 really only has to deal with our translated position.....
 * 	so the main program only knows how far it's gone in x and y in centimeters and theta in degrees?
 *
 *	1/19/2012 
 * 	DO FILTERING AND TRANSFORMS HERE so we have access to the raw data as well as filtered data
 */

#include "position.h"

// We are storing all raw data, as well as filtering it
#define NUM_FILTERS 7
#define NS_TICKS_PER_CM 45
/* f[0]  NorthStar X filter
 * f[1]  NorthStar Y filter
 * f[2]  NorthStar Theta filter
 * f[3]  Left WE filter
 * f[4]  Right WE filter
 * f[5]  Back WE filter
 * f[6]  NorthStar Raw Signal Filter
 */

/* GLOBALS TO POSITION.C */
filter *f[NUM_FILTERS];
robot_stance *current, *previous;

// Update the robot's sensor information
void update_sensor_data( robot_if_t *ri ) {
	// If first sensor update fails to respond, try one more time before giving up
	if(ri_update(ri) != RI_RESP_SUCCESS) {
		if(ri_update(ri) != RI_RESP_SUCCESS) {
			printf("Failed to update sensor information!\n");
			exit(10);
		}
        }
}

void filter_flush(robot_if_t *ri) {
	int i;
	
	for(i = 0; i < DEEP_TAPS - 1; i++) {
		fir_Filter(f[0], (float)ri_getX(ri), DEEP_FILTER);
		fir_Filter(f[1], (float)ri_getY(ri), DEEP_FILTER);
		fir_Filter(f[2], ri_getTheta(ri), DEEP_FILTER);
		fir_Filter(f[3], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_LEFT ), DEEP_FILTER);
		fir_Filter(f[4], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_RIGHT ), DEEP_FILTER);
		fir_Filter(f[5], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_REAR ), DEEP_FILTER);
		fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	
		update_sensor_data(ri);
	}
}

// use this to report ACTUAL difference between last theta and current theta ( prevent wrap around )
float delta_theta() {
	int sign;
	float d_theta;
	
	// find delta theta and get it's absolute value
	d_theta = current->ns->theta - previous->ns->theta;
	if(d_theta < 0.0) d_theta *= -1.0;
	
	// check to see if reported theta wrapped around
	if(d_theta > M_PI){
		// get sign of last theta to check direction of wrap
		if(  previous->ns->theta < 0 ) sign = -1;
		else sign = 1;
		
		// correct change in theta by 2 * PI
		d_theta = sign * ( d_theta - ( 2.0 * M_PI )); 
	}
	
	return d_theta;	
}

robot_stance *create_stance(){
	robot_stance *local = (robot_stance *) calloc(1, sizeof(robot_stance));
	
	local->ns = (ns_stance *) calloc(1, sizeof(ns_stance));
	local->ns_f = (ns_stance *) calloc(1, sizeof(ns_stance));
	
	local->we = (we_stance *) calloc(1, sizeof(we_stance));
	local->we_f = (we_stance *) calloc(1, sizeof(we_stance));	
	
	return local;
}

void get_stance(robot_stance *s, robot_if_t *ri) {
	// populate stance with northstar and wheel encoder data
	get_ns(s->ns, ri);
	get_we(s->we, ri);
	
	// filter data
	s->ns_f->x	= (int)fir_Filter(f[0], (float)s->ns->x, DEEP_FILTER);
	s->ns_f->y	= (int)fir_Filter(f[1], (float)s->ns->y, DEEP_FILTER);
	s->ns_f->theta	= fir_Filter(f[2], s->ns->theta, DEEP_FILTER);
	s->ns_f->sig	= (int)fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	s->we_f->left_tot	= (int)fir_Filter(f[3], (float)s->we->left_tot, DEEP_FILTER);
	s->we_f->right_tot	= (int)fir_Filter(f[4], (float)s->we->right_tot, DEEP_FILTER);
	s->we_f->back_tot	= (int)fir_Filter(f[5], (float)s->we->back_tot, DEEP_FILTER);
	
	// Transforms occur here??
	// s->x = something;
	// s->y = something;
	// s->theta = something;
}

void free_stance(robot_stance *s){
	free(s->we);
	free(s->we_f);
	free(s->ns);
	free(s->ns_f);
	free(s);
}

void init_pos(robot_if_t *ri){
	int i;
	
	// Get initial Northstar position and reset wheel encoder totals
	update_sensor_data(ri);
	ri_reset_state(ri);
	
	// Initialize and flush filters
	for(i = 0; i < NUM_FILTERS; i++) f[i] = fir_Filter_Create();
	filter_flush(ri);
	
	// Initialize all Robot Stances to current position
	current = create_stance();
	get_stance(current, ri);
	
	previous = create_stance();
	get_stance(previous, ri);	
}

void transformNS(robot_stance* current_stance, robot_stance* initial_stance){//in progress
	
	//use clockwise rotation matrix //(i think since initial theta represents ccw)
	//shift + --> rotate * --> scale *
	
	//initialize shift_vector
	vector shift_vector = calloc(1, sizeof(vector));
	shift_vector->v[0] = (-1)*(initial->ns_f->x);
	shift_vector->v[1] = (-1)*(initial->ns_f->y);
	shift_vector->v[2] = (-1)*(initial->ns_f->theta);
	
	//initialize clockwise_matrix
	matrix clockwise_matrix = calloc(1, sizeof(matrix));
	clockwise_matrix->v[0][0] = (float)cos((double)(initial->ns_f->theta));
	clockwise_matrix->v[0][1] = (float)sin((double)(initial->ns_f->theta));
	clockwise_matrix->v[0][2] = 0;
	
	clockwise_matrix->v[1][0] = (float)((-1)*sin((double)(initial->ns_f->theta)));
	clockwise_matrix->v[1][1] = (float)cos((double)(initial->ns_f->theta));
	clockwise_matrix->v[1][2] = 0;
	
	clockwise_matrix->v[2][0] = 0;
	clockwise_matrix->v[2][1] = 0;
	clockwise_matrix->v[2][2] = 1;
	
	//initialize scale_matrix
	matrix scale_matrix = calloc(1, sizeof(matrix));
	scale_matrix->v[0][0] = (float)(1/NS_TICKS_PER_CM);
	scale_matrix->v[0][1] = 0;
	scale_matrix->v[0][2] = 0;
	
	scale_matrix->v[1][0] = 0;
	scale_matrix->v[1][1] = (float)(1/NS_TICKS_PER_CM);
	scale_matrix->v[1][2] = 0;
	
	scale_matrix->v[2][0] = 0;
	scale_matrix->v[2][1] = 0;
	scale_matrix->v[2][2] = 1;//convert to degrees? do it here if needed
	
	//initialize currentns_vector
	vector currentns_vector = calloc(1, sizeof(vector));
	currentns_vector->v[0] = (current->ns_f->x);
	currentns_vector->v[1] = (current->ns_f->y);
	currentns_vector->v[2] = (current->ns_f->theta);
	
	//initialize working_vector //temp storage
	vector working_vector = calloc(1, sizeof(vector));
	
	//do some math and store the results in currentstance x y and theta
	//shift + --> rotate * --> scale *
	
}

int getX(robot_if_t *ri){
  
  /*
   * Should we do Northstar weighted average with wheel encoder based on signal strength?
   */
  if(ri_getNavStrength(ri)>NORTHSTAR_UNRELIABLE_STRENGTH){
    //use northstar
  }
  else{
    //use encoders
  }
  return 0;
}

int getY(robot_if_t *ri){
  if(ri_getNavStrength(ri)>NORTHSTAR_UNRELIABLE_STRENGTH){
    //use northstar
  }
  else{
    //use encoders
  }
  return 0;
}

int getTheta(robot_if_t *ri){
  if(ri_getNavStrength(ri)>NORTHSTAR_UNRELIABLE_STRENGTH){
    //use northstar
  }
  else{
    //use encoders
  }
  return 0;
}
void resetCoordinates(robot_if_t *ri){
  
  return;
}

void print_stance_csv(){
	static char init = 0;
	
	// print out header for CSV file on first pass
	if ( init == 0 ) {
		printf("L_tot, L_dlt, R_tot, R_dlt, B_tot, B_dlt, L_F, R_F, B_F, NS_X, NS_Y, NS_T, Sig, Room, NS_X_F, NS_Y_F, NS_T_F, NS_Sig_F\n");
		init = 1;
	}
	
	print_we_csv(current->we);
	printf(", %d, %d, %d, ", current->we_f->left_tot, current->we_f->right_tot, current->we_f->back_tot);
	print_ns_csv(current->ns);
	printf(", %d, %d, %f, %d\n", current->ns_f->x, current->ns_f->y, current->ns_f->theta, current->ns_f->sig);
	
	//printf("%d, %d, %f\n", current->x, current->y, current->theta);
}

//returns distance since last reset in cm
void get_Distance(robot_if_t *ri, float *dist){
	get_stance(previous, ri);
	
	update_sensor_data(ri);
	get_stance(current, ri);
	
	*dist = get_we_dist_FB(current->we);
}

void exit_pos(){
	free_stance(initial);
	free_stance(current);
	free_stance(previous);  
}