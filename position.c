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
robot_stance *initial, *current, *previous;

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

// populate stance with RAW northstar and wheel encoder data
void get_stance(robot_stance *s, robot_if_t *ri) {
	get_ns(s->ns, ri);
	get_we(s->we, ri);	
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
	initial = create_stance();
	get_stance(initial, ri);
	
	// Setup Northstar Transform matrices based on intial position
	setup_NS_transforms(initial->ns_f);
	
	current = create_stance();
	get_stance(current, ri);
	
	previous = create_stance();
	get_stance(previous, ri);	
}

void get_Position(robot_if_t *ri, vector *loc){
	// populate previous with raw data values
	get_stance(previous, ri);
	
	// deep copy all filtered data
	previous->ns_f->x 		= current->ns_f->x;
	previous->ns_f->y 		= current->ns_f->y;
	previous->ns_f->theta		= current->ns_f->theta;
	previous->ns_f->sig		= current->ns_f->sig;
	previous->we_f->left_tot	= current->we_f->left_tot;
	previous->we_f->right_tot	= current->we_f->right_tot;
	previous->we_f->back_tot	= current->we_f->back_tot;
	
	// free old nsTranslated and point to current's nsTranslated
	free(previous->nsTranslated);
	previous->nsTranslated = current->nsTranslated;
	
	// free old weTranslated and point to current's weTranslated
	free(previous->weTranslated);
	previous->weTranslated = current->weTranslated;
  
	update_sensor_data(ri);
	get_stance(current, ri);
	
	// get filtered data for current position
	current->ns_f->x	= (int)fir_Filter(f[0], (float)current->ns->x, DEEP_FILTER);
	current->ns_f->y	= (int)fir_Filter(f[1], (float)current->ns->y, DEEP_FILTER);
	current->ns_f->theta	= fir_Filter(f[2], current->ns->theta, DEEP_FILTER);
	current->ns_f->sig	= (int)fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	current->we_f->left_tot	= (int)fir_Filter(f[3], (float)current->we->left_tot, DEEP_FILTER);
	current->we_f->right_tot	= (int)fir_Filter(f[4], (float)current->we->right_tot, DEEP_FILTER);
	current->we_f->back_tot	= (int)fir_Filter(f[5], (float)current->we->back_tot, DEEP_FILTER);
	
	// Transforms occur here
	free(current->nsTranslated);
	current->nsTranslated = transform_NS(current->ns_f);
	
	free(current->weTranslated);
	current->weTranslated = transform_WE(current->we);
	
	loc->v[0] = ( current->nsTranslated->v[0] + current->weTranslated->v[0] ) / 2.0;
	loc->v[1] = ( current->nsTranslated->v[1] + current->weTranslated->v[1] ) / 2.0;
	loc->v[2] = current->nsTranslated->v[2];	
}

void print_stance_csv(){
	static char init = 0;
	
	// print out header for CSV file on first pass
	if ( init == 0 ) {
		printf("L_tot, L_dlt, R_tot, R_dlt, B_tot, B_dlt, L_F, R_F, B_F, NS_X, NS_Y, NS_T, Sig, Room, NS_X_F, NS_Y_F, NS_T_F, NS_Sig_F, NS_xfm_X, NS_xfm_Y\n");
		init = 1;
	}
	
	print_we_csv(current->we);
	printf(", %d, %d, %d, ", current->we_f->left_tot, current->we_f->right_tot, current->we_f->back_tot);
	print_ns_csv(current->ns);
	printf(", %d, %d, %f, %d", current->ns_f->x, current->ns_f->y, current->ns_f->theta, current->ns_f->sig);
	
	printf("%d, %d\n", current->nsTranslated->v[0], current->nsTranslated->v[1]);
}

void exit_pos(){
	free_stance(initial);
	free_stance(current);
	free_stance(previous);
	
	exit_ns();
}
