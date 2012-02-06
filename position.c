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

void get_filtered(robot_stance *s, robot_if_t *ri){
	s->ns_f->x		= (int)fir_Filter(f[0], (float)s->ns->x, DEEP_FILTER);
	s->ns_f->y		= (int)fir_Filter(f[1], (float)s->ns->y, DEEP_FILTER);
	s->ns_f->theta		= fir_Filter(f[2], s->ns->theta, DEEP_FILTER);
	s->ns_f->sig		= (int)fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	s->we_f->left_tot	= (int)fir_Filter(f[3], (float)s->we->left_tot, DEEP_FILTER);
	s->we_f->right_tot	= (int)fir_Filter(f[4], (float)s->we->right_tot, DEEP_FILTER);
	s->we_f->back_tot	= (int)fir_Filter(f[5], (float)s->we->back_tot, DEEP_FILTER);
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

// Create a Robot Stance structure and allocate memory for pointers
robot_stance *create_stance(){
	robot_stance *rs = (robot_stance *) calloc(1, sizeof(robot_stance));
	
	rs->ns = (ns_stance *) calloc(1, sizeof(ns_stance));
	rs->ns_f = (ns_stance *) calloc(1, sizeof(ns_stance));
	
	rs->we = (we_stance *) calloc(1, sizeof(we_stance));
	rs->we_f = (we_stance *) calloc(1, sizeof(we_stance));
	
	rs->nsTranslated = (vector *)malloc(sizeof(vector));
	rs->weTranslated = (vector *)malloc(sizeof(vector));
	
	return rs;
}

// populate stance with RAW northstar and wheel encoder data
void get_stance(robot_stance *s, robot_if_t *ri) {
	get_ns(s->ns, ri);
	get_we(s->we, ri);
	
	get_filtered(s, ri);	
}

void copy_stance(robot_stance *original, robot_stance *copy){
	// deep copy all filtered data
	copy->ns_f->x 		= original->ns_f->x;
	copy->ns_f->y 		= original->ns_f->y;
	copy->ns_f->theta	= original->ns_f->theta;
	copy->ns_f->sig		= original->ns_f->sig;
	copy->we_f->left_tot	= original->we_f->left_tot;
	copy->we_f->right_tot	= original->we_f->right_tot;
	copy->we_f->back_tot	= original->we_f->back_tot;
	
	// free copy nsTranslated and point to original's nsTranslated
	free(copy->nsTranslated);
	copy->nsTranslated = original->nsTranslated;
	
	// free copy weTranslated and point to original's weTranslated
	free(copy->weTranslated);
	copy->weTranslated = original->weTranslated;  
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
	
	// Get initial Northstar position
	update_sensor_data(ri);
		
	// Initialize and flush filters
	for(i = 0; i < NUM_FILTERS; i++) f[i] = fir_Filter_Create();
	filter_flush(ri);
	
	// Initialize all Robot Stances to current position
	initial = create_stance();
	get_stance(initial, ri);
	
	printf("Initial NS = ");
	print_ns(initial->ns);
	printf("Initial NS_F = ");
	print_ns(initial->ns_f);
	// Setup Northstar Transform matrices based on intial position
	setup_NS_transforms(initial->ns_f);
	
	current = create_stance();
	copy_stance(initial, current);
	
	previous = create_stance();
	copy_stance(initial, previous);
}

void get_Position(robot_if_t *ri, vector *loc){
	// copy current stance into previous
	copy_stance(current, previous);
	  
	update_sensor_data(ri);
	get_stance(current, ri);
	
	// Transforms occur here
	current->nsTranslated = transform_NS(current->ns_f);
	printf("NS Translation Result = ");
	PrintVector(current->nsTranslated);//diagnostic
	
	current->weTranslated = transform_WE(current->we);
	printf("WE Translation Result = ");
	PrintVector(current->weTranslated);//diagnostic
	
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
