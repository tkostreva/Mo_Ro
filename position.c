/*
 * 	Abstract all wheel encoder and northstar transforms into position.c so that 
 * 	robot_assign1 really only has to deal with our translated position.....
 * 	so the main program only knows how far it's gone in x and y in centimeters and theta in degrees?
 *
 *	1/19/2012 
 * 	DO FILTERING AND TRANSFORMS HERE so we have access to the raw data as well as filtered data
 */

#include "position.h"

#define NUM_FILTERS 2
/* f[0]  NorthStar X filter
 * f[1]  NorthStar Y filter 
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
	
	local->we = (we_stance *) calloc(1, sizeof(we_stance));
	
	return local;
}

void get_stance(robot_stance *s, robot_if_t *ri) {
	// populate stance with northstar and wheel encoder data
	get_ns(s->ns, ri);
	get_we(s->we, ri);
	
	// filtering and Transforms occur here??
	// s->x = something;
	// s->y = something;
	// s->theta = something;
}

void free_stance(robot_stance *s){
	free(s->we);
	free(s->ns);
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
	
	// Initialize all NorthStar Stances to current position
	initial = create_stance();
	get_stance(initial, ri);
	
	current = create_stance();
	get_stance(current, ri);
	
	previous = create_stance();
	get_stance(previous, ri);	
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
		printf("L_tot, L_dlt, R_tot, R_dlt, B_tot, B_dlt, NS_X, NS_Y, NS_T, Sig, Room\n");
		init = 1;
	}
	
	print_we_csv(current->we);
	print_ns_csv(current->ns);
	printf("\n");
	//printf("%d, %d, %f\n", current->x, current->y, current->theta);
}

//returns distance since last reset in cm
float get_Distance(robot_if_t *ri){
	get_stance(previous, ri);
	
	update_sensor_data(ri);
	get_stance(current, ri);
	
	return get_we_dist_FB(current->we);
}

void exit_pos(){
	free_stance(initial);
	free_stance(current);
	free_stance(previous);  
}