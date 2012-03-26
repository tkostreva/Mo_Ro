/*  
 * Filename: position.c  
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause  
 * Date: 02-24-2012  
 * Purpose: Use filtered data and PID control to navigate the robot to a goto position.
 *        Abstract all wheel encoder and northstar transforms into position.c so that 
 * 	 robot_assign1 really only has to deal with our translated position.....
 * 	 so the main program only knows how far it's gone in x and y in centimeters and theta in degrees?
 *
 *	1/19/2012 
 * 	DO FILTERING AND TRANSFORMS HERE so we have access to the raw data as well as filtered data
 */

#include "position.h"
#include "kalmanFilterDef.h"

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
robot_stance *initial, *current, *previous, *last;
vector *room_switch;
kalmanFilter *kfilter;
//int kalman_count = 0;

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
//flush the fir filter
void filter_flush(robot_if_t *ri) {
	int i;
	
	for(i = 0; i < DEEP_TAPS - 1; i++) {
		fir_Filter(f[0], (float)ri_getX(ri), SHALLOW_FILTER);
		fir_Filter(f[1], (float)ri_getY(ri), SHALLOW_FILTER);
		fir_Filter(f[2], ri_getTheta(ri), SHALLOW_FILTER);
		fir_Filter(f[3], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_LEFT ), SHALLOW_FILTER);
		fir_Filter(f[4], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_RIGHT ), SHALLOW_FILTER);
		fir_Filter(f[5], (float)ri_getWheelEncoderTotals( ri, RI_WHEEL_REAR ), SHALLOW_FILTER);
		fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	
		update_sensor_data(ri);
	}
}
//get fir filtered data
void get_filtered(robot_stance *s, robot_if_t *ri){
	s->ns_f->x		= (int)fir_Filter(f[0], (float)s->ns->x, SHALLOW_FILTER);
	s->ns_f->y		= (int)fir_Filter(f[1], (float)s->ns->y, SHALLOW_FILTER);
	s->ns_f->theta		= s->ns->theta; //(fir_Filter(f[2], s->ns->theta, SHALLOW_FILTER) + s->ns->theta) / 2.0;
	s->ns_f->sig		= (int)fir_Filter(f[6], ri_getNavStrengthRaw(ri), DEEP_FILTER);
	s->ns_f->room		= s->ns->room;
	s->we_f->left_tot	= (int)fir_Filter(f[3], (float)s->we->left_tot, SHALLOW_FILTER);
	s->we_f->right_tot	= (int)fir_Filter(f[4], (float)s->we->right_tot, SHALLOW_FILTER);
	s->we_f->back_tot	= s->we->back_tot; //((int)fir_Filter(f[5], (float)s->we->back_tot, SHALLOW_FILTER) + s->we->back_tot) / 2;
}

// use this to report ACTUAL difference between last theta and current theta ( prevent wrap around )
float delta_theta(float current_theta, float previous_theta) {
	float d_theta;
	
	// find delta theta and get it's absolute value
	d_theta = current_theta - previous_theta;
	//if(d_theta < 0.0) d_theta *= -1.0;
	
	// check to see if reported theta wrapped around
	if(d_theta > M_PI) d_theta = 2.0 * M_PI - d_theta; // theta wrapped from -PI to PI
		
	if(d_theta < -M_PI) d_theta = -2.0 * M_PI - d_theta; // theta wrapped from PI to -PI	
		
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
	rs->weTranslated = (vector *)calloc(1, sizeof(vector));
	rs->kalmanFiltered = (vector *)malloc(sizeof(vector));
	
	return rs;
}

// populate stance with RAW northstar and wheel encoder data
void get_stance(robot_stance *s, robot_if_t *ri) {
	get_ns(s->ns, ri);
	get_we(s->we, ri);
	
	get_filtered(s, ri);	
}

//get kalman filtered data
void get_kalman_filter_data(vector *kf_data){
	float track[9];
	
	rovioKalmanFilter(kfilter, current->nsTranslated->v, current->weTranslated->v, track);
	kf_data->v[0] = track[0];
	kf_data->v[1] = track[1];
	kf_data->v[2] = track[2];
	
	//printf("Kalmann filtered result = %f\t%f\t%f\n", track[0],track[1],track[2]);	
}

void init_pos(robot_if_t *ri){
	int i;
	//float vel[3] = { 350.0/54.0, 0, 0 };
	float vel[3] = {35, 0, 0 };
	//float vel[3] = {0, 0, 10 };
	float pos[3] = {0, 0, 0};
	int deltaT = 1;
	
	// Get initial Northstar position
	update_sensor_data(ri);
		
	// Initialize and flush filters
	for(i = 0; i < NUM_FILTERS; i++) f[i] = fir_Filter_Create();
	filter_flush(ri);
	
	// Get Memory for all Robot Stances
	initial = create_stance();
	current = create_stance();
	previous = create_stance();
	last = create_stance();
	
	// Get Initial position
	get_stance(initial, ri);
	
	//Debugging
	//printf("Initial NS = ");
	//print_ns(initial->ns);
	//printf("Initial NS_F = ");
	//print_ns(initial->ns_f);
	
	// Setup Northstar and Wheel Encoder Transform matrices based on intial position
	setup_NS_transforms(initial->ns_f);
	setup_WE_transforms(initial->weTranslated);
	
	//printf("Initial Kalman Theta = %f \n", initial->kalmanFiltered->v[2]);
	
	// Copy Initial into current and previous to initialize them
	copy_stance(initial, current);
	copy_stance(initial, previous);
	copy_stance(initial, last);
	
	// Get memory for room_switch vector
	room_switch = (vector *)calloc(1, sizeof(vector));
	
	// allocate memory for kalmanfilter
	kfilter = (kalmanFilter *) calloc(1, sizeof(kalmanFilter));
	//track = (float *) malloc(9*sizeof(float));
	initKalmanFilter(kfilter, pos, vel, deltaT);	
}

/* Use this function when a specific move is completed to reset wheel encoders and set up NS transforms
 * with reference to current position as origin.  Absolute postion is stored in last and will be added to
 * future kalman filter data */
void update_pos(robot_if_t *ri){
	// Copy current into last	
	copy_stance(current, last);
	
	// Reset Wheel Encoders
	ri_reset_state(ri);
	
	// Flush filters
	filter_flush(ri);
	
	setup_NS_transforms(current->ns_f);
	setup_WE_transforms(current->kalmanFiltered);
	
	/* reset room switch if it was used */
	room_switch->v[0] = 0.0;
	room_switch->v[1] = 0.0;
	room_switch->v[2] = 0.0;
}
//when a room change occurs
void room_change(robot_if_t *ri){
	// flush fliters for new room
	filter_flush(ri);
	
	// update current with new filter data
	get_stance(current, ri);
	
	printf("\n\n---------------------Room change-------------------\n\n");
	printf("current room = %d, previous room = %d\n ", current->ns->room, previous->ns->room);
	printf("previous room position: %d, %d, %f, %d\n ", previous->ns_f->x, previous->ns_f->y, previous->ns_f->theta, previous->ns_f->sig);
	printf("current room : %d, %d, %f, %d\n", current->ns_f->x, current->ns_f->y, current->ns_f->theta, current->ns_f->sig);
		
	setup_NS_transforms(current->ns_f);//use untranslated, filtered ns stance
	
	room_switch->v[0] = previous->nsTranslated->v[0];
	room_switch->v[1] = previous->nsTranslated->v[1];
	room_switch->v[2] = previous->nsTranslated->v[2];
}

int get_Position(robot_if_t *ri, vector *loc, vector *vel, int m_t){
	static int lastmove;
	float d_theta;
	int room_changed = 0;
	IplImage *image = NULL;
	
	image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	
	// copy current stance into previous
	copy_stance(current, previous);
	
	update_sensor_data(ri);
	get_stance(current, ri);
	
	// Get the current camera image
	if(ri_get_image(ri, image) != RI_RESP_SUCCESS) {
		printf("Unable to capture an image!\n");		
	}
	
	// Display the image with the drawing oon ito
	cvShowImage("Square Display", image);
		
	// Update the UI (10ms wait)
	cvWaitKey(10);
	
	// check for room change
	if (current->ns->room != previous->ns->room) {
	  room_change(ri);
	  room_changed = 1;
	}
	/* check to see if move type is changing from forward to rotate
	 * to prepare Wheel Encoders to Rotate */
	if( lastmove == FORWARD && m_t == ROTATE ) {
		prepare_to_turn(ri, previous->weTranslated);
	}
	
	/* check to see if move type is changing from rotate to forward
	 * to prepare wheel encoder shifts and rotation transforms */
	if( lastmove == ROTATE && m_t == FORWARD ) {
		finish_turn(ri, previous->weTranslated);
	}
	
	// Prevent NS Wrap Around  NO NORTHSTAR THETA FILTERING //
	//printf("Theta_curr = %f\tTheta_prev = %f\t", current->ns->theta, previous->ns->theta);
	d_theta = delta_theta(current->ns->theta, previous->ns->theta);
	//printf("Delta theta NS = %f\n", d_theta);
	current->ns_f->theta = d_theta + previous->ns_f->theta;
	
	// Transforms occur here
	print_ns(current->ns_f);
	transform_NS(current->ns_f, current->nsTranslated);
	if(m_t == ROTATE) get_turning_theta(current->we, current->weTranslated);
	else transform_WE(current->we, current->weTranslated);

	// Shift nsTranslated with room switch vector and last kalman
	current->nsTranslated->v[0] += room_switch->v[0] + last->kalmanFiltered->v[0];
	current->nsTranslated->v[1] += room_switch->v[1] + last->kalmanFiltered->v[1];
	current->nsTranslated->v[2] += room_switch->v[2] + last->kalmanFiltered->v[2];
	
	// Shift weTranslated with last kalman
	current->weTranslated->v[0] += last->kalmanFiltered->v[0];
	current->weTranslated->v[1] += last->kalmanFiltered->v[1];
	current->weTranslated->v[2] += last->kalmanFiltered->v[2];
	
	//diagnostic
	printf("NS Translation Result = ");
	PrintVector(current->nsTranslated);
	
	//diagnostic
	printf("WE Translation Result = ");
	PrintVector(current->weTranslated);

	//  Old average of both transforms
	/*loc_wo_kalman[0] = ( current->nsTranslated->v[0] + current->weTranslated->v[0] ) / 2.0;
	loc_wo_kalman[1] = ( current->nsTranslated->v[1] + current->weTranslated->v[1] ) / 2.0;
	loc_wo_kalman[2] = current->nsTranslated->v[2];
	printf("Location without Kalmann results = %f\t%f\t%f\n", loc_wo_kalman[0],loc_wo_kalman[1],loc_wo_kalman[2]);
	*/
	//get kalman filtered data
	rovioKalmanFilterSetVelocity(kfilter, vel->v); 
	get_kalman_filter_data(current->kalmanFiltered);
	
	// Report kalman filtered values
	loc->v[0] = current->kalmanFiltered->v[0];
	loc->v[1] = current->kalmanFiltered->v[1];
	loc->v[2] = current->kalmanFiltered->v[2];
	
	lastmove = m_t; // track last state of move type
	
	free(image);
	
	return room_changed;
}

/* Function for gathering Northstar data and returning it in vector u */
int NS_theta_cal(robot_if_t *ri, vector *u){
	update_sensor_data(ri);
	get_stance(current, ri);

	u->v[0] = (float)current->ns_f->x;
	u->v[1] = (float)current->ns_f->y;
	u->v[2] = current->ns_f->theta;

	return current->ns->room;
}

// Deep copy of robot stance object
void copy_stance(robot_stance *original, robot_stance *copy){
	// deep copy room id data
	copy->ns->theta		= original->ns->theta;
	copy->ns->room 		= original->ns->room;
	copy->ns_f->room 	= original->ns_f->room;
	
	// deep copy all filtered data
	copy->ns_f->x 		= original->ns_f->x;
	copy->ns_f->y 		= original->ns_f->y;
	copy->ns_f->theta	= original->ns_f->theta;
	copy->ns_f->sig		= original->ns_f->sig;
	copy->we_f->left_tot	= original->we_f->left_tot;
	copy->we_f->right_tot	= original->we_f->right_tot;
	copy->we_f->back_tot	= original->we_f->back_tot;
	
	// deep copy nsTranslated
	copy->nsTranslated->v[0] = original->nsTranslated->v[0];
	copy->nsTranslated->v[1] = original->nsTranslated->v[1];
	copy->nsTranslated->v[2] = original->nsTranslated->v[2];
	
	// deep copy weTranslated
	copy->weTranslated->v[0] = original->weTranslated->v[0];
	copy->weTranslated->v[1] = original->weTranslated->v[1];
	copy->weTranslated->v[2] = original->weTranslated->v[2];
	
	// deep copy kalman filtered
	copy->kalmanFiltered->v[0] = original->kalmanFiltered->v[0];
	copy->kalmanFiltered->v[1] = original->kalmanFiltered->v[1];
	copy->kalmanFiltered->v[2] = original->kalmanFiltered->v[2];
}

void print_stance_csv(){
	static char init = 0;
	
	// print out header for CSV file on first pass
	if ( init == 0 ) {
	        printf("L_tot, L_dlt, R_tot, R_dlt, B_tot, B_dlt, L_F, R_F, B_F, NS_X, NS_Y, NS_T,"); 
	        printf(" Sig, Room, NS_X_F, NS_Y_F, NS_T_F, NS_Sig_F, NS_xfm_X, NS_xfm_Y, NS_xfm_T,");
	        printf(" WE_xfm_X, WE_xfm_Y, WE_xfm_T\n");
	        init = 1;
	}
	
	print_we_csv(current->we);
	printf(", %d, %d, %d, ", current->we_f->left_tot, current->we_f->right_tot, current->we_f->back_tot);
	print_ns_csv(current->ns);
	printf(", %d, %d, %f, %d, ", current->ns_f->x, current->ns_f->y, current->ns_f->theta, current->ns_f->sig);
	
	printf("%f, %f, %f, %f, %f, %f\n", current->nsTranslated->v[0], current->nsTranslated->v[1],
		current->nsTranslated->v[2], current->weTranslated->v[0], current->weTranslated->v[1],
		current->weTranslated->v[2]);
}

void free_stance(robot_stance *s){
	free(s->we);
	free(s->we_f);
	free(s->ns);
	free(s->ns_f);
	free(s);
}

void exit_pos(){
	free_stance(initial);
	free_stance(current);
	free_stance(previous);
	
	free(room_switch);
	
	free(kfilter);
	
	exit_ns();
	exit_we();
}
