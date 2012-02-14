/*	
 *	Where am I according to tracked coordinates?
 *	Utilizes position data from the north-star and wheel encoders to give you an idea of the robot’s position.
 * 	Performs neccessary filtering using filter.h
 */

#ifndef _position_
#define _position_

#include <robot_if.h>
#include <stdio.h>
#include "northstar.h"
#include "wheel_encoder.h"
#include "filter.h"
#include "matvec.h"

typedef struct _robot_stance_ {
	ns_stance *ns;		// Raw North Star Data 
	ns_stance *ns_f;	// Filtered North Star Data
	we_stance *we;		// Raw Wheel Encoder Data
	we_stance *we_f;	// Filtered Wheel Encoder Data
	vector *nsTranslated;	// North Star Transformed position
	vector *weTranslated;	// Wheel Encoder Transformed position
	vector *kalmanFiltered; // Kalman Filtered Data
} robot_stance;

// I think we need to define this based on matlab results... of the Filter Talked about on Wednesday 1/18
#define NORTHSTAR_UNRELIABLE_STRENGTH RI_ROBOT_NAV_SIGNAL_WEAK

robot_stance *create_stance();
void get_kalman_filter_data(vector *kf_data);

float delta_theta();

void init_pos(robot_if_t *ri);		//sets up initial coordinates

void resetCoordinates(robot_if_t *ri);	//sets origin to current position 

void get_Position(robot_if_t *ri, vector *loc);

void room_change_check(robot_if_t *ri);

int NS_theta_cal(robot_if_t *ri, vector *u);

void print_stance_csv();

void exit_pos();			//clean up memory

#endif
