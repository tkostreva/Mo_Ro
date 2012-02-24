/*
 * Filename: position.h
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause
 * Date: 02-24-2012
 * Purpose:	
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

// Set DATA_COLLECT to 1 supress normal output and dump data for csv files
#define DATA_COLLECT 0

enum move_type {
	FORWARD,
	ROTATE
};

typedef struct _robot_stance_ {
	ns_stance *ns;		// Raw North Star Data 
	ns_stance *ns_f;	// Filtered North Star Data
	we_stance *we;		// Raw Wheel Encoder Data
	we_stance *we_f;	// Filtered Wheel Encoder Data
	vector *nsTranslated;	// North Star Transformed position
	vector *weTranslated;	// Wheel Encoder Transformed position
	vector *kalmanFiltered; // Kalman Filtered Data
} robot_stance;


// create robot stances
robot_stance *create_stance();

// obtain kalman filtered data
void get_kalman_filter_data(vector *kf_data);

//use this to report ACTUAL difference between last theta and current theta
float delta_theta(float current_theta, float previous_theta);

//sets up initial coordinates
void init_pos(robot_if_t *ri);		

/* Get Position of bot, stored in vector loc, feed kalman filter expected velocities, reports a room change */
int get_Position(robot_if_t *ri, vector *loc, vector *vel, int m_t);

// Function for gathering Northstar data and returning it in vector u 
int NS_theta_cal(robot_if_t *ri, vector *u);

// Deep copy of robot stance object
void copy_stance(robot_stance *original, robot_stance *copy);

void print_stance_csv();

//clean up memory
void exit_pos();			

#endif
