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

typedef struct _robot_stance_ {
	ns_stance *ns; 
	we_stance *we;
	int x;
	int y;
	float theta;	
} robot_stance;

// I think we need to define this based on matlab results... of the Filter Talked about on Wednesday 1/18
#define NORTHSTAR_UNRELIABLE_STRENGTH RI_ROBOT_NAV_SIGNAL_WEAK

robot_stance *create_stance();
float delta_theta();
void init_pos(robot_if_t *ri);		//sets up initial coordinates
int getX(robot_if_t *ri);		//returns right/left position relative to origin in cm
int getY(robot_if_t *ri);		//returns forward/reverse position relative to origin in cm
int getTheta(robot_if_t *ri);		//returns from 0 to 360
void resetCoordinates(robot_if_t *ri);	//sets origin to current position 
float getDistance(robot_if_t *ri);	//returns distance since last reset in cm
void print_stance_csv();
void exit_pos();			//clean up memory

#endif