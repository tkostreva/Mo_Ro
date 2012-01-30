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
<<<<<<< HEAD
#include "transform.h"
#include "matvec.h"

typedef struct _position_vector_ {
	int x;
	int y;
	float theta;
} posVec;

typedef struct _robot_stance_ {
	ns_stance *ns;		// Raw North Star Data 
	ns_stance *ns_f;	// Filtered North Star Data
	we_stance *we;		// Raw Wheel Encoder Data
	we_stance *we_f;	// Filtered Wheel Encoder Data
	posVec nsXfmd;		// North Star Transformed position
	posVec weXfmd;		// Wheel Encoder Transformed position
=======
#include "matvec.h"

typedef struct _robot_stance_ {
	ns_stance *ns;//raw
	ns_stance *ns_f;//raw
	we_stance *we;//raw
	we_stance *we_f;//raw
	int x, nsX, weX;//translated 
	int y, nsY, weY;//translated
	float theta;//translated
>>>>>>> 40137aace2ab5546467f420e97586f6c9ee3a707
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
void transformNS(robot_stance* current_stance, robot_stance* initial_stance); //transform ns stance values and store in stance x and y and theta

#endif