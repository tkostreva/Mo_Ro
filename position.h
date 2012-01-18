#ifndef _position_
#define _position_
#include "northstar.h"
#include "wheel_encoder.h"

#define NORTHSTAR_UNRELIABLE_STRENGTH RI_ROBOT_NAV_SIGNAL_WEAK

/*
Where am I?
    Utilizes position data from the north-star and wheel encoders to give you an idea of the robot’s position.
*/
int init(robot_if_t *ri);//sets up initial coordinates
int getX(robot_if_t *ri);//returns right/left position relative to origin in cm
int getY(robot_if_t *ri);//returns forward/reverse position relative to origin in cm
int getTheta(robot_if_t *ri);//returns from 0 to 360
void resetCoordinates(robot_if_t *ri);//sets origin to current position 
int getDistance(robot_if_t *ri);//returns distance since last reset in cm

#endif