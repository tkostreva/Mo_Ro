/*
 * 	Abstract all wheel encoder and northstar transforms into position.c so that 
 * 	robot_assign1 really only has to deal with our translated position.....
 * 	so the main program only knows how far it's gone in x and y in centimeters and theta in degrees?
 */

#include <robot_if.h>
#include <stdio.h>
#include "position.h"

int init(robot_if_t *ri){
  return 0;
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

//returns distance since last reset in cm
int getDistance(robot_if_t *ri){
	return 0;
}