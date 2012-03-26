/*  
 * Filename: robot_vision.c  
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause  
 * Date: 03-20-2012  
 * Purpose: robot_vision uses image processing to center the robot at its waypoints. As of right now, robot_vision
 *          only detects pink squares
 */

#ifndef __ROBOT_VISION_H__
#define __ROBOT_VISION_H__

#define MIN_AREA	1150
#define MAX_AREA	1300

#include "robot_if.h"
#include "robot_color.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int center(robot_if_t *ri);

#endif /* __ROBOT_VISION_H__ */
