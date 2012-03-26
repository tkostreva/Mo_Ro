/* Filename:  robot_vision.h
 * Author:	Based on robot_camera_example.c from API
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
