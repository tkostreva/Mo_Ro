/*
 * Filename: northstar.h
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause
 * Date: 02-24-2012
 * Purpose: Now only reports raw NorthStar Data....  filtering will be done in position.c 
*/

#ifndef _northstar_
#define _northstar_

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matvec.h"

#define NS_TICKS_PER_CM		60.0

// ns_stance structure contains x and y coordinates of NorthStar. Also it consists
// of theta, signal strength, and room ID
typedef struct _ns_stance_ {
	int x;
	int y;
	float theta;
	int sig;
	int room;	
} ns_stance;

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri );

// Setup the transformation matrices with values stored in NS stance s
void setup_NS_transforms(ns_stance *s);

// Perform northstar transformation 
void transform_NS(ns_stance *s, vector *ns);

// print out northstar data
void print_ns(ns_stance *s);

// export northstar data into a .csv file
void print_ns_csv(ns_stance *s);

void exit_ns();

#endif