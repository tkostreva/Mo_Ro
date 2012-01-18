#ifndef _northstar_
#define _northstar_

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "filter.h"

#define NS_SCALING_FACTOR	90000.0
#define NS_TICKS_TO_CM 		90.0

typedef struct _ns_stance_ {
	int x;
	int y;
	float theta;
	int sig;
	int room;	
} ns_stance;

/* North star filter holds filters for each coordinates reported */
typedef struct _ns_filter_{
	filter *x_filter;
	filter *y_filter;
	filter *t_filter;
	filter *s_filter;
} ns_filter;

float euclidean_distance(float x1, float y1, float x2, float y2);

void exit_ns();

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri );

// copy stance R into stance S
void copy_ns(ns_stance *s, ns_stance *r);

float get_ns_dist(ns_stance *current, ns_stance *last);

// get euclidean distance between two points ( point 2 - point 1 )
float euclidean_distance(float x1, float y1, float x2, float y2);

void print_ns(ns_stance *s);

void print_ns_csv(ns_stance *s);

int ns_ticksToCM(int ticks);
#endif