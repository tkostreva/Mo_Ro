#ifndef _northstar_
#define _northstar_

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NS_SCALING_FACTOR	90000.0
#define NS_TICKS_TO_CM 		90.0

typedef struct _ns_stance_ {
	int x;
	int y;
	float theta;
	int sig;
	int room;	
} ns_stance;

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri );

float get_ns_dist(ns_stance *current, ns_stance *last);

// get euclidean distance between two points ( point 2 - point 1 )
float euclidean_distance(float x1, float y1, float x2, float y2);

void print_ns(ns_stance *s);

void print_ns_csv(ns_stance *s);

int ns_ticksToCM(int ticks);

#endif