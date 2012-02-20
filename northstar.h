#ifndef _northstar_
#define _northstar_

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matvec.h"

#define NS_SCALING_FACTOR	90000.0
#define NS_TICKS_PER_CM		45.0

typedef struct _ns_stance_ {
	int x;
	int y;
	float theta;
	int sig;
	int room;	
} ns_stance;

// Populate NorthStar Stance Object from sensor data
void get_ns(ns_stance *s, robot_if_t *ri );

void setup_NS_transforms(ns_stance *s);

void update_NS_transforms_after_room_change(ns_stance *before, ns_stance *after);

void transform_NS(ns_stance *s, vector *ns);

void print_ns(ns_stance *s);

void print_ns_csv(ns_stance *s);

void exit_ns();

#endif