/*
 * Filename: wheel_encoder.h
 * Authors: Tim Kostreva, Junchao Hua, Spencer Krause
 * Date: 02-24-2012
 */

#ifndef _wheel_encoder_
#define _wheel_encoder_

#include <robot_if.h>
#include <stdio.h>
#include "matvec.h"
//rosie
#define WE_TICKS_PER_CM		4.00
#define ROTATION_SCALING	4.00

// WE struct
typedef struct _we_stance_ {
	int left_tot;
	int left_delta;
	int right_tot;
	int right_delta;
	int back_tot;
	int back_delta;
} we_stance;


// Populate Wheel Encoder Stance Object from sensor data
void get_we(we_stance *s, robot_if_t *ri );

// setup WE transformation
void setup_WE_transforms(vector *v);

// Perform WE transformation
void transform_WE(we_stance *s, vector *ws);

void get_turning_theta(we_stance *s, vector *ws);

// set up a waypoint for wheel encoder at turn location 
void prepare_to_turn(robot_if_t *ri, vector *v);

// update the shift and rotation vectors for wheel encoders from waypoint
void finish_turn(robot_if_t *ri, vector *v);

void print_we(we_stance *s);

void print_we_csv(we_stance *s);

void exit_we();
#endif