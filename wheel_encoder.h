#ifndef _wheel_encoder_
#define _wheel_encoder_

#include <robot_if.h>
#include <stdio.h>

#define WE_TICKS_TO_CM		3.3

typedef struct _we_stance_ {
	int right;
	int left;
	int back;
} we_stance;


// Populate Wheel Encoder Stance Object from sensor data
void get_we_delta(we_stance *s, robot_if_t *ri );

void get_we_totals(we_stance *s, robot_if_t *ri );

void copy_we(we_stance *s, we_stance *r);

float get_we_dist_FB(we_stance *current);

float get_we_dist_LR(we_stance *current);

void print_we(we_stance *s);

#endif