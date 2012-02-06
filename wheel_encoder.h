#ifndef _wheel_encoder_
#define _wheel_encoder_

#include <robot_if.h>
#include <stdio.h>
#include <math.h>
#include "matvec.h"

#define WE_TICKS_PER_CM		3.3

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

vector *transform_WE(we_stance *s, float theta);

void print_we(we_stance *s);

void print_we_csv(we_stance *s);

#endif