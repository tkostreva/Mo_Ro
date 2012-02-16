#ifndef _wheel_encoder_
#define _wheel_encoder_

#include <robot_if.h>
#include <stdio.h>
#include "matvec.h"
//rosie
#define WE_TICKS_PER_CM		3.615

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

void setup_WE_transforms(vector *v);

void transform_WE(we_stance *s, vector *ws);

void prepare_to_turn(robot_if_t *ri, vector *v);

void finish_turn(robot_if_t *ri, vector *v);

void print_we(we_stance *s);

void print_we_csv(we_stance *s);

void exit_we();
#endif