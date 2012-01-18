#include "wheel_encoder.h"

// Populate Wheel Encoder Stance Object from sensor data
void get_we_delta(we_stance *s, robot_if_t *ri ) {
	s->right = ri_getWheelEncoder( ri, RI_WHEEL_RIGHT );
	s->left = ri_getWheelEncoder( ri, RI_WHEEL_LEFT );
	s->back = ri_getWheelEncoder( ri, RI_WHEEL_REAR );
}

void get_we_totals(we_stance *s, robot_if_t *ri ) {
	s->right = ri_getWheelEncoderTotals( ri, RI_WHEEL_RIGHT );
	s->left = ri_getWheelEncoderTotals( ri, RI_WHEEL_LEFT );
	s->back = ri_getWheelEncoderTotals( ri, RI_WHEEL_REAR );
}

void copy_we(we_stance *s, we_stance *r) {
	s->right = r->right;
	s->left = r->left;
	s->back = r->back;	 
}

float get_we_dist_FB(we_stance *current) {
	float avg;
	avg = current->right * sin(120.0 / 180.0 * M_PI);
	avg += current->left * sin(60.0 / 180.0 * M_PI);
	avg /= 2.0;
	
	return avg / WE_TICKS_TO_CM;
	
}

float get_we_dist_LR(we_stance *current) {
	float avg;
	avg = current->right * cos(120.0 / 180.0 * M_PI);
	avg += current->left * cos(60.0 / 180.0 * M_PI);
	avg += current->back;
	avg /= 3.0;
	
	return avg / WE_TICKS_TO_CM;
	
}

void print_we(we_stance *s) {
	printf("R = %d\tL = %d\tB = %d\n", s->right, s->left, s->back);
}