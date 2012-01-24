#include "wheel_encoder.h"

// Populate Wheel Encoder Stance Object from sensor data
void get_we(we_stance *s, robot_if_t *ri ) {
	s->left_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_LEFT );
	s->left_delta = ri_getWheelEncoder( ri, RI_WHEEL_LEFT );
	s->right_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_RIGHT );
	s->right_delta = ri_getWheelEncoder( ri, RI_WHEEL_RIGHT );
	s->back_tot = ri_getWheelEncoderTotals( ri, RI_WHEEL_REAR );
	s->back_delta = ri_getWheelEncoder( ri, RI_WHEEL_REAR );
}

// Currently Reports FRONT/BACK distance from TOTALS
float get_we_dist_FB(we_stance *s) {
	float avg;
	avg = s->right_tot * sin(120.0 / 180.0 * M_PI);
	avg += s->left_tot * sin(60.0 / 180.0 * M_PI);
	avg /= 2.0;
	
	return avg / WE_TICKS_TO_CM;
	
}

// Currently Reports LEFT/RIGHT distance from TOTALS
float get_we_dist_LR(we_stance *s) {
	float avg;
	avg = s->right_tot * cos(120.0 / 180.0 * M_PI);
	avg += s->left_tot * cos(60.0 / 180.0 * M_PI);
	avg += s->back_tot;
	avg /= 3.0;
	
	return avg / WE_TICKS_TO_CM;
	
}

void print_we(we_stance *s) {
	printf("L_tot = %d\tL_dlt = %d\tR_tot = %d\tR_dlt = %d\tB_tot = %d\tB_dlt = %d\n", 
	       s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

void print_we_csv(we_stance *s) {
      printf("%d, %d, %d, %d, %d, %d",s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}