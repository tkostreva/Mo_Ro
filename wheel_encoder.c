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
float get_we_X(we_stance *s) {
	float avg;
	//print_we(s);
	avg = s->right_tot * sin(120.0 / 180.0 * M_PI);
	avg += s->left_tot * sin(60.0 / 180.0 * M_PI);
	avg /= 2.0;
	avg /= WE_TICKS_PER_CM;
	//printf("Avg = %f\n", avg);
	
	return avg;
}

// Currently Reports LEFT/RIGHT distance from TOTALS
float get_we_Y(we_stance *s) {
	float avg;
	avg = s->right_tot * cos(120.0 / 180.0 * M_PI);
	avg += s->left_tot * cos(60.0 / 180.0 * M_PI);
	avg += s->back_tot;
	avg /= 3.0;
	
	return avg / WE_TICKS_PER_CM;	
}

// Currently Reports Theta in radians for change of back WE since last
float get_we_Theta(we_stance *s) {
	float theta;
	printf("Back Total = %d\n", s->back_tot);
	theta = s->back_tot / WE_TICKS_PER_CM;
	
	theta /= -(8.0 * M_PI);
  
	return theta;
}

void transform_WE(we_stance *s, vector *ws){
	float theta = get_we_Theta(s);
  
	ws->v[0] = sin(theta) * get_we_X(s);
	ws->v[1] = cos(theta) * get_we_Y(s);
	ws->v[2] = theta;
}

void print_we(we_stance *s) {
	printf("L_tot = %d\tL_dlt = %d\tR_tot = %d\tR_dlt = %d\tB_tot = %d\tB_dlt = %d\n", 
	       s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

void print_we_csv(we_stance *s) {
      printf("%d, %d, %d, %d, %d, %d",s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}