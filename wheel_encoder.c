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

// Currently Reports LEFT/RIGHT distance from TOTALS
float get_we_X(we_stance *s) {
	float avg;
	avg = s->right_tot * cos(120.0 / 180.0 * M_PI);
	avg += s->left_tot * cos(60.0 / 180.0 * M_PI);
	avg += s->back_tot;
	avg /= 3.0;
	
	return avg / WE_TICKS_PER_CM;	
}

// Currently Reports FRONT/BACK distance from TOTALS
float get_we_Y(we_stance *s) {
	float avg;
	//print_we(s);
	avg = s->right_tot * sin(120.0 / 180.0 * M_PI);
	avg += s->left_tot * sin(60.0 / 180.0 * M_PI);
	avg /= 2.0;
	avg /= WE_TICKS_PER_CM;
	//printf("Avg = %f\n", avg);
	
	return avg;
}

// Currently Reports Theta in radians for change of back WE since last
float get_we_Theta(we_stance *s) {
	float temp;
	
	temp = s->back_delta / WE_TICKS_PER_CM;
	
	temp /= (M_PI * 29.0);
  
	return temp;
}

vector *transform_WE(we_stance *s, float theta){
	vector we_vector = calloc(1, sizeof(vector));
	float forwardMotion = TICKS_PER_CM*((float)(s->left_tot+s->right_tot))/2.0;
	
	we_vector->v[0] = -1.0*sin(theta)*forwardMotion;//x
	we_vector->v[1] = cos(theta)*forwardMotion;//y
	we_vector->v[2] = theta + get_we_Theta(s);//theta
	
	return &we_vector;
}

void print_we(we_stance *s) {
	printf("L_tot = %d\tL_dlt = %d\tR_tot = %d\tR_dlt = %d\tB_tot = %d\tB_dlt = %d\n", 
	       s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}

void print_we_csv(we_stance *s) {
      printf("%d, %d, %d, %d, %d, %d",s->left_tot, s->left_delta, s->right_tot, s->right_delta, s->back_tot, s->back_delta);
}