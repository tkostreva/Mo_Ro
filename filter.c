#include "filter.h"

float deep_coeffs[DEEP_TAPS] = { 0.022138, 0.090259, 0.16733, 0.22801, 0.22801, 0.16733, 0.090259, 0.022138 };
//float coeffs[TAPS] = { -0.031106, -0.011948, 0.16449, 0.3831, 0.3831, 0.16449, -0.011948, -0.031106 };
float shlw_coeffs[SHLW_TAPS] = { 0.22586, 0.3907, 0.3907, 0.22586 };

/* Creates and initalizes new filter */
filter *fir_Filter_Create() {
	/* get memory for filter and initilize all values to zero */
	filter *f = (filter *) calloc(1, sizeof(filter));
	
	return f;
}

float fir_Filter(filter *f, float val, int depth) {
	int i, j;
	float sum = 0.0;
	
	/* put value into into next slot */
	f->samples[f->next] = val;
	
	/* get weighted sum with val being most recent */
	if(depth == 1) {
		for( i = 0, j = f->next; i < DEEP_TAPS; i++ ) {
			  sum += f->samples[j] * deep_coeffs[i];
			  /* iterate to next sample, wrap to beginning if neccessary */
			  if ( ++j == DEEP_TAPS ) j = 0;
		}
	}
	else {
		for( i = 0, j = f->next; i < SHLW_TAPS; i++ ) {
			  sum += f->samples[j] * shlw_coeffs[i];
			  /* iterate to next sample, wrap to beginning if neccessary */
			  if ( ++j == DEEP_TAPS ) j = 0;
		}
	}
	
	/* set up filter for next sample */
	f->next++;
	if(f->next == DEEP_TAPS) f->next = 0;
	
	return sum;
}

void free_filter(filter *f) {
    free(f);  
}