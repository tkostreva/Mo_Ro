/*  Filename:	filter.c
 *  Author:	Tim Kostreva based on FIR filter code at
 * 		http://kujo.cs.pitt.edu/cs1567/index.php/Fir_code
 *  Date:	1/20/12
 *  Purpose:	A Fir_Filter is used to filter out the signal noises that are present in the room. The coefficients
                  were calculated from Matlab.  
 */

#include "filter.h"

/* GLOBALS */

/* Coefficients for both the deep and shallow filters */
float deep_coeffs[DEEP_TAPS] = { 0.022138, 0.090259, 0.16733, 0.22801, 0.22801, 0.16733, 0.090259, 0.022138 };
//float coeffs[TAPS] = { -0.031106, -0.011948, 0.16449, 0.3831, 0.3831, 0.16449, -0.011948, -0.031106 };
float shlw_coeffs[SHLW_TAPS] = { 0.22586, 0.3907, 0.3907, 0.22586 };

/* FUNCTION CODE */

/* Creates and initalizes new filter */
filter *fir_Filter_Create() {
	/* get memory for filter and initilize all values to zero */
	filter *f = (filter *) calloc(1, sizeof(filter));
	
	return f;
}

/* Returns a filtered value from filter f, with input val, and filter type depth */
float fir_Filter(filter *f, float val, int depth) {
	int i, j;		/* loop control variables */
	float sum = 0.0;	/* sum for weighted average */
	
	/* put value into into next slot */
	f->samples[f->next] = val;
	
	/* get weighted sum from a deep filter with val being most recent */
	if(depth == 1) {
		for( i = 0, j = f->next; i < DEEP_TAPS; i++ ) {
			  sum += f->samples[j] * deep_coeffs[i];
			  /* iterate to next sample, wrap to beginning if neccessary */
			  if ( ++j == DEEP_TAPS ) j = 0;
		}
	}
	/* get weighted sum from a shallow filter with val being most recent */
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

// Free memory from a instance of a filter struct
void free_filter(filter *f) {
    free(f);  
}