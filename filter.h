/*  Filename:	filter.h
 *  Author:	Tim Kostreva based on FIR filter code at
 * 		http://kujo.cs.pitt.edu/cs1567/index.php/Fir_code
 *  Date:	1/20/12
 *  Purpose:	Make available a deep and shallow FIR filter for any output.
 */

#ifndef _filter_
#define _filter_

#include <stdlib.h>

/* DEFINES */
/* filter settings */
#define DEEP_TAPS 8	/* defines the size of coeffs array */
#define SHLW_TAPS 4

/* Enumarated type for deep and shallow filters */
enum filter_type {
	SHALLOW_FILTER,
	DEEP_FILTER
};

/* Struct for a filter for a particular signal.  
 * Stores N samples and a pointer to where next sample should go.*/
typedef struct _filter_ {
	unsigned next;
	float samples[DEEP_TAPS];
} filter;

/* FUNCTION DECLARATIONS */

/* Creates and initalizes new filter */
filter *fir_Filter_Create();

/* Returns a filtered value from filter f, with input val, and filter type depth */
float fir_Filter(filter *f, float val, int deep);

/* Free memory from a instance of a filter struct
void free_filter(filter *f);
#endif