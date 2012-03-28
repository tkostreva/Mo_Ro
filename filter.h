/*  Filename:	filter.h
 *  Author:	Tim Kostreva based on FIR filter code at
 * 		http://kujo.cs.pitt.edu/cs1567/index.php/Fir_code
 *  Date:	1/20/12
 *  Purpose:	Make available a FIR filter for any output.
 */

#ifndef _filter_
#define _filter_

#include <stdlib.h>

/* DEFINES */
/* filter settings */
#define TAPS 4	/* defines the size of coeffs array */

/* Struct for a filter for a particular signal.  
 * Stores N samples and a pointer to where next sample should go.*/
typedef struct _filter_ {
	unsigned next;
	float samples[TAPS];
} filter;

/* FUNCTION DECLARATIONS */

/* Creates and initalizes new filter */
filter *fir_Filter_Create();

/* Returns a filtered value from filter f, with input val, and filter type depth */
float fir_Filter(filter *f, float val);

/* Free memory from a instance of a filter struct */
void free_filter(filter *f);
#endif
