#ifndef _filter_
#define _filter_

#include <stdlib.h>

/* filter settings */
#define DEEP_TAPS 8	/* defines the size of coeffs array */
#define SHLW_TAPS 4

enum filter_type {
	SHALLOW_FILTER,
	DEEP_FILTER
};

/* define filter */
typedef struct _filter_ {
	unsigned next;
	float samples[DEEP_TAPS];
} filter;

/* Creates and initalizes new filter */
filter *fir_Filter_Create();

float fir_Filter(filter *f, float val, int deep);

void free_filter(filter *f);
#endif