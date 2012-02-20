/* Definitions for the Kalman Redundant Sensors C Implementation */
#ifndef __KRS_H__
#define __KRS_H__

#define INIT_VAL	0.05

#define diag(x, val)	{\
	for(i=0; i<FILTER_SIZE; i++) {\
		x[i*FILTER_SIZE + i] = val;\
	}\
}

#define pmat(x) {\
	printf("===============================================================\n");\
	for(i=0; i<FILTER_SIZE; i++) {\
		for(j=0; j<FILTER_SIZE; j++) {\
			printf("%f ", x[i * FILTER_SIZE + j]);\
		}\
		printf("\n");\
	}\
}

#endif /* __KRS_H__ */
