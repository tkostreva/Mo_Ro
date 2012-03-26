/* Colors used by OpenCV */

#ifndef __RC_COLORS_H__
#define __RC_COLORS_H__

#define RC_LOW(x)	cvScalar(x - 5, 75, 75, 0)
#define RC_HIGH(x) 	cvScalar(x + 5, 255, 255, 0)

/* Pink */
#define RC_PINK		5
#define RC_PINK_LOW_1	cvScalar(0, 67, 100, 0)	//i got better results with v = 65 rather than 47
#define RC_PINK_HIGH_1	cvScalar(13, 255, 255, 0)

#define RC_PINK_LOW_2	cvScalar(150, 67, 100, 0)
#define RC_PINK_HIGH_2	cvScalar(190, 255, 255, 0)

/* Yellow */
#define RC_YELLOW	30
#define RC_YELLOW_LOW	RC_LOW(RC_YELLOW)
#define RC_YELLOW_HIGH	RC_HIGH(RC_YELLOW)

/* Blue */
#define RC_BLUE		100
#define RC_BLUE_LOW	RC_LOW(RC_BLUE)
#define RC_BLUE_HIGH	RC_HIGH(RC_BLUE)

/* Green */
#define RC_GREEN	50
#define RC_GREEN_LOW	RC_LOW(RC_GREEN)
#define RC_GREEN_HIGH	RC_HIGH(RC_GREEN)

/* Purple */		
#define RC_PURPLE	140
#define RC_PURPLE_LOW	RC_LOW(RC_PURPLE)
#define RC_PURPLE_HIGH	RC_HIGH(RC_PURPLE)

#endif /* __RC_COLORS_H__ */
