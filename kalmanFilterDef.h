#ifndef __KALMAN_FILTER_DEFS_H__
#define __KALMAN_FILTER_DEFS_H__

// the filter Size is defined by the number of elements in the filter state
// in this case, there are 9 elements, 3 vectors (xyz) for position, velocity and acceleration
#define FILTER_SIZE 9  // *** DONT CHANGE THIS *****
// the filter works by computing an optimally weighted sum of three values to generated a new 9-element
// state vector for the robot (called the prediction)
// 1 - a directly computed state based on a linear mechanical model
// 2 - the difference between the last prediction and the current Northstar Measurement
// 3 - the difference between the last prediction and the current Encoder Readings
// 
//
// part of the weight computation in each step is based on the amount of uncertainty associated with
// each of the three values
// **** you are free to adjust these values as you like ***** 
#define NORTHSTAR_UNCERTAINTY_X  .05 // this is the uncertainty of the northstar readings, we are providing
#define NORTHSTAR_UNCERTAINTY_Y  .05 // you with the capability to independently set the x,y,and theta
#define NORTHSTAR_UNCERTAINTY_TH .05

#define WHEELENC_UNCERTAINTY_X  .05 // this is the uncertainty of the wheel encoder readings, 
#define WHEELENC_UNCERTAINTY_Y  .05 // 
#define WHEELENC_UNCERTAINTY_TH .05

#define PROCESS_UNCERTAINTY_X  .05 // this is the uncertainty about whether the robot will obey the model
#define PROCESS_UNCERTAINTY_Y  .05
#define PROCESS_UNCERTAINTY_TH .05

// the following structure defines the important matrices and constants that make up the filter
typedef struct {
  // the first part of the filter definition is a set of diagonal matrixes loaded with the uncertainties
  // the matrices are diagonal in the first three elements, since velocity and acceleration are not measuure, uncertainty = 0
  float Q[FILTER_SIZE * FILTER_SIZE];  // this is a diagonal matrix (over 1st three elements) with the process uncertainty
  float R1[FILTER_SIZE * FILTER_SIZE]; // same here for the northstart
  float R2[FILTER_SIZE * FILTER_SIZE]; // same here for the wheel encoders
  //
  // the Phi matrix encodes the linear equations of motion that govern the internal filter model of the robot
  float Phi[FILTER_SIZE * FILTER_SIZE]; 
  //
  // these hold the difference between the last prediction and the current measurements
  float residual_s1[FILTER_SIZE];
  float residual_s2[FILTER_SIZE];
  //
  // the W matrics (a.k.a) Kalman gain are the computed weight factors corresponding to each of the sensor measurements
  float W1[FILTER_SIZE * FILTER_SIZE];
  float W2[FILTER_SIZE * FILTER_SIZE];

  // this is the current internal state of the filter model .. it is also the current prediction
  float current_state[FILTER_SIZE];
  
  // this matrix is the statistical covariance of the overall system computed and updated at each step
  float P[FILTER_SIZE * FILTER_SIZE]; 
} kalmanFilter;

void initKalmanFilter(kalmanFilter *, float *, float *,  int );
void rovioKalmanFilter(kalmanFilter *, float *, float *, float *);
void rovioKalmanFilterSetVelocity(kalmanFilter *,float *);
void rovioKalmanFilterSetUncertainty(kalmanFilter *, float *);

#endif
