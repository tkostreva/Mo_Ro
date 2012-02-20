/* Kalman Redundant Sensors, C Version */

#include <clapack.h>
#include "rovioKalmanFilter.h"
#include "kalmanFilterDef.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define ROWCOL(I,J) (I*FILTER_SIZE+J)

/* Initialize the filter */
void initKalmanFilter(kalmanFilter *kf, float *initPose, float *velocity, int deltat) {

  int i;
  // zero the filter arrays
  memset(kf->Q, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->Phi, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->R1, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->R2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->W1, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->W2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  memset(kf->P, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
  
  /* Initialize the model array */
  diag(kf->Phi,1);
  kf->Phi[ROWCOL(0,3)]=deltat;
  kf->Phi[ROWCOL(1,4)]=deltat;
  kf->Phi[ROWCOL(2,5)]=deltat;
  kf->Phi[ROWCOL(3,6)]=deltat;
  kf->Phi[ROWCOL(4,7)]=deltat;
  kf->Phi[ROWCOL(5,8)]=deltat;

  // initialize the uncertainty array
  kf->Q[0] = PROCESS_UNCERTAINTY_X;
  kf->Q[FILTER_SIZE+1]   = PROCESS_UNCERTAINTY_Y;
  kf->Q[2*FILTER_SIZE+2] = PROCESS_UNCERTAINTY_TH;

  kf->R1[0] = NORTHSTAR_UNCERTAINTY_X;
  kf->R1[FILTER_SIZE+1]   = NORTHSTAR_UNCERTAINTY_Y;
  kf->R1[2*FILTER_SIZE+2] = NORTHSTAR_UNCERTAINTY_TH;

  kf->R2[0] = WHEELENC_UNCERTAINTY_X;
  kf->R2[FILTER_SIZE+1]   = WHEELENC_UNCERTAINTY_Y;
  kf->R2[2*FILTER_SIZE+2] = WHEELENC_UNCERTAINTY_TH;

  // initialize the state
  kf->current_state[0] = initPose[0];
  kf->current_state[1] = initPose[1];
  kf->current_state[2] = initPose[2];
  kf->current_state[3] = velocity[0];
  kf->current_state[4] = velocity[1];
  kf->current_state[5] = velocity[2];
  kf->current_state[6] = 0;
  kf->current_state[7] = 0;
  kf->current_state[8] = 0;

return;
}

void rovioKalmanFilter(kalmanFilter *kf, float *meas_S1, float *meas_S2, float *predicted) {
	int i;

	/* Variables for matrix inversion */	
	int mtrx_sz = FILTER_SIZE;
	int info, lwork;
	int *ipiv = NULL;
	float *work = NULL;

	// some temp variables that we need
	float temp[FILTER_SIZE * FILTER_SIZE];
	float temp2[FILTER_SIZE * FILTER_SIZE];
	float temp3[FILTER_SIZE * FILTER_SIZE];
	float Ptmp[FILTER_SIZE * FILTER_SIZE];
	float eye[FILTER_SIZE * FILTER_SIZE];
	float new_state[FILTER_SIZE];
	
	/* Clear the temp matricies that aren't directly overwritten */
	memset(eye, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);

	/* Allocate ipiv and work */
	ipiv = malloc(mtrx_sz * sizeof(int));
	lwork = FILTER_SIZE * FILTER_SIZE;
	work = malloc(sizeof(float) * lwork);
	if(!ipiv || !work) {
		if(ipiv)
			free(ipiv);
		if(work)
			free(work);
		printf("Error allocating memory!\n");
		return;
	}

	/**** 2. Propagate the Covariance Matrix ****/
	/* temp2 = Phi * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f, 
		kf->Phi, FILTER_SIZE, kf->P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE); 

	/* temp = temp2 * Phi' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, kf->Phi, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* P = temp + Q */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++) 
		kf->P[i] = temp[i] + kf->Q[i]; 

	/**** 3. Propagate the model track estimate ****/
	/* new_state = Phi * current_state */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, 1, FILTER_SIZE, 1.0f,
		kf->Phi, FILTER_SIZE, kf->current_state, 1, 0.0f, new_state, 1);

	for(i=0; i<3; i++) {
	  kf->residual_s1[i] = meas_S1[i] - new_state[i];
	  kf->residual_s2[i] = meas_S2[i] - new_state[i];
	}
	for(i=3; i<9; i++) {
	  kf->residual_s1[i] = 0;
	  kf->residual_s2[i] = 0;
	}

	/* temp = P + R1 */
	for(i=0; i<FILTER_SIZE * FILTER_SIZE; i++) 
	  temp[i] = kf->P[i] + kf->R1[i];

	/* Invert temp by first performing an LU Decomposition */
	info = clapack_sgetrf(CblasRowMajor, mtrx_sz, mtrx_sz, temp, mtrx_sz, ipiv);

	/* Now, invert given the LU decomposition */
	info = clapack_sgetri(CblasRowMajor, mtrx_sz, temp, mtrx_sz, ipiv);

	/* W1 = P * temp */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		kf->P, FILTER_SIZE, temp, FILTER_SIZE, 0.0f, kf->W1, FILTER_SIZE);
	
	/* temp = P + R2 */
	for(i=0; i<FILTER_SIZE * FILTER_SIZE; i++) 
		temp[i] = kf->P[i] + kf->R2[i];
	
	/* Invert temp by first performing an LU Decomposition */
	info = clapack_sgetrf(CblasRowMajor, mtrx_sz, mtrx_sz, temp, mtrx_sz, ipiv);

	/* Now, invert given the LU decomposition */
	info = clapack_sgetri(CblasRowMajor, mtrx_sz, temp, mtrx_sz, ipiv);

	/* W2 = P * temp */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		kf->P, FILTER_SIZE, temp, FILTER_SIZE, 0.0f, kf->W2, FILTER_SIZE);

	/**** 6. Update the estimate ****/

	/* W1 * residual_s1' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, 1, 1.0f,
		kf->W1, FILTER_SIZE, kf->residual_s1, FILTER_SIZE, 0.0f, temp, 1);

	/* W2 * residual_s2' */	
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, 1, 1.0f,
		kf->W2, FILTER_SIZE, kf->residual_s2, FILTER_SIZE, 0.0f, temp2, 1);
	/* temp = temp + temp2 */
	for(i=0; i<FILTER_SIZE; i++)
		temp[i] = temp[i] + temp2[i];

	/* predicted = new_state + temp */
	for(i=0; i<FILTER_SIZE; i++){
		predicted[i] = new_state[i] + temp[i];
	        kf->current_state[i] = predicted[i];
	}

	/* Clear the temp vars */
	memset(temp, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);
	memset(temp2, 0, sizeof(float) * FILTER_SIZE * FILTER_SIZE);

	diag(eye, 1);

	/* temp3 = eye - W1 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		temp3[i] = eye[i] - kf->W1[i];
	/* temp2 = temp3 * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, kf->P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* temp = temp2 * (temp3)' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, temp3, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* temp3 = W1 * R1 */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		kf->W1, FILTER_SIZE, kf->R1, FILTER_SIZE, 0.0f, temp3, FILTER_SIZE);
	/* temp2 = temp2 * W1' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, kf->W1, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* Ptmp = temp + temp2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		Ptmp[i] = temp[i] + temp2[i];	

	/* temp3 = eye - W2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		temp3[i] = eye[i] - kf->W2[i];
	/* temp2 = temp3 * P */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, kf->P, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);
	/* temp = temp2 * (temp3)' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp2, FILTER_SIZE, temp3, FILTER_SIZE, 0.0f, temp, FILTER_SIZE);

	/* temp3 = W2 * R2 */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		kf->W2, FILTER_SIZE, kf->R2, FILTER_SIZE, 0.0f, temp3, FILTER_SIZE);
	/* P = temp2 * W2' */
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, FILTER_SIZE, FILTER_SIZE, FILTER_SIZE, 1.0f,
		temp3, FILTER_SIZE, kf->W2, FILTER_SIZE, 0.0f, temp2, FILTER_SIZE);

	/* P = Ptmp + temp + temp2 */
	for(i=0; i<FILTER_SIZE*FILTER_SIZE; i++)
		kf->P[i] = Ptmp[i] + temp[i] + temp2[i];	
	
	//	pmat(P);

	free(ipiv);
	return;
}
void rovioKalmanFilterSetVelocity(kalmanFilter *kf, float *velocity)
{
  // changes the velocity values in the state vector
  // velocity paramter is {vx,vy,vth}
  kf->current_state[3] = velocity[0];
  kf->current_state[4] = velocity[1];
  kf->current_state[5] = velocity[2];
}

void rovioKalmanFilterSetUncertainty(kalmanFilter *kf, float *uncertainty)
{
  // sets the process and sensor uncertainty matrices
  // uncertainty is a nine element float vector
  // set to {proc_x,proc_y,proc_th,ns_x,ns_y,ns_th,we_x,we_y,we_th}

  
  // initialize the uncertainty array
  kf->Q[ROWCOL(0,0)]     = uncertainty[0];
  kf->Q[ROWCOL(1,1)]     = uncertainty[1];
  kf->Q[ROWCOL(2,2)]     = uncertainty[2];

  kf->R1[ROWCOL(0,0)]     = uncertainty[3];
  kf->R1[ROWCOL(1,1)]     = uncertainty[4];
  kf->R1[ROWCOL(2,2)]     = uncertainty[5];

  kf->R2[ROWCOL(0,0)]     = uncertainty[6];
  kf->R2[ROWCOL(1,1)]     = uncertainty[7];
  kf->R2[ROWCOL(2,2)]     = uncertainty[8];
}
