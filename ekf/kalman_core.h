/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * core functions header
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PHKALMAN_CORE_H
#define PHKALMAN_CORE_H

#include "tools/phmatrix.h"

/* 
* EKF needs matrices of correct sizes to perform update step. 
* This macro provides a way to staticaly declare all necessary matrices/memory to populate measurement engine 
* TODO: some matrices can be shared across measurement models
*/
#define DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_SIZE, MEASUREMENT_SIZE) \
	static float ekf_Z_data[(MEASUREMENT_SIZE)*1] = { 0 }; \
	static float ekf_Y_data[(MEASUREMENT_SIZE)*1] = { 0 }; \
	static float ekf_S_data[(MEASUREMENT_SIZE) * (MEASUREMENT_SIZE)] = { 0 }; \
	static float ekf_K_data[(STATE_SIZE) * (MEASUREMENT_SIZE)] = { 0 }; \
	static float ekf_I_data[(STATE_SIZE) * (STATE_SIZE)] = { 0 }; \
	static float ekf_H_data[(MEASUREMENT_SIZE) * (STATE_SIZE)] = { 0 }; \
	static float ekf_R_data[(MEASUREMENT_SIZE) * (MEASUREMENT_SIZE)] = { 0 }; \
	static float ekf_hx_data[(MEASUREMENT_SIZE)*1] = { 0 }; \
	static float ekf_tmp1_data[(MEASUREMENT_SIZE) * (MEASUREMENT_SIZE)] = { 0 }; \
	static float ekf_tmp2_data[(STATE_SIZE) * (MEASUREMENT_SIZE)] = { 0 }; \
	static float ekf_tmp3_data[(MEASUREMENT_SIZE) * (STATE_SIZE)] = { 0 }; \
	static float ekf_tmp4_data[(STATE_SIZE) * (STATE_SIZE)] = { 0 }; \
	static float ekf_tmp5_data[(STATE_SIZE)*1] = { 0 }; \
	static phmatrix_t ekf_Z = { .rows = (MEASUREMENT_SIZE), .cols = 1, .transposed = 0, .data = ekf_Z_data };                        /* measurement matrix */ \
	static phmatrix_t ekf_Y = { .rows = (MEASUREMENT_SIZE), .cols = 1, .transposed = 0, .data = ekf_Y_data };                        /* innovation matrix */ \
	static phmatrix_t ekf_S = { .rows = (MEASUREMENT_SIZE), .cols = (MEASUREMENT_SIZE), .transposed = 0, .data = ekf_S_data };       /* innovation covariance matrix */ \
	static phmatrix_t ekf_K = { .rows = (STATE_SIZE), .cols = (MEASUREMENT_SIZE), .transposed = 0, .data = ekf_K_data };             /* kalman gain matrix */ \
	static phmatrix_t ekf_I = { .rows = (STATE_SIZE), .cols = (STATE_SIZE), .transposed = 0, .data = ekf_I_data };                   /* square identity matrix */ \
	static phmatrix_t ekf_H = { .rows = (MEASUREMENT_SIZE), .cols = (STATE_SIZE), .transposed = 0, .data = ekf_H_data };             /* jacobian matrix */ \
	static phmatrix_t ekf_R = { .rows = (MEASUREMENT_SIZE), .cols = (MEASUREMENT_SIZE), .transposed = 0, .data = ekf_R_data };       /* jacobian matrix */ \
	static phmatrix_t ekf_hx = { .rows = (MEASUREMENT_SIZE), .cols = 1, .transposed = 0, .data = ekf_hx_data };                      /* h(x) matrix */ \
	static phmatrix_t ekf_tmp1 = { .rows = (MEASUREMENT_SIZE), .cols = (MEASUREMENT_SIZE), .transposed = 0, .data = ekf_tmp1_data }; /* temporary matrix #1: small square */ \
	static phmatrix_t ekf_tmp2 = { .rows = (STATE_SIZE), .cols = (MEASUREMENT_SIZE), .transposed = 0, .data = ekf_tmp2_data };       /* temporary matrix #2: high rectangular */ \
	static phmatrix_t ekf_tmp3 = { .rows = (MEASUREMENT_SIZE), .cols = (STATE_SIZE), .transposed = 0, .data = ekf_tmp3_data };       /* temporary matrix #3: wide rectangular UNUSED */ \
	static phmatrix_t ekf_tmp4 = { .rows = (STATE_SIZE), .cols = (STATE_SIZE), .transposed = 0, .data = ekf_tmp4_data };             /* temporary matrix #4: big square */ \
	static phmatrix_t ekf_tmp5 = { .rows = (STATE_SIZE), .cols = 1, .transposed = 0, .data = ekf_tmp5_data };                        /* temporary matrix #4: state length column vector */ \
	static float ekf_S_inv_buff[(MEASUREMENT_SIZE) * (MEASUREMENT_SIZE)*2];                                                          /* S inversion buffer */ \
	static unsigned int ekf_S_inv_buff_len = (MEASUREMENT_SIZE) * (MEASUREMENT_SIZE)*2;                                              /* S inversion buffer size */

#define POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(measurementEngine) \
	(measurementEngine->Z) = &ekf_Z; \
	(measurementEngine->Y) = &ekf_Y; \
	(measurementEngine->S) = &ekf_S; \
	(measurementEngine->K) = &ekf_K; \
	(measurementEngine->I) = &ekf_I; \
	(measurementEngine->H) = &ekf_H; \
	(measurementEngine->R) = &ekf_R; \
	(measurementEngine->hx) = &ekf_hx; \
	(measurementEngine->invBuf) = ekf_S_inv_buff; \
	(measurementEngine->invBufLen) = ekf_S_inv_buff_len; \
	(measurementEngine->tmp1) = &ekf_tmp1; \
	(measurementEngine->tmp2) = &ekf_tmp2; \
	(measurementEngine->tmp3) = &ekf_tmp3; \
	(measurementEngine->tmp4) = &ekf_tmp4; \
	(measurementEngine->tmp5) = &ekf_tmp5;


/* Function that acquires measuremnets and puts it into Z matrix */
typedef phmatrix_t *(*dataGetter)(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, time_t timeStep);

/*  function that fills jacobian matrix H based on data from state vector and dt */
typedef void (*updateJacobian)(phmatrix_t *H, phmatrix_t *state, time_t timeStep);

/* function that fills state estimation based on current state and dt */
typedef void (*stateEstimation)(phmatrix_t *state, phmatrix_t *state_est, time_t timeStep);

/* function that calculates measurements of some update model based on current state estimation */
typedef phmatrix_t *(*predictMeasurements)(phmatrix_t *state_est, phmatrix_t *hx);

typedef void (*initMeasurementCov)(phmatrix_t *R);


/* Update engine is set of matrices and functions neccessary to perform update step with prediction step matrices */
typedef struct {
	/* user initializable update matrices pointers */
	phmatrix_t *H;
	phmatrix_t *R;

	/* standard EKF matrices pointers */
	phmatrix_t *Z;
	phmatrix_t *Y;
	phmatrix_t *S;
	phmatrix_t *K;
	phmatrix_t *I;
	phmatrix_t *hx;

	/* inversion helper elements */
	phmatrix_t *invS;
	float *invBuf;
	unsigned int invBufLen;

	/* phmatrix calculation buffers */
	phmatrix_t *tmp1;
	phmatrix_t *tmp2;
	phmatrix_t *tmp3;
	phmatrix_t *tmp4;
	phmatrix_t *tmp5;


	dataGetter getData;                      /* data getter function */
	updateJacobian getJacobian;              /* update step jacobian calculation */
	predictMeasurements predictMeasurements; /* predict hx bector based on state estimation */
	initMeasurementCov initMeasCov;

} update_engine_t;


typedef struct {
	phmatrix_t state;
	phmatrix_t state_est;
	phmatrix_t cov;
	phmatrix_t cov_est;

	phmatrix_t F;
	phmatrix_t Q;

	stateEstimation estimateState;
	updateJacobian getJacobian;
} state_engine_t;


/* performs kalman prediction step */
void kalmanPredictionStep(state_engine_t *engine, time_t timeStep, int verbose);

/* performs kalman measurement update step */
int kalmanUpdateStep(time_t timeStep, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine);

/* Initializes measurement engine. All measurement matrices HAVE TO BE PROVIDED at the time of this function call */
void kalmanCreateMeasurementEngine(initMeasurementCov initMeasCov, dataGetter getData, updateJacobian getJacobian, predictMeasurements predictMeasurements);

#endif
