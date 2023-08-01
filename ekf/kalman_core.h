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

#include <stdbool.h>
#include <matrix.h>

/* UPDATE STEP FUNCTIONS */

/* Function that acquires measuremnets and puts it into Z matrix */
typedef matrix_t *(*dataGetter)(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep);

/*  function that fills jacobian matrix H based on data from state vector and dt */
typedef void (*updateJacobian)(matrix_t *H, matrix_t *state, time_t timeStep);

/*  function that fills jacobian matrix H based on data from state vector and dt */
typedef void (*predJacobian)(matrix_t *F, matrix_t *state, matrix_t *U, time_t timeStep);

/* PREDICTION STEP FUNCTIONS */

/* Function that acquires values of control vector and writes it to the U vector. On success returns pointer to U, NULL on fail */
typedef matrix_t *(*controlVectorGetter)(matrix_t *U);

/* function that fills state estimation based on current state and dt */
typedef void (*stateEstimation)(matrix_t *state, matrix_t *state_est, matrix_t *U, time_t timeStep);

/* function that calculates measurements of some update model based on current state estimation */
typedef matrix_t *(*predictMeasurements)(matrix_t *state_est, matrix_t *hx, time_t timestep);

/* function calculates current process noise covariance matrix based on 'state` and `U` matrices */
typedef void (*predNoiseGetter)(matrix_t *state, matrix_t *U, matrix_t *Q, time_t timestep);

typedef void (*initMeasurementCov)(matrix_t *R);


/* Update engine is set of matrices and functions neccessary to perform update step with prediction step matrices */
typedef struct {
	/* user initializable update matrices pointers */
	matrix_t H;
	matrix_t R;

	/* standard EKF matrices pointers */
	matrix_t Z;
	matrix_t Y;
	matrix_t S;
	matrix_t K;
	matrix_t I;
	matrix_t hx;

	/* inversion helper elements */
	matrix_t invS;
	float *invBuf;
	unsigned int invBufLen;

	/* phmatrix calculation buffers */
	matrix_t tmp1;
	matrix_t tmp2;
	matrix_t tmp3;
	matrix_t tmp4;
	matrix_t tmp5;

	dataGetter getData;                      /* data getter function */
	updateJacobian getJacobian;              /* update step jacobian calculation */
	predictMeasurements predictMeasurements; /* predict hx bector based on state estimation */
	initMeasurementCov initMeasCov;

	/* active/initialized flag */
	bool active;
} update_engine_t;


typedef struct {
	matrix_t state;
	matrix_t state_est;
	matrix_t cov;
	matrix_t cov_est;

	matrix_t U;

	matrix_t F;
	matrix_t Q;

	matrix_t B; /* buffer matrix for covariance estimate calculations */

	stateEstimation estimateState;
	predJacobian getJacobian;
	controlVectorGetter getControl;
	predNoiseGetter getNoiseQ;
} state_engine_t;


/* performs kalman prediction step */
extern void kalman_predict(state_engine_t *engine, time_t timeStep, int verbose);

/* performs kalman measurement update step */
extern int kalman_update(time_t timeStep, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine);

/* Deallocates update engine */
extern void kalman_updateDealloc(update_engine_t *engine);

/* Allocates update engine */
extern int kalman_updateAlloc(update_engine_t *engine, unsigned int stateLen, unsigned int measLen);

/* Deallocates prediction engine */
extern void kalman_predictDealloc(state_engine_t *engine);

/* Allocates prediction engine with state vector length `stateLen` and control vector length `ctrlLen` */
extern int kalman_predictAlloc(state_engine_t *engine, int stateLen, int ctrlLen);

#endif
