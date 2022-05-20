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

#include <tools/phmatrix.h>

/* Function that acquires measuremnets and puts it into Z matrix */
typedef phmatrix_t *(*dataGetter)(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, float dt);

/*  function that fills jacobian matrix H based on data from state vector and dt */
typedef void (*updateJacobian)(phmatrix_t *H, phmatrix_t *state, float dt);

/* function that fills state estimation based on current state and dt */
typedef void (*stateEstimation)(phmatrix_t *state, phmatrix_t *state_est, float dt);

/* function that calculates measurements of some update model based on current state estimation */
typedef phmatrix_t *(*predictMeasurements)(phmatrix_t *state_est);


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

} update_engine_t;


typedef struct {
	phmatrix_t *state;
	phmatrix_t *state_est;
	phmatrix_t *cov;
	phmatrix_t *cov_est;

	phmatrix_t *F;
	phmatrix_t *Q;

	stateEstimation estimateState;
	updateJacobian getJacobian;
} state_engine_t;


/* performs kalman prediction step */
void kalmanPredictionStep(state_engine_t *engine, float dt, int verbose);


/* performs kalman measurement update step */
int kalmanUpdateStep(float dt, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine);

#endif
