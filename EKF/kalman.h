/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PHKALMAN_H
#define PHKALMAN_H

#include <tools/phmatrix.h>
#include <tools/rotas_dummy.h>

#define STATE_COLS 1
#define STATE_ROWS 19
#define MEAS_ROWS  13

#define EARTH_G 9.80665F

/* abbreviation of IndexMeasurement(valuename) */
#define imax 0
#define imay 1
#define imaz 2
#define imwx 3
#define imwy 4
#define imwz 5
#define immx 6
#define immy 7
#define immz 8
#define imqa 9
#define imqb 10
#define imqc 11
#define imqd 12

/* abbreviation of index(valuename */
#define ixx  0
#define ixy  1
#define ixz  2
#define ivx  3
#define ivy  4
#define ivz  5
#define iqa  6
#define iqb  7
#define iqc  8
#define iqd  9
#define iax  10
#define iay  11
#define iaz  12
#define iwx  13
#define iwy  14
#define iwz  15
#define imx  16
#define imy  17
#define imz  18


/* vale name */
#define xx  state->data[0]
#define xy  state->data[1]
#define xz  state->data[2]
#define vx  state->data[3]
#define vy  state->data[4]
#define vz  state->data[5]
#define qa  state->data[6]
#define qb  state->data[7]
#define qc  state->data[8]
#define qd  state->data[9]
#define ax  state->data[10]
#define ay  state->data[11]
#define az  state->data[12]
#define wx  state->data[13]
#define wy  state->data[14]
#define wz  state->data[15]
#define mx  state->data[16]
#define my  state->data[17]
#define mz  state->data[18]

/* initial values calculated during calibration */
quat_t init_q;
vec_t init_m;
vec_t gyr_nivel;
int verbose;

/* New elements can be added only at the end of structure */
typedef struct {
	int verbose;

	float P_xerr;
	float P_verr;
	float P_aerr;
	float P_werr;
	float P_merr;
	float P_qaerr;
	float P_qijkerr;

	float R_acov;
	float R_wcov;
	float R_mcov;
	float R_qcov;

	float Q_acov;
	float Q_wcov;
	float Q_mcov;
	float Q_qcov;
} kalman_init_t;

/* CALIBRATION AND MEASUREMENTS */

void read_config(void);

void imu_calibrate_acc_gyr_mag(void);

vec_t *imu_measurements(void);

/* T=0 VALUES INITIALIZATION METHODS */

/* state values at t=0 initialization method */
void init_state_vector(phmatrix_t *state);

/* covariance values at t=0 initialization method */
void init_cov_vector(phmatrix_t *cov);


/* PHMATRIX MATRICES MEMORY INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
void init_prediction_matrices(phmatrix_t *state, phmatrix_t *state_est, phmatrix_t *cov, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt);

/* initializes matrices related to update/innovation step of kalman filter */
void init_update_matrices(phmatrix_t *H, phmatrix_t *R);

/* JACOBIANS CALCULATIONS */

/* calculates jacobian matrix F given state and control vector (control vector implemen. absent) */
extern void jacobian_F(phmatrix_t *state, phmatrix_t *F, float dt);

/* calculates jacobian matrix H given state estimate */
extern void jacobian_H(phmatrix_t *state, phmatrix_t *H, float dt);


/* KALMAN STEPS METHODS */

/* performs kalman prediction step */
extern void kalman_predict(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt, int verbose);

/* performs kalman update/innovation step */
extern void kalman_update(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *H, phmatrix_t *R, float dt, int verbose);


#endif
