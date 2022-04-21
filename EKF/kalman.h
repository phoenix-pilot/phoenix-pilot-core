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

#define STATE_COLS 1
#define STATE_ROWS 16
#define MEAS_ROWS 6

#define EARTH_G 9.80665F

#define imax 0
#define imay 1
#define imaz 2
#define imwx 3
#define imwy 4
#define imwz 5

#define ixx 0
#define ixy 1
#define ixz 2
#define ivx 3
#define ivy 4 
#define ivz 5
#define iqa 6
#define iqb 7
#define iqc 8
#define iqd 9
#define iax 10
#define iay 11
#define iaz 12
#define iwx 13
#define iwy 14
#define iwz 15
#define iaxp 16
#define iayp 17
#define iazp 18
#define iwxp 19
#define iwyp 20
#define iwzp 21

#define xx state->data[0]
#define xy state->data[1]
#define xz state->data[2]
#define vx state->data[3]
#define vy state->data[4] 
#define vz state->data[5]
#define qa state->data[6]
#define qb state->data[7]
#define qc state->data[8]
#define qd state->data[9]
#define ax state->data[10]
#define ay state->data[11]
#define az state->data[12]
#define wx state->data[13]
#define wy state->data[14]
#define wz state->data[15]
#define axp state->data[16]
#define ayp state->data[17]
#define azp state->data[18]
#define wxp state->data[19]
#define wyp state->data[20]
#define wzp state->data[21]

/* CALIBRATION */

void imu_calibrate_acc_gyr(void);

/* T=0 VALUES INITIALIZATION METHODS */

/* state values at t=0 initialization method */
void init_state_vector(phmatrix_t * state);

/* covariance values at t=0 initialization method */
void init_cov_vector(phmatrix_t * cov);


/* PHMATRIX MATRICES MEMORY INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
void init_prediction_matrices(phmatrix_t * state, phmatrix_t * state_est, phmatrix_t * cov, phmatrix_t * cov_est, phmatrix_t * F, phmatrix_t * Q, float dt);

/* initializes matrices related to update/innovation step of kalman filter */
void init_update_matrices(phmatrix_t * H, phmatrix_t * R);

/* JACOBIANS CALCULATIONS */

/* calculates jacobian matrix F given state and control vector (control vector implemen. absent) */
extern void jacobian_F(phmatrix_t * state, phmatrix_t * F, float dt);

/* calculates jacobian matrix H given state estimate */
extern void jacobian_H(phmatrix_t * state, phmatrix_t * H, float dt);


/* KALMAN STEPS METHODS */

/* performs kalman prediction step */
extern void kalman_predict(phmatrix_t * state, phmatrix_t * cov, phmatrix_t * state_est, phmatrix_t * cov_est, phmatrix_t * F, phmatrix_t * Q, float dt, int verbose);

/* performs kalman update/innovation step */
extern void kalman_update(phmatrix_t * state, phmatrix_t * cov, phmatrix_t * state_est, phmatrix_t * cov_est, phmatrix_t * H, phmatrix_t * R, float dt, int verbose);


#endif
