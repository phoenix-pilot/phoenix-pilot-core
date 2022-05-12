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

#include <sys/time.h>

#include <tools/phmatrix.h>
#include <tools/rotas_dummy.h>

#define STATE_COLS 1
#define STATE_ROWS 20
#define IMUMEAS_ROWS  13
#define BAROMEAS_ROWS 2

#define EARTH_G 9.80665F         /* m/s^2 */
#define UNI_GAS_CONST 8.3144598F /* J/(mol * K) */
#define AIR_MOL_MASS  0.0289644F /* kg/mol */

/* abbreviation of IndexMeasurement(valuename) */

/* imu measurements */
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

/* baro measurements */
//#define impx 0
#define imhz 0
#define imxz 1

/* index of state variable of: */
#define ixx 0  /* position x */
#define ixy 1  /* position y */
#define ixz 2  /* position z */
#define ivx 3  /* velocity x */
#define ivy 4  /* velocity y */
#define ivz 5  /* velocity z */
#define iqa 6  /* rotation quaternion real part */
#define iqb 7  /* rotation quaternion imaginary i part */
#define iqc 8  /* rotation quaternion imaginary j part */
#define iqd 9  /* rotation quaternion imaginary k part */
#define iax 10 /* earth based acceleration x */
#define iay 11 /* earth based acceleration y */
#define iaz 12 /* earth based acceleration z */
#define iwx 13 /* earth based angular speed x */
#define iwy 14 /* earth based angular speed y */
#define iwz 15 /* earth based angular speed z */
#define imx 16 /* magnetic field x */
#define imy 17 /* magnetic field y */
#define imz 18 /* magnetic field z */
#define ihz 19 /* pressure */


/* value name */
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
#define mx state->data[16]
#define my state->data[17]
#define mz state->data[18]
#define hz state->data[19]

/* initial values calculated during calibration */
quat_t init_q;
vec_t init_m;
vec_t gyr_nivel;
float base_pressure, base_temp;
int verbose;

/* IMPORTANT: must be kept in order with 'char * config_names' in 'kalman.inits.c' */
typedef struct {
	int verbose;

	float P_xerr;     /* initial covariance of position x/y/z */
	float P_verr;     /* initial covariance of velocity x/y/z */
	float P_aerr;     /* initial covariance of accelerations x/y/z */
	float P_werr;     /* initial covariance of angular rates x/y/z */
	float P_merr;     /* initial covariance of magnetic flux measurment x/y/z */
	float P_qaerr;    /* initial covariance of rotation quaternion real part a */
	float P_qijkerr;  /* initial covariance of rotation quaternion imaginary parts i/j/k */ 
	float P_pxerr;    /* initial covariance of pressure measurement */

	float R_acov;     /* measurement noise of acceleration */
	float R_wcov;     /* measurement noise of angular rates */
	float R_mcov;     /* measurement noise of magnetic flux */
	float R_qcov;     /* measurement noise of rotation quaternion */

	float R_pcov;     /* measurement noise of pressure */
	float R_hcov;     /* measurement noise of barometric altitude */
	float R_xzcov;

	float Q_hcov;     /* process noise of altitude */
	float Q_avertcov; /* process noise of vertical acceleration */
	float Q_ahoricov; /* process noise of horizontal accelerations */
	float Q_wcov;     /* process noise of angular rates */
	float Q_mcov;     /* process noise of magnetic flux */ 
	float Q_qcov;     /* process noise of rotation quaternion */
	float Q_pcov;     /* process noise of pressure */
} kalman_init_t;


typedef struct {
	float t;                      /* total time since filtering begin */
	float dt;                     /* current time step length */
	int lastprint;                /* printing flag for periodic print purposes */
	struct timeval last_time;     /* last kalman loop time */
	struct timeval current_time;  /* current kalman loop time */
} kalman_common_t;

/* CALIBRATION AND MEASUREMENTS */

void read_config(void);

void imu_calibrate_acc_gyr_mag(void);

void acquireImuMeasurements(vec_t *accels, vec_t *gyros, vec_t *mags);

int acquireBaroMeasurements(float * pressure);

/* T=0 VALUES INITIALIZATION METHODS */

/* state values at t=0 initialization method */
void init_state_vector(phmatrix_t *state);

/* covariance values at t=0 initialization method */
void init_cov_vector(phmatrix_t *cov);


/* PHMATRIX MATRICES MEMORY INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
void init_prediction_matrices(phmatrix_t *state, phmatrix_t *state_est, phmatrix_t *cov, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt);

/* initializes matrices related to imu update/innovation step of kalman filter */
void imuUpdateInitializations(phmatrix_t *H, phmatrix_t *R);

/* initializes matrices related to barometer update/innovation step of kalman filter */
void baroUpdateInitializations(phmatrix_t *H, phmatrix_t *R);

/* JACOBIANS CALCULATIONS */

/* calculates jacobian matrix F given state and control vector (control vector implemen. absent) */
void calcPredictionJacobian(phmatrix_t *state, phmatrix_t *F, float dt);

/* calculates jacobian matrix H given state estimate */
void calcImuJacobian(phmatrix_t *state, phmatrix_t *H, float dt);

/* calculates jacobian matrix H given state estimate */
void calcBaroJacobian(phmatrix_t *state, phmatrix_t *H, float dt);


/* KALMAN STEPS METHODS */

/* performs kalman prediction step */
void kalman_predict(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt, int verbose);

/* performs kalman update/innovation step */
void kalman_updateImu(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *H, phmatrix_t *R, float dt, int verbose);


int kalman_updateBaro(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *H, phmatrix_t *R, float dt, int verbose);


#endif
