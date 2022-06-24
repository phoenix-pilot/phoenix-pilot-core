/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * EKF implementation specific code header file. Declares prediction and update engines routines/functions
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
#include <stdint.h>

#include "tools/phmatrix.h"
#include "tools/rotas_dummy.h"

#include "kalman_core.h"

#define STATE_COLS 1
#define STATE_ROWS 21

#define IMUMEAS_ROWS  13
#define BAROMEAS_ROWS 4
#define GPSMEAS_ROWS  4

#define EARTH_G       9.80665F   /* m/s^2 */
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
#define imhz 0
#define imxz 1
#define imhv 2
#define imvz 3

/* gps measurements */
#define imgpsxx 0
#define imgpsxy 1
#define imgpsvx 2
#define imgpsvy 3

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
#define ihz 19 /* baro height */
#define ihv 20 /* baro speed */


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
#define hv state->data[20]

/* IMPORTANT: must be kept in order with 'char * config_names' in 'kalman.inits.c' */
typedef struct {
	int verbose;

	float P_xerr;    /* initial covariance of position x/y/z */
	float P_verr;    /* initial covariance of velocity x/y/z */
	float P_aerr;    /* initial covariance of accelerations x/y/z */
	float P_werr;    /* initial covariance of angular rates x/y/z */
	float P_merr;    /* initial covariance of magnetic flux measurment x/y/z */
	float P_qaerr;   /* initial covariance of rotation quaternion real part a */
	float P_qijkerr; /* initial covariance of rotation quaternion imaginary parts i/j/k */
	float P_pxerr;   /* initial covariance of pressure measurement */

	float R_acov; /* measurement noise of acceleration */
	float R_wcov; /* measurement noise of angular rates */
	float R_mcov; /* measurement noise of magnetic flux */
	float R_qcov; /* measurement noise of rotation quaternion */

	float R_pcov; /* measurement noise of pressure */
	float R_hcov; /* measurement noise of barometric altitude */
	float R_xzcov;
	float R_hvcov;
	float R_vzcov;

	float Q_xcov;
	float Q_vcov;
	float Q_hcov;     /* process noise of altitude */
	float Q_avertcov; /* process noise of vertical acceleration */
	float Q_ahoricov; /* process noise of horizontal accelerations */
	float Q_wcov;     /* process noise of angular rates */
	float Q_mcov;     /* process noise of magnetic flux */
	float Q_qcov;     /* process noise of rotation quaternion */
	float Q_pcov;     /* process noise of pressure */
	float Q_pvcov;
} kalman_init_t;


typedef struct {

} kalman_common_t;


typedef struct {
	float lat;
	float lon;
	float h;
} geodetic_t;

/* initial values calculated during calibration */
typedef struct {
	quat_t init_q;
	vec_t init_m;
	vec_t gyr_nivel;
	vec_t gpsRefEcef;
	float base_pressure;
	float base_temp;
	geodetic_t gpsRefGeodetic;
} kalman_calib_t;

int verbose;

/* CALIBRATIONS */
/* TODO: move to separate, calibration-related header */

void read_config(void);

void meas_imuCalib(void);

void meas_baroCalib(void);

const kalman_calib_t * meas_calibGet(void);

float meas_calibPressGet(void);


/* MEASUREMENT ACQUISITION */

int meas_imuGet(vec_t *accels, vec_t *gyros, vec_t *mags, uint64_t *timestamp);

int meas_baroGet(float *pressure, float *temperature, uint64_t *dtBaroUs);

int meas_gpsGet(vec_t *enu, vec_t *enu_speed, float *hdop);


/* PHMATRIX MATRICES INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
int kmn_predInit(state_engine_t *engine, const kalman_calib_t *calib);

/* deinitializes prediction matrices */
void kmn_predDeinit(state_engine_t *engine);

/* imu update engine composer */
void kmn_imuEngInit(update_engine_t *engine);

/* barometer update engine composer */
void kmn_baroEngInit(update_engine_t *engine);

/* GPS update engine composer */
void kmn_gpsEngInit(update_engine_t *engine);


#endif
