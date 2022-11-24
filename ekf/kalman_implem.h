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

#include <matrix.h>
#include <vec.h>
#include <quat.h>

#include "kalman_core.h"
#include "meas.h"

#define STATE_COLS 1
#define STATE_ROWS 20

#define IMUMEAS_ROWS  14
#define BAROMEAS_ROWS 2
#define GPSMEAS_ROWS  4

#define EARTH_G       9.80665F   /* m/s^2 */
#define UNI_GAS_CONST 8.3144598F /* J/(mol * K) */
#define AIR_MOL_MASS  0.0289644F /* kg/mol */

#define DEG2RAD 0.0174532925

#define BARO_UPDATE_PERIOD 50000 /* time in microseconds between barometer update procedure */

/* abbreviation of IndexMeasurement(valuename) */

/* imu measurements */
#define IMAX 0
#define IMAY 1
#define IMAZ 2
#define IMWX 3
#define IMWY 4
#define IMWZ 5
#define IMMX 6
#define IMMY 7
#define IMMZ 8
#define IMQA 9
#define IMQB 10
#define IMQC 11
#define IMQD 12
#define IMBAZ 13

/* baro measurements */
#define IMBXZ 0
#define IMBVZ 1

/* gps measurements */
#define IMGPSXX 0
#define IMGPSXY 1
#define IMGPSVX 2
#define IMGPSVY 3

/* index of state variable of: */
#define IXX 0  /* position x */
#define IXY 1  /* position y */
#define IXZ 2  /* position z */
#define IVX 3  /* velocity x */
#define IVY 4  /* velocity y */
#define IVZ 5  /* velocity z */
#define IQA 6  /* rotation quaternion real part */
#define IQB 7  /* rotation quaternion imaginary i part */
#define IQC 8  /* rotation quaternion imaginary j part */
#define IQD 9  /* rotation quaternion imaginary k part */
#define IAX 10 /* earth based acceleration x */
#define IAY 11 /* earth based acceleration y */
#define IAZ 12 /* earth based acceleration z */
#define IWX 13 /* earth based angular speed x */
#define IWY 14 /* earth based angular speed y */
#define IWZ 15 /* earth based angular speed z */
#define IMX 16 /* magnetic field x */
#define IMY 17 /* magnetic field y */
#define IMZ 18 /* magnetic field z */
#define IBAZ 19 /* acceleration z bias */


/* value name */
#define XX state->data[0]
#define XY state->data[1]
#define XZ state->data[2]
#define VX state->data[3]
#define VY state->data[4]
#define VZ state->data[5]
#define QA state->data[6]
#define QB state->data[7]
#define QC state->data[8]
#define QD state->data[9]
#define AX state->data[10]
#define AY state->data[11]
#define AZ state->data[12]
#define WX state->data[13]
#define WY state->data[14]
#define WZ state->data[15]
#define MX state->data[16]
#define MY state->data[17]
#define MZ state->data[18]
#define BAZ state->data[19]

/* IMPORTANT: must be kept in order with 'char * configNames' in 'kalman.inits.c' */
typedef struct {
	int verbose;
	int log;

	float P_xerr;    /* initial covariance of position x/y/z */
	float P_verr;    /* initial covariance of velocity x/y/z */
	float P_aerr;    /* initial covariance of accelerations x/y/z */
	float P_werr;    /* initial covariance of angular rates x/y/z */
	float P_merr;    /* initial covariance of magnetic flux measurment x/y/z */
	float P_qaerr;   /* initial covariance of rotation quaternion real part a */
	float P_qijkerr; /* initial covariance of rotation quaternion imaginary parts i/j/k */
	float P_pxerr;   /* initial covariance of pressure measurement */
	float P_bazerr;  /* initial covariance of accelerometer z bias */

	float R_acov; /* measurement noise of acceleration */
	float R_wcov; /* measurement noise of angular rates */
	float R_mcov; /* measurement noise of magnetic flux */
	float R_qcov; /* measurement noise of rotation quaternion */
	float R_azbias; /* measurement noise of z axis bias */

	float R_xzcov;
	float R_vzcov;

	float Q_xcov;
	float Q_vcov;
	float Q_hcov;     /* process noise of altitude */
	float Q_avertcov; /* process noise of vertical acceleration */
	float Q_ahoricov; /* process noise of horizontal accelerations */
	float Q_wcov;     /* process noise of angular rates */
	float Q_mcov;     /* process noise of magnetic flux */
	float Q_qcov;     /* process noise of rotation quaternion */
	float Q_azbias;   /* process noise of accelerometer z axis bias */
} kalman_init_t;


int verbose;


extern void kmn_configRead(kalman_init_t *initVals);


/* PHMATRIX MATRICES INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
extern int kmn_predInit(state_engine_t *engine, const meas_calib_t *calib, const kalman_init_t *inits);

/* deinitializes prediction matrices */
extern void kmn_predDeinit(state_engine_t *engine);

/* imu update engine composer */
extern void kmn_imuEngInit(update_engine_t *engine, const kalman_init_t *inits);

/* barometer update engine composer */
extern void kmn_baroEngInit(update_engine_t *engine, const kalman_init_t *inits);

/* GPS update engine composer */
extern void kmn_gpsEngInit(update_engine_t *engine, const kalman_init_t *inits);


#endif
