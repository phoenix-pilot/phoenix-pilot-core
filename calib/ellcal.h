/*
 * Phoenix-Pilot
 *
 * Ellipsoidal calibration common functions
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef CALIBTOOL_ELLCAL_H_
#define CALIBTOOL_ELLCAL_H_

#include <vec.h>

/* 
 * Performs acquisition of `nSamples` data points of `sensorType` type during rotational movement od the device.
 * Measurements are taken each `deltaAngle` radians of rotation. Assumes `sensc` is initialized.
 * 
 * Possible sensortypes: SENSOR_TYPE_ACCEL, SENSOR_TYPE_MAG.
*/
int ellcal_rotDataGet(vec_t *samplesBuf, size_t nSamples, float deltaAngle, int sensorType);


/*
* Interprets parameters returned by the LMA for ellipsoid fitting:
*
* - ellipsoid center offset is saved to 3x1 matrix pointed by `H`
* - ellipsoid deformation is saved to 3x3 matrix pointed by `S`
*/
void ellcal_lma2matrices(const matrix_t *lmaParams, matrix_t *S, matrix_t *H);


/* Prepared residuum calculation callback for ellipsoid fitting */
int ellcal_lmaResiduum(const matrix_t *P, const matrix_t *V, float *res, bool log);


/* Prepared jacobian calculation callback for ellipsoid fitting */
int ellcal_lmaJacobian(const matrix_t *P, const matrix_t *V, matrix_t *J, bool log);


/* Prepared initial guess callback for ellipsoid fitting */
void ellcal_lmaGuess(matrix_t *P);

#endif
