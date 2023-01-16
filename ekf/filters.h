/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * auxiliary data filters
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _EKF_FILTERS_
#define _EKF_FILTERS_

#include <vec.h>


/* Filters accelerometer signal using windowed-sinc FIR filter. Passing null clears the buffer. Thread unsafe! */
void fltr_accLpf(vec_t *raw);

/* Filters barometer speed signal using windowed-sinc FIR filter. Passing null clears the buffer. Thread unsafe! */
void fltr_vBaroLpf(float *raw);

void fltr_gyroLpf(vec_t *raw);


#endif
