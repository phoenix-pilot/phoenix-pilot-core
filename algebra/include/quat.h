/*
 * Phoenix-Pilot
 *
 * quat - quaternion algebra library - header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PHOENIX_QUAT_H__
#define __PHOENIX_QUAT_H__


/* identity quaternion macro */
#define IDEN_QUAT \
	(quat_t) { .a = 1, .i = 0, .j = 0, .k = 0 }

#define PI_QUAT \
	(quat_t) { .a = 0, .i = 1, .j = 0, .k = 0 }


#define DEG2RAD 0.0174532925


typedef struct {
	float i, j, k, a;
} quat_t;


/* get quaternion */
extern quat_t quat(float a, float i, float j, float k);


/* A += B; adds quaternion B to A */
extern void quat_add(quat_t *A, const quat_t *B);


/* A * B = C; multiply A and B and store result in C */
extern void quat_mlt(const quat_t *A, const quat_t *B, quat_t *C);


/* x = AB | returns euclidian 4D dot product of two quaternions */ 
extern float quat_dot(const quat_t *A, const quat_t *B);


/* B = A* | conjugates quaternion A */
extern void quat_cjg(quat_t *A);


/* calculates C that is result of: A * B * cjg(A), C must not overlap with A or B */
extern void quat_sandwich(const quat_t *A, const quat_t *B, quat_t *C);


/* print quaternion in human readable form */
extern void quat_print(const quat_t *A);


/* C = C / ||C|| | normalizes quaternion to a unit quaternion */
extern void quat_normalize(quat_t * A);


/* calculates C that is result of: A * B * cjg(A), C can overlap with A or B */
extern void quat_sandwichFast(const quat_t *A, const quat_t *B, quat_t *C);


/* multiplies each quaternion element by a factor of x */
extern void quat_times(quat_t *A, float x);


/* calculates rotation euler angles from rotation quaternion */
extern void quat_quat2euler(quat_t *q, float *roll, float *pitch, float *yaw);

#endif