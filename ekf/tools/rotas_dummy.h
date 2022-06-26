/*
 * Phoenix-Pilot
 *
 * rotas
 *
 * quaternion and 3d vectors operations library - header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef ROTAS_DUMMY_H
#define ROTAS_DUMMY_H

/* identity quaternion macro */
#define IDEN_QUAT \
	(quat_t) { .a = 1, .i = 0, .j = 0, .k = 0 }

#define PI_QUAT \
	(quat_t) { .a = 0, .i = 1, .j = 0, .k = 0 }


typedef struct {
	float i, j, k, a;
} quat_t;


/* vector has additional value 'l' that is only for easy casting to/from quaternion (quat_t <-> vec_t) */
typedef struct {
	float x, y, z, l;
} vec_t;


/* returns vector initialized with passed values */
vec_t vec(float x, float y, float z);


/* C = A + B; stores sum of A and B into C */
void vec_sum(const vec_t *A, const vec_t *B, vec_t *C);


/* A += B; adds B to A */
void vec_add(vec_t *A, const vec_t *B);


/* C = A - B; stores difference of A and B in C */
void vec_dif(const vec_t *A, const vec_t *B, vec_t *C);


/* A -= B; subtracts B from A */
void vec_sub(vec_t *A, const vec_t *B);


/* C = A x B; stores cross product of A and B into C */
void vec_cross(const vec_t *A, const vec_t *B, vec_t *C);


/* returns the dot product of two vectors */
float vec_dot(const vec_t *A, const vec_t *B);


/* multiplies each vector element times scalar 'a' */
void vec_times(vec_t *A, float a);


/* returns length of passed vector */
float vec_len(const vec_t *A);


/* calculates unit vector C that is perpendicular to both A and B */
void vec_normal(const vec_t *A, const vec_t *B, vec_t *C);


/* normalizes A to a unit vector */
void vec_normalize(vec_t *A);


/* get quaternion */
quat_t quat(float a, float i, float j, float k);


/* A += B; adds quaternion B to A */
void quat_add(quat_t *A, const quat_t *B);


/* A * B = C; multiply A and B and store result in C */
void quat_mlt(const quat_t *A, const quat_t *B, quat_t *C);


/* x = AB | returns euclidian 4D dot product of two quaternions */ 
float quat_dot(const quat_t *A, const quat_t *B);


/* B = A* | conjugates quaternion A */
void quat_cjg(quat_t *A);


/* calculates C that is result of: A * B * cjg(A), C must not overlap with A or B */
void quat_sandwich(const quat_t *A, const quat_t *B, quat_t *C);


/* print quaternion in human readable form */
void quat_print(const quat_t *A);


/* C = C / ||C|| | normalizes quaternion to a unit quaternion */
void quat_normalize(quat_t * A);


/* calculates C that is result of: A * B * cjg(A), C can overlap with A or B */
void quat_sandwichFast(const quat_t *A, const quat_t *B, quat_t *C);


/* multiplies each quaternion element by a factor of x */
void quat_times(quat_t *A, float x);


/* Transforms */


/* calculate quaternion q that rotates v1 into v2, assumed len(v1) == len(v2) */
void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q);


/* rotates vector using rotation quaternion */
void quat_vecRot(vec_t *vec, const quat_t *qRot);


/* calculates quaternion res (closest to help_q), that rotates frame of reference (v1, v2) into (w1, w2) */
void quat_frameRot(const vec_t *v1, const vec_t *v2, const vec_t *w1, const vec_t *w2, quat_t *res, const quat_t *help_q);


/* calculates rotation euler angles from rotation quaternion */
void quat_quat2euler(quat_t *q, float *roll, float *pitch, float *yaw);

#endif
