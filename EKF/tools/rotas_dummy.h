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
#define IDEN_QUAT (quat_t){.a=1,.i=0,.j=0,.k=0}
#define PI_QUAT (quat_t){.a=0,.i=1,.j=0,.k=0}


typedef struct {
	float i, j, k, a;
} quat_t;


/* vector has additional value 'l' that is only for easy casting to/from quaternion (quat_t <-> vec_t) */
typedef struct {
	float x, y, z, l;
} vec_t;


/* returns vector initialized with passed values */
vec_t vec(float x, float y, float z);


/* returns sum of passed vectors */
vec_t vec_add(vec_t *A, vec_t *B);


/* subtracts two vectors */
vec_t vec_sub(vec_t *A, vec_t *B);


/* returns cross product of given vectors */
vec_t vec_cross(vec_t *A, vec_t *B);


/* returns the dot product of two vectors */
float vec_dot(vec_t *A, vec_t *B);


/* multiplies each vector element times scalar 'a' */
vec_t vec_times(vec_t *A, float a);


/* returns length of passed vector */
float vec_len(vec_t *A);


/* returns vector that is normal (perpendicular) to both A and B */
vec_t vec_normal(vec_t *A, vec_t *B);


/* normalizes vector to a unit vector */
vec_t * vec_normalize(vec_t * A);


/* get quaternion */
quat_t quat(float a, float i, float j, float k);


/* add two quaternions together */
quat_t quat_add(const quat_t *A, const quat_t *B);


/* A * B = C | multiply two quaternions and return the product */
quat_t quat_mlt(const quat_t *A, const quat_t *B);


/* x = AB | returns euclidian 4D dot product of two quaternions */ 
float quat_dot(const quat_t *A, const quat_t *B);


/* B = A* | return conjugated quaternion A */
quat_t quat_cjg(const quat_t *A);


/* C = A * B * A* | make sandwich product of quaternions */
quat_t quat_sandwich(const quat_t *A, const quat_t *B);


/* print quaternion in human readable form */
void quat_print(const quat_t *A);


/* C = C / ||C|| | normalizes quaternion to a unit quaternion */
void quat_normalize(quat_t * A);


/* C = A * B * A* | make fast sandwich product of quaternions */
quat_t quat_sandwich_fast(const quat_t *A, const quat_t *B);


/* returns pointer to the quaternion that is multiplied times a scalar value */
quat_t * quat_times(quat_t * A, float x);


/* Transforms */


/* get rotation quaternion between two unit vectors */
quat_t quat_uvec2uvec(vec_t *A, vec_t * B);


/* rotates vector using rotation quaternion */
vec_t * quat_vecrot(vec_t * vec, quat_t * rotquat);


/* returns one of two quaternions that rotates (v1,v2) into (w1,w2) closest to help_q quat*/
quat_t quat_framerot(vec_t *v1, vec_t *v2, vec_t *w1, vec_t *w2, quat_t * help_q);


/* returns euler angles vector (heading, pitch, bank) */
vec_t quat_quat2euler(quat_t q);

#endif
