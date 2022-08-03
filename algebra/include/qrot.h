/*
 * Phoenix-Pilot
 *
 * qrot - quaternion/vector rotation library - header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __PHOENIX_QROT_H__
#define __PHOENIX_QROT_H__

#include <vec.h>
#include <quat.h>

/* calculate quaternion q that rotates v1 into v2, assumed len(v1) == len(v2) */
extern void quat_uvec2uvec(const vec_t *v1, const vec_t *v2, quat_t *q);


/* rotates vector using rotation quaternion */
extern void quat_vecRot(vec_t *vec, const quat_t *qRot);


/* calculates quaternion res (closest to help_q), that rotates frame of reference (v1, v2) into (w1, w2) */
extern void quat_frameRot(const vec_t *v1, const vec_t *v2, const vec_t *w1, const vec_t *w2, quat_t *res, const quat_t *help_q);

#endif
