/*
 * Phoenix-Pilot
 *
 * Calibration library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _LIBCALIB_CALIB_H_
#define _LIBCALIB_CALIB_H_

#include <matrix.h>
#include <quat.h>

#define CALIB_PATH    "/etc/calib.conf"
#define NUM_OF_MOTORS 4

#define MAGMOT_TAG             "magmot"
#define MAGMOT_PARAMS          36
#define MAGMOT_CUTOFF_THROTTLE 0.3 /* minimal throttle value calibration is made with */

#define MAGIRON_TAG     "magiron"
#define MAGIRON_PARAMS  12
#define CHAR_SOFTIRON   's'
#define CHAR_HARDIRON   'h'
#define SOFTCAL_ROWSPAN 3
#define SOFTCAL_COLSPAN 3
#define HARDCAL_ROWSPAN 3
#define HARDCAL_COLSPAN 1

#define ACCORTH_TAG        "accorth"
#define ACCORTH_PARAMS     20
#define ACC_CHAR_ORTHO     'o'
#define ACC_CHAR_OFFSET    'h'
#define ACC_CHAR_QUAT      'q'
#define ACC_CHAR_SWAP      's'
#define ACC_CHAR_SWAP_SIGN 's'
#define ACC_CHAR_SWAP_ORDR 'o'
#define ACC_ORTHO_ROWSPAN  3
#define ACC_ORTHO_COLSPAN  3
#define ACC_OFFSET_ROWSPAN 3
#define ACC_OFFSET_COLSPAN 1
#define ACC_QUAT_ERR       0.001f

#define MOTLIN_TAG    "motlin"
#define MOTLIN_PARAMS 8

/* clang-format off */
typedef enum { typeMagmot = 0, typeMagiron, typeMotlin, typeAccorth } calibType_t;


typedef enum { accSwapXYZ = 0, accSwapXZY, accSwapYXZ, accSwapYZX, accSwapZXY, accSwapZYX } accSwap_t;
/* clang-format on */

typedef struct {
	calibType_t type;
	union {
		struct {
			matrix_t softCal; /* 3x3 matrix for soft iron calib. parameters. */
			matrix_t hardCal; /* 3x1 matrix for hard iron calibration */
		} magiron;

		struct {
			matrix_t ortho;      /* 3x3 nonorthogonality parameters matrix */
			matrix_t offset;     /* 3x1 measurement offset matrix */
			quat_t frameQ;       /* initial rotation quaternion of accelerometer in relation to body frame */
			accSwap_t swapOrder; /* axis swap order; according to `accSwap_t` enum */
			int axisInv[3];      /* x,y,z axis inversion flags. 1 means that axis should be inverted after swapping is performed */
		} accorth;

		struct {
			float motorEq[NUM_OF_MOTORS][3][3]; /* motorEq[motorId 0/1/2...NUM_OF_MOTORS][axisId x/y/z][equation_param a/b/c] */
		} magmot;

		struct {
			float motorEq[NUM_OF_MOTORS][2]; /* motorEq[motorId 0/1/2...NUM_OF_MOTORS][equation_parameter a/b] */
		} motlin;
	} params;
} calib_data_t;


/*
* Read calibration file pointed by 'path' searching for calibration named `tag` and saving its content to 'cal'.
* If 'path' does not point to a file default values are written and 0 is returned (success).
*/
int calib_readFile(const char *path, calibType_t type, calib_data_t *cal);


/* Deallocates all memory used by 'cal' */
void calib_free(calib_data_t *cal);


#endif
