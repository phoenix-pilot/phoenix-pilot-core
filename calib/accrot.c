/*
 * Phoenix-Pilot
 *
 * Drone accelerometer initial rotation calibration
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>

#include <libsensors.h>

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <calib.h>

#include "calibtool.h"

#define AVG_WAIT        2000            /* time in microseconds between taking samples to average */
#define AVG_SAMPLES     1500            /* number of samples gathered for averaging */
#define MIN_TILT_COSINE (M_SQRT2 / 2.f) /* minimal cosine of tilt angle between down and front measurement (45 degrees) */

struct {
	calib_data_t data;
} accrot_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *accrot_dataGet(void)
{
	return &accrot_common.data;
}


const char *accrot_help(void)
{
	return "Accelerometer initial rotation calibration\n";
}


static int accrot_write(FILE *file)
{
	static const char base[] = "accq";
	const quat_t *q = &accrot_common.data.params.accrot.frameQ;

	fprintf(file, "%s0 %f\n%s1 %f\n%s2 %f\n%s3 %f\n", base, q->a, base, q->i, base, q->j, base, q->k);

	return 0;
}


/* Get average accelerometer reading over `n` samples */
static void accrot_accAvg(vec_t *out, unsigned int n)
{
	unsigned int i;
	sensor_event_t accelEvt, gyroEvt, magEvt;

	out->x = out->y = out->z = 0;
	for (i = 0; i < n; i++) {
		sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		out->x += accelEvt.accels.accelX;
		out->y += accelEvt.accels.accelY;
		out->z += accelEvt.accels.accelZ;
		usleep(AVG_WAIT);
	}
	vec_times(out, 1. / n);
}

/*
* Calibration procedure that acquires initial accelerometer rotation.
* Quaternion is acquired by:
* - measuring up acceleration on flat surface
* - measuring front acceleration by tilting drone front
* - calculating left direction with cross product of above
* - calculating frame rotation quaternion that rotates measured vectors into ideal ones
*/
static int accrot_run(void)
{
	static const vec_t nedZ = { .x = 0, .y = 0, .z = 1 }; /* z versor of NED frame of reference */
	static const vec_t nedY = { .x = 0, .y = 1, .z = 0 }; /* y versor of NED frame of reference */
	static const quat_t idenQ = { .a = 1, .i = 0, .j = 0, .k = 0 };

	vec_t bodyZ, accX, bodyY = { 0 };
	quat_t q;

	/*
	* Body frame placed on flat surface should provide a measurement of acceleration pointing upward.
	* Using NED frame of reference means that earth acceleration is parallel to NED z versor, but of opposite direction.
	* Taking negative earth acceleration as NED z versor.
	*/
	printf("Place drone on a flat surface and press [Enter]\n");
	getchar();
	fflush(stdin);

	sleep(1); /* Sleep for any vibration to disperse */
	accrot_accAvg(&bodyZ, 1500);

	vec_times(&bodyZ, -1);

	/*
	* Body frame pointing upward (+90 degree pitch angle) experience earth acceleration that is:
	* - parallel to NED x axis versor
	* - of common direction to NED x axis versor.
	*/
	printf("Tilt drone up (positive pitch) and press [Enter]\n");
	getchar();
	fflush(stdin);

	sleep(1); /* Sleep for any vibration to disperse */
	accrot_accAvg(&accX, 1500);

	/* Non-allignment check by calculating cosine between measured accelerations using dot product */
	vec_normalize(&bodyZ);
	vec_normalize(&accX);
	if (vec_dot(&bodyZ, &accX) > MIN_TILT_COSINE) {
		printf("Calibration failed. Too small angle of front tilt\n");
		return -1;
	}

	/* Calculate cross vector to acquire frame of reference with orthogonal axis */
	vec_cross(&bodyZ, &accX, &bodyY);
	vec_normalize(&bodyY);

	/* Calculate rotation */
	quat_frameRot(&nedZ, &nedY, &bodyZ, &bodyY, &q, &idenQ);

	/* Quaternion validity check */
	if ((1 - quat_len(&q)) >= ACCROT_QUAT_ERR) {
		printf("Too big error in quaternion. Calibration aborted\n");
		return -1;
	}

	accrot_common.data.params.accrot.frameQ = q;

	return EOK;
}


static int accrot_done(void)
{
	sensc_deinit();

	return EOK;
}


static int accrot_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, false) < 0) {
		return -ENXIO;
	}

	return EOK;
}


__attribute__((constructor(102))) static void accrot_register(void)
{
	static calib_ops_t cal = {
		.name = ACCROT_TAG,
		.init = accrot_init,
		.run = accrot_run,
		.done = accrot_done,
		.write = accrot_write,
		.help = accrot_help,
		.dataGet = accrot_dataGet
	};

	calib_register(&cal);

	accrot_common.data.type = typeAccrot;
}
