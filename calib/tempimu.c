/*
 * Phoenix-Pilot
 *
 * IMU temperature compensation
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <sensc.h>
#include <vec.h>
#include <statistics.h>
#include <libsensors.h>

#include "calibtool.h"
#include "linlsm.h"


#define DATA_POINTS                  600   /* How many data points linear fittting uses */
#define DATA_POINT_SAMPLES           100   /* How many times each data point is sampled */
#define DATA_POINT_SAMPLES_PERIOD_US 10000 /* Sampling perion [us] during data point averaging */

#define MIN_TEMP_DIFF 3

struct {
	calib_data_t data;
} tempimu_common;


/* Averages accel reading over `samples` samples with `usDelay` period. Returns 0 on success */
static int tempimu_imuAvg(unsigned int samples, time_t usDelay, sensor_event_t *accOut, sensor_event_t *gyrOut)
{
	sensor_event_t accelEvt, gyrEvt, magEvt;
	int64_t acc[3], gyr[3], tempAcc, tempGyr;
	int i;

	memset(acc, 0, sizeof(int64_t) * 3);
	memset(gyr, 0, sizeof(int64_t) * 3);
	tempAcc = 0;
	tempGyr = 0;

	for (i = 0; i < samples; i++) {
		if (sensc_imuGet(&accelEvt, &gyrEvt, &magEvt) != 0) {
			fprintf(stderr, "%s: imuGet fail\n", TEMPIMU_TAG);
			return -1;
		}

		acc[0] += accelEvt.accels.accelX;
		acc[1] += accelEvt.accels.accelY;
		acc[2] += accelEvt.accels.accelZ;
		tempAcc += accelEvt.accels.temp;

		gyr[0] += gyrEvt.gyro.gyroX;
		gyr[1] += gyrEvt.gyro.gyroY;
		gyr[2] += gyrEvt.gyro.gyroZ;
		tempGyr += gyrEvt.gyro.temp;

		usleep(usDelay);
	}

	accOut->accels.accelX = (float)acc[0] / samples;
	accOut->accels.accelY = (float)acc[1] / samples;
	accOut->accels.accelZ = (float)acc[2] / samples;
	accOut->accels.temp = (float)tempAcc / samples;

	gyrOut->gyro.gyroX = (float)gyr[0] / samples;
	gyrOut->gyro.gyroY = (float)gyr[1] / samples;
	gyrOut->gyro.gyroZ = (float)gyr[2] / samples;
	gyrOut->gyro.temp = (float)tempGyr / samples;

	return 0;
}


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *tempimu_dataGet(void)
{
	return &tempimu_common.data;
}


static const char *tempimu_help(void)
{
	return "Temperature-IMU calibration\n";
}


static int tempimu_write(FILE *file)
{
	int axis;

	/* reference temperature write */
	fprintf(file, "rt %f\n", tempimu_common.data.params.tempimu.refTemp);

	/* accelerometer coefficients */
	for (axis = 0; axis < 3; axis++) {
		fprintf(file, "a%d %f\n", axis, tempimu_common.data.params.tempimu.alfaAcc[axis]);
	}

	/* gyro coefficients */
	for (axis = 0; axis < 3; axis++) {
		fprintf(file, "g%d %f\n", axis, tempimu_common.data.params.tempimu.alfaGyr[axis]);
	}

	return EOK;
}


static int tempimu_done(void)
{
	sensc_deinit();

	return EOK;
}


int tempimu_run(void)
{
	calib_data_t *calib = &tempimu_common.data;
	sensor_event_t accelEvt, gyrEvt;
	linlsm_t lsmAcc[3];
	linlsm_t lsmGyr[3];
	float refTempStart, refTempEnd;
	int i;

	for (i = 0; i < 3; i++) {
		linlsm_init(&lsmAcc[i]);
		linlsm_init(&lsmGyr[i]);
	}

	/* Averaging start/end temperature 2x more than usual samples */
	if (tempimu_imuAvg(DATA_POINT_SAMPLES * 2, DATA_POINT_SAMPLES_PERIOD_US, &accelEvt, &gyrEvt) != 0) {
		fprintf(stderr, "%s: failed to average temp\n", TEMPIMU_TAG);
		return -1;
	}
	refTempStart = (float)accelEvt.accels.temp / 1000; /* from millikelvins to kelvins */

	fprintf(stdout, "Reference temperature is %.1f K\n", refTempStart);
	fprintf(stdout, "Keep drone still for 10 minutes. Collecting data...\n");

	for (i = 0; i < DATA_POINTS; i++) {
		/*  */
		if (tempimu_imuAvg(DATA_POINT_SAMPLES, DATA_POINT_SAMPLES_PERIOD_US, &accelEvt, &gyrEvt) < 0) {
			fprintf(stderr, "%s: imuGet fail\n", TEMPIMU_TAG);
			return -1;
		}

		linlsm_update(&lsmAcc[0], (float)accelEvt.accels.temp / 1000, accelEvt.accels.accelX);
		linlsm_update(&lsmAcc[1], (float)accelEvt.accels.temp / 1000, accelEvt.accels.accelY);
		linlsm_update(&lsmAcc[2], (float)accelEvt.accels.temp / 1000, accelEvt.accels.accelZ);

		linlsm_update(&lsmGyr[0], (float)gyrEvt.gyro.temp / 1000, gyrEvt.gyro.gyroX);
		linlsm_update(&lsmGyr[1], (float)gyrEvt.gyro.temp / 1000, gyrEvt.gyro.gyroY);
		linlsm_update(&lsmGyr[2], (float)gyrEvt.gyro.temp / 1000, gyrEvt.gyro.gyroZ);
	}

	/* Averaging start/end temperature 2x more than usual samples */
	if (tempimu_imuAvg(DATA_POINT_SAMPLES_PERIOD_US * 2, DATA_POINT_SAMPLES_PERIOD_US, &accelEvt, &gyrEvt) != 0) {
		fprintf(stderr, "%s: failed to average temp\n", TEMPIMU_TAG);
		return -1;
	}
	refTempEnd = (float)accelEvt.accels.temp / 1000; /* from millikelvins to kelvins */

	if (fabs(refTempEnd - refTempStart) < MIN_TEMP_DIFF) {
		fprintf(stderr, "%s: insufficient temperature difference from %d to %d. Must be at least %d\n", TEMPIMU_TAG, (int)refTempStart, (int)refTempEnd, MIN_TEMP_DIFF);
		return -1;
	}

	/* Calibration finished, checking parmateres integrity. Average temperature will be used as refTemp */
	for (i = 0; i < 3; i++) {
		linlsm_get(&lsmAcc[i], &calib->params.tempimu.alfaAcc[i], NULL, NULL);
		linlsm_get(&lsmGyr[i], &calib->params.tempimu.alfaGyr[i], NULL, NULL);
	}
	calib->params.tempimu.refTemp = (refTempStart + refTempEnd) / 2;

	printf("%s params:\n", TEMPIMU_TAG);
	printf("acc: %f %f %f\n", calib->params.tempimu.alfaAcc[0], calib->params.tempimu.alfaAcc[1], calib->params.tempimu.alfaAcc[2]);
	printf("gyr: %f %f %f\n", calib->params.tempimu.alfaGyr[0], calib->params.tempimu.alfaGyr[1], calib->params.tempimu.alfaGyr[2]);
	printf("reftemp: %.1f\n", calib->params.tempimu.refTemp);

	return EOK;
}


static int tempimu_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, TEMPIMU_CALIB_DEPENDENCY, SENSC_INIT_IMU) < 0) {
		return -1;
	}

	return EOK;
}


__attribute__((constructor(102))) static void tempimu_register(void)
{
	static calib_ops_t cal = {
		.name = TEMPIMU_TAG,
		.init = tempimu_init,
		.run = tempimu_run,
		.done = tempimu_done,
		.write = tempimu_write,
		.help = tempimu_help,
		.dataGet = tempimu_dataGet
	};

	calib_register(&cal);

	tempimu_common.data.type = typeTempimu;
}
