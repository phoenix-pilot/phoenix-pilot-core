/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * ekf sensor reading test that aims to measure noise levels in sensor readings
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/threads.h>

#include <sensc.h>
#include <ekflib.h>

#define R2D           57.2957
#define THROTTLE_MAX  50
#define THROTTLE_MIN  10
#define NUM_OF_MOTORS 4

struct {
	int motorFile[NUM_OF_MOTORS];
	volatile int run;
	char stack[2048] __attribute__((aligned(8)));
	char throttle[7];
} sensortest_common;


static void sensortest_helpPrint(const char *appName)
{
	printf("Usage: %s [throttle]\n", appName);
	printf("  throttle - integer in range [%d,%d], interpreted as percents\n", THROTTLE_MIN, THROTTLE_MAX);
}


static int sensortest_motorsWrite(const char *val, int sz)
{
	int i;

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		if (write(sensortest_common.motorFile[i], val, sz) != sz) {
			return -1;
		}
	}

	return 0;
}

static void sensortest_motorsClose(void)
{
	int i;

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		if (sensortest_common.motorFile[i] >= 0) {
			close(sensortest_common.motorFile[i]);
		}
	}
}

static void sensortest_thread(void *arg)
{
	sensor_event_t accelEvt, gyroEvt, magEvt;
	ekf_state_t uavState;

	sensortest_common.run = 1;
	while (sensortest_common.run > 0) {
		usleep(1000 * 10);
		sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		ekf_stateGet(&uavState);
		printf("%f %f %f %d %d %d %d %d %d %f %f %f\n", uavState.accelX, uavState.accelY, uavState.accelZ, gyroEvt.gyro.gyroX, gyroEvt.gyro.gyroY, gyroEvt.gyro.gyroZ, magEvt.mag.magX, magEvt.mag.magY, magEvt.mag.magZ, uavState.yaw * R2D, uavState.pitch * R2D, uavState.roll * R2D);
	}

	endthread();
}


static void sensortest_emergencyShutdown(const char *shutdownMsg, int msgSz)
{
	unsigned int motorShdwn[NUM_OF_MOTORS] = { 0 };
	int i = 0, flag = 0;

	while (flag == 0) {
		flag = 1;
		for (i = 0; i < NUM_OF_MOTORS; i++) {
			if ((motorShdwn[i] == 0) && (write(sensortest_common.motorFile[i], shutdownMsg, msgSz) != msgSz)) {
				flag = 0;
				printf("Emergency shutdown failed, retry...\n");
			}
			else {
				motorShdwn[i] = 1;
			}
		}
	}
}


static int sensortest_motorsProcedure(void)
{
	const char pwmInit[] = "100000";

	if (sensortest_motorsWrite(pwmInit, sizeof(pwmInit)) < 0) {
		sensortest_emergencyShutdown(pwmInit, sizeof(pwmInit));
		return -1;
	}

	sleep(3);

	if (sensortest_motorsWrite(sensortest_common.throttle, sizeof(sensortest_common.throttle)) < 0) {
		sensortest_emergencyShutdown(pwmInit, sizeof(pwmInit));
		return -1;
	}

	sleep(7);

	if (sensortest_motorsWrite(pwmInit, sizeof(pwmInit)) < 0) {
		sensortest_emergencyShutdown(pwmInit, sizeof(pwmInit));
		return -1;
	}

	return 0;
}


int main(int argc, char **argv)
{
	unsigned int throttle, i, flag = 0;
	int ret;

	if (argc != 2) {
		sensortest_helpPrint(argv[0]);
		return EXIT_FAILURE;
	}

	throttle = atoi(argv[1]);
	if (throttle > THROTTLE_MAX) {
		printf("Throtle of %u too big! Max throttle = %d%%", throttle, THROTTLE_MAX);
		return EXIT_FAILURE;
	}
	if (throttle < THROTTLE_MIN) {
		printf("Throtle of %u too low, Min throttle = %d%%\n", throttle, THROTTLE_MIN);
		return EXIT_FAILURE;
	}

	if (sensc_init("/dev/sensors", false) < 0) {
		printf("cannot initialize sensor client\n");
		return EXIT_FAILURE;
	}

	if (snprintf(sensortest_common.throttle, sizeof(sensortest_common.throttle), "1%d000", throttle) < 0) {
		printf("cannot create correct throttle input\n");
		return EXIT_FAILURE;
	}

	printf("WARNING: starting motors on %d%%!\n", throttle);
	printf("Remove props or ensure safety of the test run! Press [Enter] to continue...\n");
	getchar();

	sensortest_common.motorFile[0] = open(PWM_MOTOR1, O_WRONLY);
	sensortest_common.motorFile[1] = open(PWM_MOTOR2, O_WRONLY);
	sensortest_common.motorFile[2] = open(PWM_MOTOR3, O_WRONLY);
	sensortest_common.motorFile[3] = open(PWM_MOTOR4, O_WRONLY);

	for (i = 0; i < NUM_OF_MOTORS; i++) {
		if (sensortest_common.motorFile[i] < 0) {
			printf("failed at opening %u-th motor descriptor\n", i);
			flag = 1;
		}
	}
	if (flag != 0) {
		sensortest_motorsClose();
		return EXIT_FAILURE;
	}

	if (ekf_init() != 0) {
		printf("Cannot initialize ekf\n");
		sensortest_motorsClose();
		return EXIT_FAILURE;
	}

	if (ekf_run() < 0) {
		printf("Cannot start ekf\n");
		sensortest_motorsClose();
		return EXIT_FAILURE;
	}

	if (beginthread(sensortest_thread, 4, sensortest_common.stack, sizeof(sensortest_common.stack), NULL) >= 0) {
		/* data acquisition */
		ret = sensortest_motorsProcedure();

		sensortest_common.run = 0;
		threadJoin(-1, 0);
	}
	else {
		printf("Failed to start data acquisition thread\n");
		ret = -1;
	}

	ekf_done();

	sensortest_motorsClose();

	return (ret == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
