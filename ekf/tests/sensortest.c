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
#include <stdlib.h>
#include <fcntl.h>
#include <sys/threads.h>

#include "../sensc.h"
#include <ekflib.h>

#define R2D           57.2957
#define POWER_HIGHEST 50
#define POWER_LOWEST  10

struct
{
	int m[4];
	volatile int run;
	char stack[2048];
} sensortest_common;


static void sensortest_helpPrint(const char *appName)
{
	printf("Usage: %s [throttle]\n", appName);
	printf("  throttle - integer in range [%d,%d], interpreted as percents\n", POWER_LOWEST, POWER_HIGHEST);
}


static void sensortest_writeToAll(const char *val, int sz)
{
	int i = 0;
	for (i = 0; i < 4; i++) {
		write(sensortest_common.m[i], val, sz);
	}
}

static void sensortest_thread(void *arg)
{
	sensor_event_t aEvt, gEvt, mEvt;
	ekf_state_t uavState;

	if (ekf_init() == 0) {
		ekf_run();
	}


	while (sensortest_common.run > 0) {
		usleep(1000 * 10);
		sensc_imuGet(&aEvt, &gEvt, &mEvt);
		ekf_stateGet(&uavState);
		printf("%f %f %f %d %d %d %d %d %d %f %f %f\n", uavState.accelX, uavState.accelY, uavState.accelZ, gEvt.gyro.gyroX, gEvt.gyro.gyroY, gEvt.gyro.gyroZ, mEvt.mag.magX, mEvt.mag.magY, mEvt.mag.magZ, uavState.yaw * R2D, uavState.pitch * R2D, uavState.roll * R2D);
	}

	endthread();
}


int main(int argc, char **argv)
{
	unsigned int power, i;
	char zero[] = "100000";
	char goal[7];

	if (argc <= 1) {
		sensortest_helpPrint(argv[0]);
		return -1;
	}

	power = atoi(argv[1]);
	if (power > POWER_HIGHEST) {
		printf("Throtle of %u too big! Max throttle = %d%%", power, POWER_HIGHEST);
		return -1;
	}
	if (power < POWER_LOWEST) {
		printf("Throtle of %d too low, Min throttle = %d%%\n", power, POWER_LOWEST);
		return -1;
	}

	sensc_init("/dev/sensors");

	sprintf(goal, "1%d000", power);

	printf("WARNING: starting motors on %d%%!\n", power);
	printf("Remove props or ensure safety of the test run! Press [Enter] to continue...\n");
	getchar();

	sensortest_common.m[0] = open(PWM_MOTOR1, O_WRONLY);
	sensortest_common.m[1] = open(PWM_MOTOR2, O_WRONLY);
	sensortest_common.m[2] = open(PWM_MOTOR3, O_WRONLY);
	sensortest_common.m[3] = open(PWM_MOTOR4, O_WRONLY);

	beginthread(sensortest_thread, 4, sensortest_common.stack, sizeof(sensortest_common.stack), NULL);

	sensortest_writeToAll(zero, sizeof(zero));
	sleep(9);
	sensortest_writeToAll(goal, sizeof(goal));
	sleep(7);
	sensortest_writeToAll(zero, sizeof(zero));

	sensortest_common.run = 0;
	threadJoin(0);


	for (i = 0; i < 4; i++) {
		close(sensortest_common.m[i]);
	}

	return 0;
}