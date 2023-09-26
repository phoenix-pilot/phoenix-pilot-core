/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * ekf simple client
 *
 * Copyright 2022 Phoenix Systems
 * Authors: Mateusz Niewiadomski, Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include <ekflib.h>

#include <vec.h>
#include <quat.h>


#define EARTH_G 9.80665


enum printMode { prntVersor, prntAtt, printPos, printAcc, silent };


volatile int devekf_run;


static void *keyboardHandler(void *args)
{
	while (getchar() != 'q');

	devekf_run = 0;

	return NULL;
}


/* Printing versors of local NED frame of reference in global ENU frame of reference */
static void printUavVersors(ekf_state_t *uavState)
{
	const quat_t ned2enu = { .a = 0, .i = -0.7071f, .j = -0.7071f, .k = 0 };

	vec_t pos = { .x = uavState->enuX, .y = uavState->enuY, .z = uavState->enuZ };
	vec_t versx = { .x = 1, .y = 0, .z = 0 }, versy = { .x = 0, .y = 1, .z = 0 }, versz = { .x = 0, .y = 0, .z = 1 };
	quat_t q = { .a = uavState->q0, .i = uavState->q1, .j = uavState->q2, .k = uavState->q3 };

	/* Rotate versors from local to global frame of reference (all done in NED frame of reference) */
	quat_vecRot(&versx, &q);
	quat_vecRot(&versy, &q);
	quat_vecRot(&versz, &q);

	/* Rotate versors from NED to ENU frame of reference */
	quat_vecRot(&versx, &ned2enu);
	quat_vecRot(&versy, &ned2enu);
	quat_vecRot(&versz, &ned2enu);

	/* Print NED versors in ENU frame of reference */
	printf(
		"%4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f %4.3f ",
		versx.x, versx.y, versx.z, versy.x, versy.y, versy.z, versz.x, versz.y, versz.z);

	/* Position is already in ENU, just prinitng it */
	printf("%6.3f %6.3f %6.3f ", pos.x, pos.y, pos.z);
	printf("\n");
}


static inline void printUavAtt(ekf_state_t *uavState)
{
	printf("YPR: %5.2f %5.2f %5.2f YPR_DOT %6.3f %6.3f %6.3f NED/DOT: %5.2f %6.3f\n", uavState->yaw * 180 / 3.1415, uavState->pitch * 180 / 3.1415, uavState->roll * 180 / 3.1415, uavState->yawDot, uavState->pitchDot, uavState->rollDot, uavState->enuZ, uavState->veloZ);
}


static inline void printUavPos(ekf_state_t *uavState)
{
	printf("A %3.0f %4.1f %4.1f ", uavState->yaw * 180 / 3.1415, uavState->pitch * 180 / 3.1415, uavState->roll * 180 / 3.1415);
	printf("R %3.1f %4.2f %4.2f ", uavState->yawDot, uavState->pitchDot, uavState->rollDot);
	printf("P %6.2f %6.2f %6.2f ", uavState->enuX, uavState->enuY, uavState->enuZ);
	printf("V %6.2f %6.2f %6.2f ", uavState->veloX, uavState->veloY, uavState->veloZ);
	printf("\n");
}

/* Printing net acceleration in NED frame of reference, with EARTH_G removed */
static inline void printUavAcc(ekf_state_t *uavState)
{
	quat_t qState = { .a = uavState->q0, .i = uavState->q1, .j = uavState->q2, .k = uavState->q3 };
	vec_t aEarth, aBody = { .x = uavState->accelX, .y = uavState->accelY, .z = uavState->accelZ };

	aEarth = aBody;
	/* velocity estimation */
	quat_vecRot(&aEarth, &qState);
	aEarth.z += EARTH_G;

	printf("YPR: %.1f %.2f %.2f E_ACC %.3f %.3f %.3f\n", uavState->yaw, uavState->pitch, uavState->roll, aEarth.x, aEarth.y, aEarth.z);
}


static int handleUavStatus(int uavStatus)
{
	int res = 0;

	if ((uavStatus & EKF_RUNNING) == 0) {
		printf("EKF stopped working\n");
		res = -1;
	}

	if ((uavStatus & EKF_ERROR) != 0) {
		printf("EKF error occurred\n");
		res = -1;
	}
	else if ((uavStatus & EKF_MEAS_EOF) != 0) {
		printf("EKF: meas module encountered EOF\n");
		res = -1;
	}

	return res;
}


int main(int argc, char **argv)
{
	ekf_state_t uavState;
	enum printMode mode;
	pthread_t keyboardHandlerTID;
	pthread_attr_t threadAttr;

	if (argc != 2) {
		printf("Wrong arguments count!\n");
		return EXIT_FAILURE;
	}

	switch (argv[1][0]) {
		case '1':
			mode = prntAtt;
			break;
		case '2':
			mode = printPos;
			break;
		case '3':
			mode = printAcc;
			break;
		case '4':
			mode = silent;
			break;
		default:
			mode = prntVersor;
			break;
	}

	devekf_run = 1;

	/* Changing output color to red and back to default */
	printf("\033[31m");
	printf("Press 'q' and confirm with enter to exit\n");
	printf("\033[39m");


	if (ekf_init(EKF_INIT_LOG_SRC) != 0) {
		fprintf(stderr, "devekf: error during ekf init\n");
		return EXIT_FAILURE;
	}

	if (ekf_run() != 0) {
		fprintf(stderr, "devekf: cannot start ekf\n");
		ekf_done();
		return EXIT_FAILURE;
	}

	pthread_attr_init(&threadAttr);

	if (pthread_create(&keyboardHandlerTID, &threadAttr, keyboardHandler, NULL) != 0) {
		fprintf(stderr, "devekf: cannot run keyboard input handler\n");
		pthread_attr_destroy(&threadAttr);
		ekf_stop();
		ekf_done();
		return EXIT_FAILURE;
	}

	/* This is testing app that may be used to present EKF capabilities and stability, thus no run time is set */
	while (devekf_run == 1) {
		usleep(1000 * 100);
		ekf_stateGet(&uavState);

		if (handleUavStatus(uavState.status) != 0) {
			pthread_cancel(keyboardHandlerTID);
			break;
		}

		if (mode == prntVersor) {
			printUavVersors(&uavState);
		}
		if (mode == prntAtt) {
			printUavAtt(&uavState);
		}
		if (mode == printPos) {
			printUavPos(&uavState);
		}
		if (mode == printAcc) {
			printUavAcc(&uavState);
		}
		/* Mode = silent do not invoke any printing function */
	}

	pthread_join(keyboardHandlerTID, NULL);

	ekf_stop();
	ekf_done();

	pthread_attr_destroy(&threadAttr);

	return EXIT_SUCCESS;
}
