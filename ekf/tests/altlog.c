/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * altitude logging utility
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>

#include <ekflib.h>
#include <sensc.h>


static void altlog_help(void)
{
	printf("Usage: altlog dur [tstep]\n");
	printf("  dur - logging time in full seconds\n");
	printf("  tstep - interval between samples in milliseconds, 1000 default\n");
}


static float altlog_timeSec(void)
{
	time_t now;

	gettime(&now, NULL);

	return (float)now / 1000000;
}


int main(int argc, char **argv)
{
	ekf_state_t uavState = { 0 };
	FILE *file;
	time_t now, step;
	sensor_event_t baroEvt;
	char buf[128];
	float delta, t0, tmax;

	step = 1000 * 1000;
	switch (argc) {
		case 3:
			step = atoi(argv[2]) * 1000;
		case 2:
			if (strcmp("-h", argv[1]) == 0) {
				altlog_help();
				return EXIT_SUCCESS;
			}
			tmax = (float)atoi(argv[1]);
			break;
		default:
			altlog_help();
			return EXIT_FAILURE;
			break;
	}
	fprintf(stdout, "Logging for %.1f seconds with %llims step\n", tmax, step / 1000);

	if (sensc_init("/dev/sensors", true) < 0) {
		fprintf(stderr, "Cannot initialize sensor client\n");
		return EXIT_FAILURE;
	}

	file = fopen("/etc/altlog.txt", "w");
	if (file == NULL) {
		sensc_deinit();
		fprintf(stderr, "Cannot open '/etc/altlog.txt' logfile\n");
		return EXIT_FAILURE;
	}

	if (ekf_init() != 0) {
		fprintf(stderr, "Cannot initialize ekf!\n");
		return EXIT_FAILURE;
	}
	ekf_run();

	t0 = altlog_timeSec();
	do {
		ekf_stateGet(&uavState);
		sensc_baroGet(&baroEvt);

		gettime(&now, NULL);
		delta = altlog_timeSec() - t0;

		sprintf(buf, "%.3f %f %i\n", delta, uavState.enuZ, baroEvt.baro.pressure);
		fprintf(file, "%s", buf);
		fprintf(stdout, "%s", buf);

		usleep(step);
	} while (delta < tmax);

	fclose(file);
	ekf_done();
	sensc_deinit();

	return EXIT_SUCCESS;
}
