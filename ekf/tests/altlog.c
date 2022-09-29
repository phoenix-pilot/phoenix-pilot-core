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

/* TODO: this tool should be placed outside ekf directory */

struct {
	float tmax;
	time_t step;
	bool init;
} altlog_common;


static void altlog_help(const char *progname)
{
	printf("Usage: %s [options]\n options:\n", progname);
	printf("  -t,\tlogging time in full seconds\n");
	printf("  -s,\tsampling interval in milliseconds\n");
}


static inline float altlog_timeSec(void)
{
	time_t now;

	gettime(&now, NULL);

	return (float)now / 1000000.0;
}


/* Parsing arguments Returns 0 on success, -1 when parsing failed */
static int altlog_parseArgs(int argc, char **argv)
{
	int c;

	altlog_common.step = 0;
	altlog_common.tmax = 0;
	altlog_common.init = false;

	if (argc < 2) {
		altlog_help(argv[0]);
		return -1;
	}

	while ((c = getopt(argc, argv, "ht:s:")) != -1) {
		switch (c) {
			case 'h': /* help message */
				altlog_help(argv[0]);
				return 0;

			case 't': /* time set */
				altlog_common.tmax = atof(optarg);
				break;

			case 's': /* samppling interval set */
				altlog_common.step = atoi(optarg);
				break;

			default: /* any other case is an error */
				altlog_help(argv[0]);
				return -1;
		}
	}

	if (altlog_common.step == 0 || altlog_common.tmax == 0) {
		return -1;
	}

	altlog_common.step *= 1000; /* make it microseconds for usleep */
	altlog_common.init = true;

	return 0;
}


int main(int argc, char **argv)
{
	ekf_state_t uavState = { 0 };
	FILE *file;
	sensor_event_t baroEvt;
	char buf[128];
	float delta, t0;
	int parse;

	parse = altlog_parseArgs(argc, argv);
	if (!altlog_common.init) {
		/* Check if parsing failed, or only help was printed */
		return (parse == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
	}

	fprintf(stdout, "Logging for %.1f seconds with %llims step\n", altlog_common.tmax, altlog_common.step / 1000);

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
		sensc_deinit();
		fclose(file);
		return EXIT_FAILURE;
	}
	ekf_run();

	t0 = altlog_timeSec();
	/* Main logging loop */
	do {
		ekf_stateGet(&uavState);
		sensc_baroGet(&baroEvt);

		delta = altlog_timeSec() - t0;

		sprintf(buf, "%.3f %f %i\n", delta, uavState.enuZ, baroEvt.baro.pressure);
		fprintf(file, "%s", buf);
		fprintf(stdout, "%s", buf);

		usleep(altlog_common.step);
	} while (delta < altlog_common.tmax);

	fclose(file);
	ekf_done();
	sensc_deinit();

	return EXIT_SUCCESS;
}