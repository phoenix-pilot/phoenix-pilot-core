/*
 * Phoenix-Pilot
 *
 * pilot-core utilities
 * 
 * sensor logging utility
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

#include <sensc.h>

struct {
	time_t diffMax; /* Maximum time span on logging process (in microseconds) */
	time_t step;    /* Step between logs (in microseconds) */
	bool raw;       /* use corrections in measurements */

	/* device log selection flags */
	struct {
		bool accel;
		bool gyro;
		bool mag;
		bool baro;
		bool gps;
	} flags;
} senslog_common;


static void senslog_help(const char *progname)
{
	printf("Usage: %s [options]\n options:\n", progname);
	printf("  -t,\tlogging time in full seconds\n");
	printf("  -s,\tsampling interval in milliseconds\n");
	printf("  -d [agmbp]\tdevices to be logged as string\n");
	printf("     a - accelerometer\n");
	printf("     g - gyroscope\n");
	printf("     m - magnetometer\n");
	printf("     b - barometer\n");
	printf("     p - gps\n");
	printf("  -r raw mode, don`t use calib.conf corrections\n");
}

/* Parse devices log selection options */
static int senslog_parseDevices(const char *devs)
{
	bool err = false;

	if (devs == NULL || strlen(devs) == 0) {
		return -1;
	}

	senslog_common.flags.accel = false;
	senslog_common.flags.gyro = false;
	senslog_common.flags.mag = false;
	senslog_common.flags.baro = false;
	senslog_common.flags.gps = false;

	while (*devs != '\0' && !err) {
		switch (*devs) {
			case 'a':
				senslog_common.flags.accel = true;
				break;

			case 'g':
				senslog_common.flags.gyro = true;
				break;

			case 'm':
				senslog_common.flags.mag = true;
				break;

			case 'b':
				senslog_common.flags.baro = true;
				break;

			case 'p':
				senslog_common.flags.gps = true;
				break;

			default:
				fprintf(stderr, "Unknown sensor log option: '%c'\n", *devs);
				err = true;
				break;
		}

		devs++;
	}

	if (err) {
		return -1;
	}

	return 0;
}


/* 
* Parsing arguments. Returns:
* 0 if all variables were successfully initialized, and application is good to go.
* -1 if an error occurred during parsing
* 1 if only help was printed and application shall exit.
 */
static int senslog_parseArgs(int argc, char **argv)
{
	int c;
	const char *devs = NULL;

	senslog_common.step = 0;
	senslog_common.diffMax = 0;
	senslog_common.raw = false;

	if (argc < 2) {
		senslog_help(argv[0]);
		return -1;
	}

	while ((c = getopt(argc, argv, "ht:s:d:r")) != -1) {
		switch (c) {
			case 'h':
				senslog_help(argv[0]);
				return 1; /* only help message printed, returning +1 */

			case 't': /* time set */
				senslog_common.diffMax = atoi(optarg) * 1000000;
				break;

			case 's': /* samppling interval set */
				senslog_common.step = atoi(optarg) * 1000;
				break;

			case 'd': /* devices to log */
				devs = optarg;
				break;

			case 'r': /* do not use corrections, raw mode */
				senslog_common.raw = true;
				break;

			default: /* any other case is an error */
				senslog_help(argv[0]);
				return -1;
		}
	}

	if (senslog_common.step == 0 || senslog_common.diffMax == 0 || senslog_parseDevices(devs) < 0) {
		fprintf(stderr, "Invalid arguments!\n");
		return -1;
	}

	return 0;
}


int main(int argc, char **argv)
{
	FILE *file;
	sensor_event_t accelEvt, gyroEvt, magEvt, baroEvt, gpsEvt;
	char buf[256];
	time_t t0, now, diff;
	int res, cnt;

	res = senslog_parseArgs(argc, argv);
	if (res != 0) {
		return (res < 0) ? EXIT_FAILURE : EXIT_SUCCESS;
	}

	fprintf(stdout, "Logging for %lli seconds with %llims step\n", senslog_common.diffMax / 1000000, senslog_common.step / 1000);

	/* initialization of sensor client with selectable corrections (raw == true means corrections disabled) */
	if (sensc_init("/dev/sensors", !senslog_common.raw) < 0) {
		fprintf(stderr, "Cannot initialize sensor client\n");
		return EXIT_FAILURE;
	}

	file = fopen("/etc/senslog.txt", "w");
	if (file == NULL) {
		sensc_deinit();
		fprintf(stderr, "Cannot open '/etc/altlog.txt' logfile\n");
		return EXIT_FAILURE;
	}

	gettime(&t0, NULL);
	now = t0;
	diff = 0;

	/* Main logging loop */
	while (diff < senslog_common.diffMax) {
		gettime(&now, NULL);
		diff = now - t0;

		if (senslog_common.flags.accel || senslog_common.flags.gyro || senslog_common.flags.mag) {
			sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		}

		if (senslog_common.flags.baro) {
			sensc_baroGet(&baroEvt);
		}

		if (senslog_common.flags.gps) {
			sensc_gpsGet(&gpsEvt);
		}
		/* Above ifs to be expanded with new sensors */

		memset(buf, 0, sizeof(buf));
		cnt = 0;

		/* printing time as a fake float value */
		res = sprintf(&buf[cnt], "%lli.%06lli ", diff / 1000000, (diff - 1000000 * (diff / 1000000))); /* Max 28 characters */
		cnt = (res < 0) ? -1 : cnt + res;

		if (senslog_common.flags.accel && cnt >= 0) {
			res = sprintf(&buf[cnt], "%d %d %d ", accelEvt.accels.accelX, accelEvt.accels.accelY, accelEvt.accels.accelZ); /* Max 36 characters */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		if (senslog_common.flags.gyro && cnt >= 0) {
			res = sprintf(&buf[cnt], "%d %d %d ", gyroEvt.gyro.gyroX, gyroEvt.gyro.gyroY, gyroEvt.gyro.gyroZ); /* Max 36 characters */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		if (senslog_common.flags.mag && cnt >= 0) {
			res = sprintf(&buf[cnt], "%d %d %d ", magEvt.mag.magX, magEvt.mag.magY, magEvt.mag.magZ); /* Max 36 characters */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		if (senslog_common.flags.baro && cnt >= 0) {
			res = sprintf(&buf[cnt], "%d %d ", baroEvt.baro.pressure, baroEvt.baro.temp); /* Max 24 characters */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		if (senslog_common.flags.gps && cnt >= 0) {
			res = sprintf(&buf[cnt], "%lld %lld %u %u ", gpsEvt.gps.lat, gpsEvt.gps.lon, gpsEvt.gps.satsNb, gpsEvt.gps.fix); /* Max 62 characters */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		if (cnt >= 0) {
			res = sprintf(&buf[cnt], "\n"); /* Max 2 characters ("\n\0") */
			cnt = (res < 0) ? -1 : cnt + res;
		}

		/*
		* Worst-case scenario for maximum characters printed is 224 now. Change the `buf` size accordingly when adding new sensors!
		*/

		if (cnt >= 0) {
			fprintf(file, "%s", buf);
			fprintf(stdout, "%s", buf);
		}

		usleep(senslog_common.step);
	}

	fclose(file);
	sensc_deinit();

	return EXIT_SUCCESS;
}
