/*
 * Phoenix-Pilot
 *
 * calib - magnetometer calibration procedure
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>

#include <board_config.h>
#include <libsensors.h>
#include <sensc.h>
#include <mctl.h>

#include "procedures/calls.h"


#define CALIBMODE_OPT_MAGIRON "iron"
#define CALIBMODE_OPT_MAGMOT  "mot"
#define CALIBMODE_OPT_ACCROT  "rot"

#define DEFAULT_FILESTR  "/etc/calib.conf"
#define DEFAULT_SENSPATH "/dev/sensors"

#define NUMBER_OF_MOTORS 4


enum mCalibMode { modeMagIron = 0,
	modeMagMot, modeAccRot };


struct {
	const char *filePath;
	const char *sensPath;
} cal_common;


const char *motPaths = {
	PWM_MOTOR1,
	PWM_MOTOR2,
	PWM_MOTOR3,
	PWM_MOTOR4
};


/* Config file read into `calib_params` structure */
int cal_calibsRead(const char *filepath)
{
	return 0;
}

/* Overwriting config file with values from `calib_params` structure */
int cal_calibsWrite(const char *filepath)
{
	return 0;
}


void cal_helpPrint(char *name)
{
	printf("Usage: %s [-m mode] [-f path]\n\
  -m - magnetometer calibration mode:\n\
    '%s' - hard/soft iron calib. process,\n\
    '%s' - motors interference calib. process.\n\
  -x - accelerometer calibration mode.\n\
  -f - optional filename to save calibration output.\n\
  -s - optional filename of sensorhub driver.\n\
  -h - prints this message and aborts calibration process.\n",
		name, CALIBMODE_OPT_MAGIRON, CALIBMODE_OPT_MAGMOT);
}


int cal_argParse(enum mCalibMode *mode, int argc, char *argv[])
{
	int opt, modeCnt = 0;
	const char *modeStr = NULL;
	static char helpMsg[] = "Use -h to show help message.";
	optind = 1; /* strange that this has to be set manually using sysexec */

	while ((opt = getopt(argc, argv, "hxm:f:")) != -1) {
		switch (opt) {
			case 'f':
				cal_common.filePath = optarg;
				break;
			case 'm':
				modeStr = optarg;
				modeCnt++;
				break;
			case 'x':
				modeStr = CALIBMODE_OPT_ACCROT;
				modeCnt++;
			case 'h':
				cal_helpPrint(argv[0]);
				break;
			default: /* '?' */
				printf("Parsing arguments error. %s", helpMsg);
				return -1;
		}
	}

	/* Handle mode selection mistakes */
	if (modeStr == NULL) {
		printf("No calibration mode specified! %s\n", helpMsg);
		return -1;
	}
	if (modeCnt != 1) {
		printf("Only one calibration mode can be specified\n");
	}

	/* Handle mode selection */
	if (strcmp(modeStr, CALIBMODE_OPT_MAGMOT) == 0) {
		*mode = modeMagMot;
	}
	else if (strcmp(modeStr, CALIBMODE_OPT_MAGIRON) == 0) {
		*mode = modeMagIron;
	}
	else if (strcmp(modeStr, CALIBMODE_OPT_ACCROT) == 0) {
		*mode = modeAccRot;
	}
	else {
		printf("'%s' - unknown calibration mode!\n", modeStr);
		return -1;
	}

	/* Handle optionals */
	if (cal_common.filePath == NULL) {
		cal_common.filePath = DEFAULT_FILESTR;
	}
	if (cal_common.sensPath == NULL) {
		cal_common.sensPath = DEFAULT_SENSPATH;
	}

	return 0;
}


void cal_shutdownDeinit(void)
{
	sensc_deinit();
	mctl_deinit();
}


int cal_calibDo(enum mCalibMode mode)
{
	switch (mode) {
		case modeMagMot:
			return cal_mMotCalib();
		case modeMagIron:
			return cal_mStaticCalib();
		case modeAccRot:
			return cal_aRotCalib();
		default:
			return -1;
	}
}


bool cal_isMotEnbl(enum mCalibMode mode)
{
	return (mode == modeMagMot) ? true : false;
}


int main(int argc, char **argv)
{
	enum mCalibMode mode;

	if (cal_argParse(&mode, argc, argv) < 0) {
		printf("Parsing arguments failed!\n");
		return EXIT_FAILURE;
	}

	if (cal_calibsRead(cal_common.filePath) < 0) {
		/* error printed by cal_calibsWrite() */
		return EXIT_FAILURE;
	}

	if (sensc_init(cal_common.sensPath) < 0) {
		printf("Failed to setup sensorhub client!\n");
		return EXIT_FAILURE;
	}

	/* Not arming engines here, this should be done by appropriate procedure */
	if (cal_isMotEnbl(mode)) {
		if (mctl_init(NUMBER_OF_MOTORS, motPaths) < 0) {
			printf("Failed to init motors!\n");
			cal_shutdownDeinit();
			return EXIT_FAILURE;
		}
	}

	if (cal_calibDo(mode) < 0) {
		printf("Failed to perform calibration process!\n");
		cal_shutdownDeinit();
		return EXIT_FAILURE;
	}

	if (cal_calibsWrite(cal_common.filePath) < 0) {
		/* error printed by cal_calibsWrite() */
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
