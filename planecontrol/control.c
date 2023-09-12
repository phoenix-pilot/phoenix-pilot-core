/*
 * Phoenix-RTOS
 *
 * Plane Flight Controller
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-PILOT
 *
 * %LICENSE%
 */

#include "mma.h"

#include <ekflib.h>

#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include <board_config.h>


typedef enum { mode_rc = 0, mode_auto, mode_stabilize } control_mode_t;


struct {
	control_mode_t mode;
} plane_common;


static int plane_run(void)
{
	ekf_state_t measure;
	float proll, ppitch, pyaw;

	mma_start();

	/* Simulation mode to check control of surfaces
	 * PIDs are not used. */
	while (plane_common.mode == mode_stabilize) {
		ekf_stateGet(&measure);

		proll = (-measure.roll * 1.6 + M_PI) / (2 * M_PI);
		ppitch = (-measure.pitch * 1.6 + M_PI / 2) / M_PI;
		pyaw = (-measure.yaw + M_PI) / (2 * M_PI);

		if (mma_control(0, proll, ppitch, pyaw) < 0) {
			return -1;
		}

		usleep(1000 * 2);
	}

	mma_stop();

	return 0;
}


static int plane_done(void)
{
	mma_done();
	ekf_done();

	return 0;
}


static int plane_init(void)
{
	/* MMA initialization */
	if (mma_init() < 0) {
		fprintf(stderr, "planecontrol: cannot initialize mma module\n");
		return -1;
	}

	/* EKF initialization */
	if (ekf_init(0) < 0) {
		fprintf(stderr, "planecontrol: cannot initialize ekf\n");
		return -1;
	}

	if (ekf_run() < 0) {
		fprintf(stderr, "planecontrol: cannot run ekf\n");
		return -1;
	}

	/* EKF needs time to calibrate itself */
	sleep(10);

	return 0;
}


static void plane_help(const char *progName)
{
	printf("Usage: %s [OPTIONS]\n"
		   "\t-c <sim>   :  sets control mode\n"
		   "\t-h         :  prints help\n",
		progName);
}


int main(int argc, char **argv)
{
	int err = EOK, c;
	pid_t pid, ret;
	int status = 0;

	sleep(5);

	if (argc < 2) {
		plane_help(argv[0]);
		return EXIT_FAILURE;
	}

	while ((c = getopt(argc, argv, "c:h")) != -1 && err == EOK) {
		switch (c) {
			case 'c':
				if (strcmp(optarg, "sim") == 0) {
					plane_common.mode = mode_stabilize;
					printf("Simulator mode.\n");
				}
				else {
					err = -EINVAL;
				}
				break;

			case 'h':
				plane_help(argv[0]);
				return EXIT_SUCCESS;

			default:
				plane_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	if (err < 0) {
		plane_help(argv[0]);
		return EXIT_FAILURE;
	}

	priority(1);

	err = plane_init();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	signal(SIGINT, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	signal(SIGTERM, SIG_IGN);

	pid = vfork();
	/* Parent process waits on child and makes clean up  */
	if (pid > 0) {
		do {
			ret = waitpid(pid, &status, 0);
		} while (ret < 0 && errno == EINTR);
	}
	/* Child process runs flight scenario and can be terminated */
	else if (pid == 0) {
		signal(SIGINT, SIG_DFL);
		signal(SIGQUIT, SIG_DFL);
		signal(SIGTERM, SIG_DFL);

		/* Take terminal control */
		tcsetpgrp(STDIN_FILENO, getpid());

		plane_run();
		exit(EXIT_SUCCESS);
	}
	else {
		fprintf(stderr, "planecontrol: vfork failed with code %d\n", pid);
	}

	plane_done();

	/* Take back terminal control */
	tcsetpgrp(STDIN_FILENO, getpid());

	return EXIT_SUCCESS;
}
