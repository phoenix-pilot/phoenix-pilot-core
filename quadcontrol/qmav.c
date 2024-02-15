#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <errno.h>

#include <mavlink.h>
#include <mavlink_enums.h>

#include "qmav.h"

struct {
	mav_sys_t sys;        /* main mavlink system */
	mav_comp_t autopilot; /* autopilot component abstraction */

	/* Thread related variables */
	pthread_t tid;
	volatile int run;
	pthread_mutex_t lock;
	pthread_attr_t threadAttr;

	/* heartbeat status/mode buffers */
	volatile uint8_t state;
	volatile uint8_t mode;
	volatile uint8_t custMode;
} qmav_common;


void qmav_setStatus(uint8_t state, uint8_t mode)
{
	if (pthread_mutex_trylock(&qmav_common.lock) == 0) {
		qmav_common.state = state;
		qmav_common.mode = mode;

		pthread_mutex_unlock(&qmav_common.lock);
	}
}


static void *qmav_thread(void *arg)
{
	mav_heartbeat_t heartbeat = { .type = MAV_TYPE_QUADROTOR, .autopilot = MAV_AUTOPILOT_GENERIC };

	fprintf(stdout, "qmav: started...\n");
	qmav_common.run = 1;

	while (qmav_common.run) {
		pthread_mutex_lock(&qmav_common.lock);
		heartbeat.base_mode = qmav_common.mode;
		heartbeat.custom_mode = qmav_common.custMode;
		heartbeat.system_status = qmav_common.state;
		pthread_mutex_unlock(&qmav_common.lock);

		mav_sendHeartbeat(&qmav_common.autopilot, &heartbeat);

		sleep(1);
	}

	return NULL;
}


int qmav_run(void)
{
	if (pthread_create(&qmav_common.tid, &qmav_common.threadAttr, qmav_thread, NULL) != 0) {
		fprintf(stderr, "qmav: failed to start\n");
		return -1;
	}

	return 0;
}


int qmav_stop(void)
{
	qmav_common.run = 0;

	return pthread_join(qmav_common.tid, NULL);
}


void qmav_done(void)
{
	pthread_mutex_destroy(&qmav_common.lock);
	pthread_attr_destroy(&qmav_common.threadAttr);

	mav_sysDone(&qmav_common.sys);
	mav_compDone(&qmav_common.autopilot);

	close(qmav_common.sys.fd);
}


int qmav_init(const char *path)
{
	int fd = -1;

	if (path == NULL) {
		fd = dup(STDOUT_FILENO);
	}
	else {
		fprintf(stderr, "qmav: device/file opening not supported!\n");
		return -1;
	}

	if (fd < 0) {
		fprintf(stderr, "Failed to open file for mavlink\n");
		return -1;
	}

	if (pthread_attr_init(&qmav_common.threadAttr) != 0) {
		fprintf(stderr, "Attribute init failed\n");
		return -1;
	}

	if (pthread_mutex_init(&qmav_common.lock, NULL) < 0) {
		printf("Cannot create mutex for qmav\n");
		pthread_attr_destroy(&qmav_common.threadAttr);
		close(fd);
		return -1;
	}

	if (mav_sysInit(&qmav_common.sys, fd, MAV_AUTOPILOT_GENERIC, mav_version_1) < 0) {
		fprintf(stderr, "Failed to init mavlink system context\n");
		pthread_mutex_destroy(&qmav_common.lock);
		pthread_attr_destroy(&qmav_common.threadAttr);
		close(fd);
		return -1;
	}

	if (mav_compInit(&qmav_common.autopilot, MAV_COMP_ID_AUTOPILOT1, &qmav_common.sys) < 0) {
		fprintf(stderr, "Failed to init mavlink autopilot component\n");
		pthread_mutex_destroy(&qmav_common.lock);
		pthread_attr_destroy(&qmav_common.threadAttr);
		mav_sysDone(&qmav_common.sys);
		close(fd);
		return -1;
	}

	qmav_common.run = 0;
	qmav_common.state = MAV_STATE_UNINIT;
	qmav_common.mode = MAV_MODE_PREFLIGHT;
	qmav_common.custMode = 0;

	return 0;
}
