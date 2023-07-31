#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

#include <libsensors.h>
#include <ekflib.h>

#include "../../log.h"

#define TEST_FILE     "log_volume_test.bin"
#define TEST_LOGS_CNT 10000


static const sensor_event_t logsVol_evtData = {
	.type = SENSOR_TYPE_GPS,
	.timestamp = 1,
	.gps = {
		.devId = 2,
		.alt = 3,
		.lat = 4,
		.lon = 5,
		.utc = 6,
		.hdop = 7,
		.vdop = 8,
		.altEllipsoid = 9,
		.groundSpeed = 10,
		.velNorth = 11,
		.velEast = 12,
		.velDown = 13,
		.eph = 14,
		.epv = 15,
		.evel = 16,
		.heading = 17,
		.headingOffs = 18,
		.headingAccur = 19,
		.satsNb = 20,
		.fix = 21,
	}
};


static size_t gpsLogSize = 0;


int logsVol_maxLogsGet(bool *success, double *bytesPerSec)
{
	int i, lost = 0, err = 0;
	struct timeval start, end;
	double timeDiff;

	if (ekflog_init(TEST_FILE, EKFLOG_SENSC) != 0) {
		return -1;
	}

	gettimeofday(&start, NULL);

	for (i = 0; i < TEST_LOGS_CNT; i++) {
		if (ekflog_senscGpsWrite(&logsVol_evtData) != 0) {
			lost++;
		}
	}

	gettimeofday(&end, NULL);

	err |= ekflog_done();

	timeDiff = (end.tv_sec - start.tv_sec) + (double)(end.tv_usec - start.tv_usec) / 1000000;

	*success = i == 0 ? true : false;
	*bytesPerSec = gpsLogSize * TEST_LOGS_CNT / timeDiff;

	return err;
}


int logsVol_gpsLogSizeGet(size_t *result)
{
	int res;
	struct stat file_stats;

	if (ekflog_init(TEST_FILE, EKFLOG_SENSC | EKFLOG_STRICT_MODE) != 0) {
		return -1;
	}

	res = ekflog_senscGpsWrite(&logsVol_evtData);
	ekflog_done();

	if (res != 0) {
		return -1;
	}

	if (stat(TEST_FILE, &file_stats) != 0) {
		fprintf(stderr, "logs volume: cannot check file size\n");
		return -1;
	}

	*result = file_stats.st_size;

	return 0;
}


int main(int argc, char **argv)
{
	bool success;
	double bytesPerSec;

	if (logsVol_gpsLogSizeGet(&gpsLogSize) != 0) {
		return -1;
	}

	ekf_init();
	ekf_run();

	/* Wait for ekf */
	sleep(15);

	if (logsVol_maxLogsGet(&success, &bytesPerSec) != 0) {
		return -1;
	}

	printf("success: %s bytes per sec: %f\n", success ? "true" : "false", bytesPerSec);

	if (ekf_stop() != 0) {
		return -1;
	}

	ekf_done();

	return 0;
}
