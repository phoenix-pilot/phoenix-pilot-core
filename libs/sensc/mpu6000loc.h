#ifndef _MPU_6000_LOC_H_
#define _MPU_6000_LOC_H_

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <spi.h>
#include <sensors-spi.h>
#include <string.h>

#include <libsensors.h>

typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtAccel;
	sensor_event_t evtGyro;
	uint8_t lpfSel;
	char stack[512] __attribute__((aligned(8)));
} mpu6000_ctx_t;


int mpu6000_getData(mpu6000_ctx_t *ctx, sensor_event_t *gyroEvt, sensor_event_t *accelEvt);


int libmpu6000_alloc(mpu6000_ctx_t *ctx, const char *args);


#endif