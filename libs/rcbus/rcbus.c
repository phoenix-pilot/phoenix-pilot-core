/*
 * Phoenix-Pilot
 *
 * Communication bus library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "rcbus.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <string.h>
#include <sys/minmax.h>
#include <sys/select.h>
#include <sys/threads.h>


#define SIZE_PACKET_IBUS 32
#define SIZE_PACKET_SBUS 25

#define SIZE_CHANNELS_IBUS 14
#define SIZE_CHANNELS_SBUS 16

#define MAX_VAL_IBUS 2000
#define MIN_VAL_IBUS 1000


static struct {
	int fd;
	rcbus_type_t type;
	time_t timeout;

	unsigned int tid;
	volatile unsigned int run;
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(8)));
} rcbus_common;


static void rcbus_scaleIbus(rcbus_msg_t *msg)
{
	int i;
	float ratio;

	for (i = 0; i < msg->channelsCnt; ++i) {
		ratio = (float)(msg->channels[i] - MIN_VAL_IBUS) / (float)(MAX_VAL_IBUS - MIN_VAL_IBUS);
		msg->channels[i] = ratio * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE) + MIN_CHANNEL_VALUE;
	}
}


/* Protocol structure:
 * uint16_t header
 * uint16_t channels[SIZE_CHANNELS_IBUS]
 * uint16_t checksum
 */
static int rcbus_parseIbus(const uint8_t *data, size_t sz, rcbus_msg_t *msg)
{
	static size_t pos = 0;
	static uint16_t sum = 0;
	static uint8_t buff[SIZE_PACKET_IBUS];
	static const uint8_t header[] = { 0x20, 0x40 };

	int i, res = -1;
	uint16_t checksum;

	for (i = 0; i < sz; ++i) {
		/* Find a packet header */
		if (pos < sizeof(header)) {
			if (data[i] == header[pos]) {
				buff[pos++] = data[i];
				sum += data[i];
			}
			else {
				pos = 0;
				sum = 0;
			}
		}
		/* Get channels data */
		else if (pos >= sizeof(header) && pos < (SIZE_PACKET_IBUS - sizeof(checksum))) {
			buff[pos++] = data[i];
			sum += data[i];
		}
		/* Get checksum and parse packet to rcbus_msg_t structure */
		else if (pos >= (SIZE_PACKET_IBUS - sizeof(checksum)) && pos < SIZE_PACKET_IBUS) {
			buff[pos++] = data[i];
			if (pos == SIZE_PACKET_IBUS) {
				checksum = buff[SIZE_PACKET_IBUS - sizeof(checksum)] | (buff[SIZE_PACKET_IBUS - 1] << 8);

				if ((checksum + sum) == 0xffff) {
					memcpy(msg->channels, buff + sizeof(header), (msg->channelsCnt) * sizeof(msg->channels[0]));
					rcbus_scaleIbus(msg);
					res = 0;
				}

				pos = 0;
				sum = 0;
			}
		}
	}

	return res;
}


static int rcbus_parseSbus(const uint8_t *data, size_t sz, rcbus_msg_t *msg)
{
	/* TODO: take into account that S-bus channel's data is 11 bits. */

	return -1;
}


static void rcbus_rcvThread(void *arg)
{
	int err;
	size_t sz;
	ssize_t res;
	rcbus_msg_t msg;
	fd_set rfds;
	struct timeval tv;

	uint8_t data[max(SIZE_PACKET_SBUS, SIZE_PACKET_IBUS)];
	int (*parser)(const uint8_t *, size_t, rcbus_msg_t *);

	RcMsgHandler handler = (RcMsgHandler)arg;

	switch (rcbus_common.type) {
		case rc_typeIbus:
			sz = SIZE_PACKET_IBUS;
			msg.channelsCnt = SIZE_CHANNELS_IBUS;
			msg.channels = malloc(SIZE_CHANNELS_IBUS * sizeof(msg.channels[0]));
			parser = rcbus_parseIbus;
			break;

		case rc_typeSbus:
			sz = SIZE_PACKET_SBUS;
			msg.channelsCnt = SIZE_CHANNELS_SBUS;
			msg.channels = malloc(SIZE_CHANNELS_SBUS * sizeof(msg.channels[0]));
			parser = rcbus_parseSbus;
			break;

		default:
			perror("bus: wrong bus type.");
			endthread();
			return;
	}

	if (msg.channels == NULL) {
		endthread();
		return;
	}

	while (rcbus_common.run == 1) {
		FD_ZERO(&rfds);
		FD_SET(rcbus_common.fd, &rfds);

		tv.tv_sec = rcbus_common.timeout / 1000;
		tv.tv_usec = (rcbus_common.timeout % 1000) * 1000;

		err = select(rcbus_common.fd + 1, &rfds, NULL, NULL, &tv);
		if (err > 0) {
			res = read(rcbus_common.fd, data, sz);
			if (res > 0) {
				err = parser(data, res, &msg);
				if (err == 0) {
					handler((const rcbus_msg_t *)&msg);
				}
			}
		}
		else if (err == 0) {
			perror("bus: timeout occurred.");
		}
		else {
			perror("bus: select error occurred.");
		}
	}

	free(msg.channels);

	endthread();
}


int rcbus_run(RcMsgHandler handler, time_t timeout)
{
	rcbus_common.run = 1;
	rcbus_common.timeout = timeout;

	return beginthreadex(rcbus_rcvThread, 2, rcbus_common.stack, sizeof(rcbus_common.stack), (void *)handler, &rcbus_common.tid);
}


void rcbus_done(void)
{
	rcbus_common.run = 0;

	/* FIXME: add tid to threadJoin function, after merge in libphoenix */
	threadJoin(0);

	close(rcbus_common.fd);
}


static int rcbus_ibusConfig(void)
{
	struct termios attr;

	if (tcgetattr(rcbus_common.fd, &attr) < 0) {
		perror("bus: tcgetattr failed.");
		return -1;
	}

	attr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	attr.c_oflag &= ~OPOST;
	attr.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	attr.c_cflag &= ~(CSIZE | CSTOPB);
	attr.c_cflag |= CS8 | CREAD | CLOCAL;

	attr.c_cc[VMIN] = 1;
	attr.c_cc[VTIME] = 0;

	/* Set baudrate (B115200) */
	if (cfsetispeed(&attr, B115200) < 0 || cfsetospeed(&attr, B115200) < 0) {
		perror("bus: failed to set baudrate.");
		return -1;
	}

	if (tcflush(rcbus_common.fd, TCIOFLUSH) < 0) {
		printf("bus: tcflush failed.");
		return -1;
	}

	if (tcsetattr(rcbus_common.fd, TCSANOW, &attr) < 0) {
		printf("bus: tcsetattr failed.");
		return -1;
	}

	return 0;
}


static int rcbus_sbusConfig(void)
{
	/* TODO: configure serial for sbus receiver */

	return -1;
}


int rcbus_init(const char *devPath, rcbus_type_t type)
{
	int ret = 0, cnt = 0;

	if (devPath == NULL) {
		return -1;
	}

	rcbus_common.run = 0;
	rcbus_common.type = type;

	rcbus_common.fd = open(devPath, O_RDONLY | O_NOCTTY);
	while (rcbus_common.fd < 0) {
		usleep(10 * 1000);

		if (++cnt > 10000) {
			fprintf(stderr, "bus: timeout waiting on %s \n", devPath);
			return -1;
		}

		rcbus_common.fd = open(devPath, O_RDONLY | O_NOCTTY);
	}

	switch (type) {
		case rc_typeIbus:
			ret = rcbus_ibusConfig();
			break;

		case rc_typeSbus:
			ret = rcbus_sbusConfig();
			break;

		default:
			break;
	}

	return ret;
}
