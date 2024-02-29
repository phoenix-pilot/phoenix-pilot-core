/*
 * Phoenix-Pilot
 *
 * mavlink.c
 *
 * Mavlink protocol implementation
 *
 * Copyright 2024 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "mavlink.h"
#include "mavlink_enums.h"


#define MAVLINK_LABEL "mavlink"

#define MAVLINK_10_MAGIC      0xfe /* magic number for mavlink v1.0 */
#define MAVLINK_20_MAGIC      0xfd /* magic number for mavlink v2.0 */
#define MAVLINK_10_LEN_HEADER 6    /* header length for mavlink v1.0 */
#define MAVLINK_20_LEN_HEADER 9    /* header length for mavlink v2.0 */
#define MAVLINK_LEN_CHECKSUM  2    /* checksum field length */

/* clang-format off */
static const uint8_t crcExtra[256] = { 50, 124, 137, 0, 237, 217, 104, 119, 117, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23,\
	170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41,\
	39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158,\
	152, 143, 0, 0, 14, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102,\
	158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87,\
	203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178,\
	200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
	0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
	0, 0, 0, 0, 0, 208, 0, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8,\
	204, 49, 170, 44, 83, 46, 0 };
/* clang-format on */

/* Mavlink CRC-16/MCRF4XX checksum calculator. Returns calculated checksum over `msgBuf` buffer of length `msgBufSz`. `msgBuf` must be a valid pointer. */
static uint16_t mav_crc16(const uint8_t *msgBuf, uint16_t msgBufSz)
{
	uint16_t i, tmp, crc = 0xffff;

	for (i = 0; i < msgBufSz; i++) {
		tmp = msgBuf[i] ^ (crc & 0xff);
		tmp ^= (tmp << 4);
		crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
	}

	return crc;
}


/* Fills header pointed with `h` for sending the message of `payloadLen` length, `msgid` type and `comp` mavlink component as sender. */
static inline void mav1_headerFill(mav1_header_t *h, const mav_comp_t *comp, uint8_t payloadLen, uint8_t msgid)
{
	h->magic = MAVLINK_10_MAGIC;
	h->len = payloadLen;
	h->seq = comp->seq;
	h->sysid = comp->sys->id;
	h->compid = comp->id;
	h->msgid = msgid;
}


/* Generic message writing function. Blocking, as if `write()` is used. Returns 0 if whole message was sent successfully, -1 otherwise. */
static int mav_sendMsg(mav_comp_t *comp, enum mav_msgid msgId, const uint8_t *payload, uint8_t payloadLen)
{
	uint8_t *buf;
	uint16_t checksum, totalLen = 0;
	int ret, sent, tries;

	/* Message buffer alias for better readability */
	buf = comp->sys->msgBuf;

	switch (comp->sys->ver) {
		case mav_version_1:
			mav1_headerFill((mav1_header_t *)&buf[0], comp, payloadLen, msgId);

			memcpy(buf + MAVLINK_10_LEN_HEADER, payload, payloadLen);

			/*
			 * Mavlink checksum is calculated over entire frame, excluding first byte and checksum,
			 * and additional crc_extra byte (payload type specific magic number) "glued" at the end of payload.
			 *
			 * This implementation uses the first byte of checksum (bytewise in frame) for crc_extra injection,
			 * so that crc function sees it as ordinary data to calculate checksum over.
			 */
			buf[MAVLINK_10_LEN_HEADER + payloadLen] = crcExtra[msgId];
			checksum = mav_crc16(&buf[1], MAVLINK_10_LEN_HEADER + payloadLen);
			buf[MAVLINK_10_LEN_HEADER + payloadLen] = (uint8_t)(checksum & 0xff);
			buf[MAVLINK_10_LEN_HEADER + payloadLen + 1] = (uint8_t)(checksum >> 8);

			totalLen = MAVLINK_10_LEN_HEADER + payloadLen + MAVLINK_LEN_CHECKSUM;

			break;

		case mav_version_2:
		default:
			fprintf(stderr, "%s: unsupported mavlink version\n", MAVLINK_LABEL);
			return -1;
	}


	sent = 0;
	for (tries = 3; tries > 0; tries--) {
		ret = write(comp->sys->fd, buf, totalLen);
		if (ret >= 0) {
			sent += ret;
		}

		if (sent == totalLen) {
			break;
		}
	}

	return (sent == totalLen) ? 0 : -1;
}


int mav_sendHeartbeat(mav_comp_t *comp, const mav_heartbeat_t *payload)
{
	return mav_sendMsg(comp, mav_msgid_heartbeat, (uint8_t *)payload, sizeof(mav_heartbeat_t));
}


int mav_sendGlobalPositionInt(mav_comp_t *comp, const mav_globalPositionInt_t *payload)
{
	return mav_sendMsg(comp, mav_msgid_globalPositionInt, (uint8_t *)payload, sizeof(mav_globalPositionInt_t));
}


int mav_compDone(mav_comp_t *comp)
{
	/* Nothing to be done yet! */
	return 0;
}


int mav_compInit(mav_comp_t *comp, uint8_t id, mav_sys_t *sys)
{
	/* component id == 	MAV_COMP_ID_ALL is invalid by mavlink standard */
	if (id == MAV_COMP_ID_ALL) {
		return -1;
	}

	comp->id = id;
	comp->sys = sys;
	comp->seq = 0;

	return 0;
}


int mav_sysDone(mav_sys_t *sys)
{
	/* Nothing to be done yet! */
	return 0;
}


int mav_sysInit(mav_sys_t *sys, int fd, uint8_t id, enum mav_version ver)
{
	/* system id == 0 is invalid by mavlink standard */
	if (fd < 0 || id == 0) {
		return -1;
	}

	switch (ver) {
		case mav_version_1:
			sys->fd = fd;
			sys->ver = mav_version_1;
			sys->id = id;
			break;

		case mav_version_2:
		default:
			fprintf(stderr, "%s: unsupported mavlink version\n", MAVLINK_LABEL);
			return -1;
	}

	return 0;
}
