/*
 * Phoenix-Pilot
 *
 * mavlink.h
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

#ifndef PHOENIX_MAVLINK_1_FRAME_H
#define PHOENIX_MAVLINK_1_FRAME_H

#include <stdint.h>

/* clang-format off */
/* Mavlink version */
enum mav_version { mav_version_1 = 0, mav_version_2};


/* Supported messages ids. Enum values match XML message types identification */
enum mav_msgid { mav_msgid_heartbeat = 0, mav_msgid_globalPositionInt = 33 };
/* clang-format on */

/* Mavlink system/connection context. Represents one Micro-Air-Vehicle system */
typedef struct {
	int fd;     /* file descriptor to write/read messages */
	uint8_t id; /* system`s ID */

	enum mav_version ver; /* version of mavlink protocol used by this system */
	uint8_t msgBuf[280];  /* accommodates both 1.0 and 2.0 messages */
} mav_sys_t;


/* Mavlink component. Represents one component of Micro-Air-Vehicle system */
typedef struct {
	mav_sys_t *sys; /* Mavlink system this component belongs to */
	uint8_t id;     /* component`s ID */
	uint8_t seq;    /* message sequence counter */
} mav_comp_t;


/* mavlink v1.0 header structure */
typedef struct __attribute__((packed)) {
	uint8_t magic;
	uint8_t len;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;
} mav1_header_t;

/* Heartbeat message structure. Over-the-wire order of variables */
typedef struct __attribute__((packed)) {
	uint32_t custom_mode;    /* A bitfield for use for autopilot-specific flags */
	uint8_t type;            /* MAV_TYPE_* field. Vehicle or component type */
	uint8_t autopilot;       /* MAV_AUTOPILOT_* field. Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. */
	uint8_t base_mode;       /* MAV_MODE_FLAG_* field. System mode bitmap. */
	uint8_t system_status;   /* MAV_STATE_* field. System status flag. */
	uint8_t mavlink_version; /* MAVLink version, not writable by user, gets added by protocol. */
} mav_heartbeat_t;


/* The filtered global position. Over-the-wire order of variables */
typedef struct __attribute__((packed)) {
	uint32_t time_boot_ms; /* Timestamp (time since system boot) in [ms] */
	int32_t lat;           /* Latitude, expressed in [degE7] */
	int32_t lon;           /* Longitude, expressed in [degE7] */
	int32_t alt;           /* Altitude (MSL). */
	int32_t relative_alt;  /* Altitude above ground */
	int16_t vx;            /* Ground X Speed (Latitude, positive north) */
	int16_t vy;            /* Ground Y Speed (Longitude, positive east) */
	int16_t vz;            /* Ground Z Speed (Altitude, positive down) */
	uint16_t hdg;          /* Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX */
} mav_globalPositionInt_t;


/* Sends heartbear message as `comp` context */
int mav_sendHeartbeat(mav_comp_t *comp, const mav_heartbeat_t *payload);


int mav_sendGlobalPositionInt(mav_comp_t *comp, const mav_globalPositionInt_t *payload);


/* Deinitializes `comp` mavlink component */
int mav_compDone(mav_comp_t *comp);


/*
 * Initializes `comp` mavlink component context
 *
 * `id`  - this component ID
 * `sys` - mavlink system this component is part of
 */
int mav_compInit(mav_comp_t *comp, uint8_t id, mav_sys_t *sys);


/* Deinitializes mavlink context. Does not close an underlying file descriptor! */
int mav_sysDone(mav_sys_t *sys);


/*
 * Initializes `sys` mavlink context
 *
 * `fd`  - opened file descriptor to read to / write from
 * `id`  - this system ID
 * `ver` - mavlink protocol version to be used
 */
int mav_sysInit(mav_sys_t *sys, int fd, uint8_t id, enum mav_version ver);


#endif
