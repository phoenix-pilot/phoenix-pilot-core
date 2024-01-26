/*
 * Phoenix-RTOS
 *
 * Quadcopter Flight Controller
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-PILOT
 *
 * %LICENSE%
 */

#ifndef _QUADCONTROL_H_
#define _QUADCONTROL_H_

#include <time.h>
#include <stdint.h>

#include "pid.h"


/* Flight Modes Definitions */


/* Quadcopter flight modes */
/* clang-format off */
typedef enum { /* Basic modes:  */ flight_idle = 0, flight_disarm, flight_arm,
			   /* Auto modes:   */ flight_takeoff, flight_waypoint, flight_landing, flight_end,
               /* Manual modes: */ flight_manual, flight_manualAbort } flight_type_t;
/* clang-format on */

typedef struct {
	int32_t alt; /* altitude in 1E-3 [m] (millimetres) above MSL */
	time_t time; /* time (milliseconds) of hover after takeoff */
} flight_takeoff_t;


typedef struct {
	float yaw;   /* yaw angle in milliradians in range (-PI, PI] */
	float pitch; /* pitch angle in milliradians in range (-PI/2, PI/2] */
	float roll;  /* roll angle in milliradians in range (-PI/2, PI/2] */
} quad_att_t;


typedef struct {
	int32_t descent; /* Descent speed in millimeters per second */
	int32_t alt;     /* Altitude from which cautious landing begins */
	time_t timeout;  /* Time threshold (milliseconds) of low descend speed concluding landing */
} flight_landing_t;


typedef struct {
	time_t time;      /* Minimal time at waypoint (milliseconds) */
	int32_t posNorth; /* Waypoint position north of home (millimeters) */
	int32_t posEast;  /* Waypoint position east of home (millimeters) */
	int32_t alt;      /* Waypoint height above home (millimeters) */
	int32_t dist;     /* Minimum distance of waypoint arrival acceptance (millimeters) */
} flight_waypoint_t;


typedef struct {
	flight_type_t type;

	union {
		flight_takeoff_t takeoff;
		flight_waypoint_t waypoint;
		flight_landing_t landing;
	};
} flight_mode_t;

/* Throttle configuration */
typedef struct {
	float min;
	float max;
} quad_throttle_t;

/* Quadcopter configuration */
typedef struct {
	float dragCoeff;  /* drag factor [kg * m] */
	float trustCoeff; /* trust coefficient [kg * m] */
	float dist;       /* distance between motor axis and center of gravity [m] */
	uint32_t maxRPM;
} quad_coeffs_t;

typedef struct {
	pid_ctx_t alt;
	pid_ctx_t pitch;
	pid_ctx_t roll;
	pid_ctx_t yaw;
	pid_ctx_t pos;
} quad_pids_t;


#endif
