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


/* Flight Modes Definitions */

/* Basic quadcopter flight modes */
typedef enum { flight_takeoff = 0, flight_pos, flight_hover, flight_landing, flight_end } flight_type_t;


typedef struct {
	int32_t alt; /* altitude in 1E-3 [m] (millimetres) above MSL */
	float time;  /* time of engine spool up in milliseconds */
} flight_takeoff_t;


typedef struct {
	float yaw;   /* yaw angle in milliradians in range (-PI, PI] */
	float pitch; /* pitch angle in milliradians in range (-PI/2, PI/2] */
	float roll;  /* roll angle in milliradians in range (-PI/2, PI/2] */
} quad_att_t;


typedef struct {
	int32_t alt; /* altitude in 1E-3 [m] (millimetres) above MSL */
	int32_t lat; /* latitude in 1E-7 degrees */
	int32_t lon; /* longitude in 1E-7 degrees */
} flight_position_t;


typedef struct {
	uint32_t alt; /* altitude in 1E-3 [m] (millimetres) above MSL */
	time_t time;  /* time in milliseconds spend in hover */
} flight_hover_t;


typedef struct {
	time_t time; /* time of engine spool down in milliseconds */
} flight_landing_t;


typedef struct {
	flight_type_t type;

	union {
		flight_takeoff_t takeoff;
		flight_position_t pos;
		flight_hover_t hover;
		flight_landing_t landing;
	};
} flight_mode_t;


/* Quadcopter configuration */
typedef struct {
	float dragCoeff;  /* drag factor [kg * m] */
	float trustCoeff; /* trust coefficient [kg * m] */
	float dist;       /* distance between motor axis and center of gravity [m] */
	uint32_t maxRPM;
} quad_coeffs_t;


#endif
