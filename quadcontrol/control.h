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

#define DEBUG_LOG(str_, ...) do { if (1) printf(str_ , ##__VA_ARGS__); } while (0)

/* Flight Modes Definitions */

/* Basic quadcopter flight modes */
typedef enum { flight_takeoff = 0, flight_pos, flight_hover, flight_landing, flight_end } flight_type_t;


typedef struct {
	int32_t alt; /* altitude in 1E-3 [m] (millimetres) above MSL */
} flight_takeoff_t;


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
	flight_type_t type;

	union {
		flight_takeoff_t takeoff;
		flight_position_t pos;
		flight_hover_t hover;
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
