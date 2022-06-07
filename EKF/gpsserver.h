#ifndef GPSSERVER_H
#define GPSSERVER_H

#define EARTH_SEMI_MAJOR 6378137.0
#define EARTH_SEMI_MINOR 6356752.3
#define EARTH_ECCENTRICITY_SQUARED  0.006694384

#include <stdint.h>

typedef struct {
	uint32_t devId;
	int32_t lat;          /* latitude in 1E-7 degrees */
	int32_t lon;          /* longitude in 1E-7 degrees */
	int32_t hdop;         /* horizontal dilution of precision */
	int32_t vdop;         /* vertical dilution of precision */
	int32_t alt;          /* altitude in 1E-3 [m] (millimetres) above MSL */
	int32_t altEllipsoid; /* altitude in 1E-3 [m] (millimetres) above Ellipsoid */
	int32_t groundSpeed;  /* GPS ground speed, [mm/s] */
	int32_t velNorth;     /* GPS North velocity, [mm/s] */
	int32_t velEast;      /* GPS East velocity, [mm/s] */
	int32_t velDown;      /* GPS Down velocity, [mm/s] */
	uint32_t eph;         /* GPS horizontal position accuracy 1E-3 [m] (millimetres)*/
	uint32_t epv;         /* GPS vertical position accuracy 1E-3 [m] (millimetres)*/
	int32_t heading;      /* value in [mrad] */
	int32_t headingOffs;  /* value in [mrad] */
	int32_t headingAccur; /* value in [mrad] */
	uint8_t satsNb;       /* number of used satellites */
	uint8_t reserved[3];
} gps_data_t;


void gpsServerThread(void);

int sensGps(gps_data_t * data);

void print_gps_data(gps_data_t * data);

#endif
