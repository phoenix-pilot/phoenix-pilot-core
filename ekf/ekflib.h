#ifndef EKFLIB_H
#define EKFLIB_H

typedef struct {
	/* position in ENU frame in meters */
	float enuX;
	float enuY;
	float enuZ;

	/* vehicle attitude, ranges according to Tait–Bryan convention */
	float pitch; /* (-PI/2, PI/2) */
	float yaw;   /* (-PI, PI) */
	float roll;  /* (-PI, PI) */
} ekf_state_t;

int ekf_init(void);

int ekf_run(void);

void ekf_done(void);

void ekfReset(void);

#endif