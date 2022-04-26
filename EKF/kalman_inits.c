/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * matrix (memory and values) initialization 
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* NOTE: must be kept in the same order as 'config_names' */
kalman_init_t init_values = {
	.P_xerr = 0.1F,
	.P_verr = 0.1F,
	.P_aerr = 0.001F,
	.P_werr = DEG2RAD,
	.P_qaerr = DEG2RAD,
	.P_qijkerr = DEG2RAD,

	.R_acov = 0.001F,
	.R_wcov = 0.001F,

	.Q_acov = 0,
	.Q_wcov = 0
};

/* NOTE: must be kept in the same order as 'init_values' */
char *config_names[] = {
	"P_xerr", "P_verr", "P_aerr", "P_werr", "P_qaerr", "P_qijkerr",
	"R_acov", "R_wcov",
	"Q_acov", "Q_wcov"
};


void read_config(void)
{
	char buf[30], *p, *v;
	int i;
	float val;
	FILE *fd = fopen("config", "r");
	if (fd == NULL) {
		printf("No config file found!\n");
	}
	else {
		while (fgets(buf, sizeof(buf), fd)) {
			p = strtok(buf, " ");
			v = strtok(NULL, " ");
			val = atof(v);
			for (i = 0; i < sizeof(init_values) / sizeof(float); i++) {
				if (memcmp(p, config_names[i], strlen(config_names[i])) == 0) {
					((float *)&init_values)[i] = (float)val;
					break;
				}
			}
		}
		fclose(fd);
	}

	printf("config:\n");
	for (i = 0; i < sizeof(init_values) / sizeof(float); i++) {
		printf("%s = %f\n", config_names[i], ((float *)&init_values)[i]);
	}
}

/* state values init */
void init_state_vector(phmatrix_t *state)
{
	state->data[ixx] = state->data[ixy] = state->data[ixz] = 0; /* start position at [0,0,0] */
	state->data[ivx] = state->data[ivy] = state->data[ivz] = 0; /* start velocity at [0,0,0] */
	state->data[iax] = state->data[iay] = state->data[iaz] = 0; /* start acceleration at [0,0,0] */
	state->data[iwx] = state->data[iwy] = state->data[iwz] = 0; /* start angular speed at [0,0,0] */
	state->data[iqa] = init_q.a;                                /* start rotation at identity quaternion */
	state->data[iqb] = init_q.i;
	state->data[iqc] = init_q.j;
	state->data[iqd] = init_q.k;
}

void init_cov_vector(phmatrix_t *cov)
{
	phx_zeroes(cov);
	cov->data[cov->cols * ixx + ixx] = init_values.P_xerr * init_values.P_xerr;
	cov->data[cov->cols * ixy + ixy] = init_values.P_xerr * init_values.P_xerr;
	cov->data[cov->cols * ixz + ixz] = init_values.P_xerr * init_values.P_xerr;
	cov->data[cov->cols * ivx + ivx] = init_values.P_verr * init_values.P_verr;
	cov->data[cov->cols * ivy + ivy] = init_values.P_verr * init_values.P_verr;
	cov->data[cov->cols * ivz + ivz] = init_values.P_verr * init_values.P_verr;
	cov->data[cov->cols * iax + iax] = init_values.P_aerr * init_values.P_aerr;
	cov->data[cov->cols * iay + iay] = init_values.P_aerr * init_values.P_aerr;
	cov->data[cov->cols * iaz + iaz] = init_values.P_aerr * init_values.P_aerr;
	cov->data[cov->cols * iwx + iwx] = init_values.P_werr * init_values.P_werr;
	cov->data[cov->cols * iwy + iwy] = init_values.P_werr * init_values.P_werr;
	cov->data[cov->cols * iwz + iwz] = init_values.P_werr * init_values.P_werr;
	cov->data[cov->cols * iqa + iqa] = init_values.P_qaerr * init_values.P_qaerr;
	cov->data[cov->cols * iqb + iqb] = init_values.P_qijkerr * init_values.P_qijkerr;
	cov->data[cov->cols * iqc + iqc] = init_values.P_qijkerr * init_values.P_qijkerr;
	cov->data[cov->cols * iqd + iqd] = init_values.P_qijkerr * init_values.P_qijkerr;
}


void init_prediction_matrices(phmatrix_t *state, phmatrix_t *state_est, phmatrix_t *cov, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt)
{
	/* Q_meas noise matrix */
	float Qm_data[STATE_ROWS * STATE_ROWS];
	phmatrix_t Qm = { .cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = Qm_data };
	float tmp_data[STATE_ROWS * STATE_ROWS];
	phmatrix_t tmp = { .cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = tmp_data };

	/* matrix initialization */
	phx_newmatrix(state, STATE_ROWS, STATE_COLS);     /* 16x1 */
	phx_newmatrix(state_est, STATE_ROWS, STATE_COLS); /* 16x1 */

	phx_newmatrix(cov, STATE_ROWS, STATE_ROWS);     /* 16x16 */
	phx_newmatrix(cov_est, STATE_ROWS, STATE_ROWS); /* 16x16 */
	phx_newmatrix(F, STATE_ROWS, STATE_ROWS);       /* 16x16 */
	phx_newmatrix(Q, STATE_ROWS, STATE_ROWS);       /* 16x16 */

	init_state_vector(state);
	init_cov_vector(cov);

	jacobian_F(state, F, dt); /* F needed for Q matrix */
	phx_zeroes(&Qm);
	Qm.data[Qm.cols * iax + iax] = Qm.data[Qm.cols * iay + iay] = Qm.data[Qm.cols * iaz + iaz] = init_values.Q_acov;
	Qm.data[Qm.cols * iwx + iwx] = Qm.data[Qm.cols * iwy + iwy] = Qm.data[Qm.cols * iwz + iwz] = init_values.Q_wcov;
	//phx_sadwitch_product(F, &Qm, Q, &tmp);
	phx_zeroes(Q);
	phx_add(Q, &Qm, NULL);
}

void init_update_matrices(phmatrix_t *H, phmatrix_t *R)
{
	/* matrix initialization */
	phx_newmatrix(H, MEAS_ROWS, STATE_ROWS); /* 6x16 */
	phx_newmatrix(R, MEAS_ROWS, MEAS_ROWS);  /* 6x6 */

	/* init of measurement noise matrix R */
	R->data[R->cols * imax + imax] = init_values.R_acov;
	R->data[R->cols * imay + imay] = init_values.R_acov;
	R->data[R->cols * imaz + imaz] = init_values.R_acov;
	R->data[R->cols * imwx + imwx] = init_values.R_wcov;
	R->data[R->cols * imwy + imwy] = init_values.R_wcov;
	R->data[R->cols * imwz + imwz] = init_values.R_wcov;
}
