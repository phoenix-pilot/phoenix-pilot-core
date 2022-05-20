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

#include "kalman_implem.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>


/* NOTE: must be kept in the same order as 'config_names' */
kalman_init_t init_values = {
	.verbose = 1,

	.P_xerr = 0.1F,            /* 0.1 m */
	.P_verr = 0.1F,            /* 0.1 m/s */
	.P_aerr = 0.001F,          /* 0.001 m/s^2 */
	.P_werr = DEG2RAD,         /* 1 degree */
	.P_merr = 300,             /* 300 uT */
	.P_qaerr = 10 * DEG2RAD,   /* 10 degrees */
	.P_qijkerr = 10 * DEG2RAD, /* 10 degrees */
	.P_pxerr = 10,             /* 10 hPa */

	.R_acov = 0.001F,
	.R_wcov = 0.001F,
	.R_mcov = 10,
	.R_qcov = 1. / DEG2RAD,

	.R_pcov = 0.1,
	.R_hcov = 1,
	.R_xzcov = 1,
	.R_hvcov = 1,
	.R_vzcov = 2,

	/* better to keep Q low */
	.Q_hcov = 0.01,
	.Q_avertcov = 0.01,
	.Q_ahoricov = 0,
	.Q_wcov = 0.0001,
	.Q_mcov = 0.001,
	.Q_qcov = 0.001,
	.Q_pcov = 0.01,
	.Q_pvcov = 0.001
};


/* NOTE: must be kept in the same order as 'init_values' */
char *config_names[] = {
	"verbose",
	"P_xerr", "P_verr", "P_aerr", "P_werr", "P_merr", "P_qaerr", "P_qijkerr", "P_pxerr",
	"R_acov", "R_wcov", "R_mcov", "R_qcov", "R_pcov", "R_hcov", "R_xzcov", "R_hvcov", "R_vzcov",
	"Q_hcov", "Q_avertcov", "Q_ahoricov", "Q_wcov", "Q_mcov", "Q_qcov", "Q_pcov", "Q_pvcov"
};


/* reads config file named "config" from filesystem */
void read_config(void)
{
	char buf[32], *p, *v;
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
	verbose = init_values.verbose;

	printf("config:\n");
	for (i = 0; i < sizeof(init_values) / sizeof(float); i++) {
		printf("%s = %f\n", config_names[i], ((float *)&init_values)[i]);
	}
}


/* state vectors values init */
static void init_state_vector(phmatrix_t *state)
{
	state->data[ixx] = state->data[ixy] = state->data[ixz] = 0; /* start position at [0,0,0] */
	state->data[ivx] = state->data[ivy] = state->data[ivz] = 0; /* start velocity at [0,0,0] */
	state->data[iax] = state->data[iay] = state->data[iaz] = 0; /* start acceleration at [0,0,0] */
	state->data[iwx] = state->data[iwy] = state->data[iwz] = 0; /* start angular speed at [0,0,0] */

	/* start rotation at identity quaternion */
	state->data[iqa] = init_q.a;
	state->data[iqb] = init_q.i;
	state->data[iqc] = init_q.j;
	state->data[iqd] = init_q.k;

	/* start magnetic field as calibrated */
	state->data[imx] = init_m.x;
	state->data[imx] = init_m.y;
	state->data[imx] = init_m.z;

	/* start pressure set to 1013 hPa */
	state->data[ihz] = 0;
	state->data[ihv] = 0;
}


/* covariance matrox values inits */
static void init_cov_vector(phmatrix_t *cov)
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

	cov->data[cov->cols * imx + imx] = init_values.P_merr * init_values.P_merr;
	cov->data[cov->cols * imy + imy] = init_values.P_merr * init_values.P_merr;
	cov->data[cov->cols * imz + imz] = init_values.P_merr * init_values.P_merr;

	cov->data[cov->cols * ihz + ihz] = init_values.P_pxerr * init_values.P_pxerr;
	cov->data[cov->cols * ihv + ihv] = init_values.P_verr * init_values.P_verr;
}


/* State estimation function definition */
static void calculateStateEstimation(phmatrix_t *state, phmatrix_t *state_est, float dt)
{
	float dt2 = dt * dt / 2;
	quat_t quat_q, quat_w, /*quat_ap, quat_wp,*/ res;

	quat_q = quat(qa, qb, qc, qd);
	quat_w = quat(0, wx, wy, wz);

	state_est->data[ixx] = xx + vx * dt + ax * dt2;
	state_est->data[ixy] = xy + vy * dt + ay * dt2;
	state_est->data[ixz] = xz + vz * dt + az * dt2; /* complementary-like filter on height data to cancel accel druft */

	/* as no direct velocity measurements are done, time corelation is introduced to velocity with assumption that velocity always decreases */
	state_est->data[ivx] = (vx + ax * dt) * 0.9994;
	state_est->data[ivy] = (vy + ay * dt) * 0.9994;
	state_est->data[ivz] = (vz + az * dt) * 0.9994;

	/* predition from w */
	res = quat_mlt(&quat_w, &quat_q);
	quat_times(&res, dt / 2);
	quat_q = quat_add(&quat_q, &res);
	quat_normalize(&quat_q);

	state_est->data[iqa] = quat_q.a;
	state_est->data[iqb] = quat_q.i;
	state_est->data[iqc] = quat_q.j;
	state_est->data[iqd] = quat_q.k;

	state_est->data[iax] = ax;
	state_est->data[iay] = ay;
	state_est->data[iaz] = az;

	state_est->data[iwx] = wx;
	state_est->data[iwy] = wy;
	state_est->data[iwz] = wz;

	state_est->data[imx] = mx;
	state_est->data[imy] = my;
	state_est->data[imz] = mz;

	state_est->data[ihz] = hz;
	state_est->data[ihv] = hv;
}


/* prediction step jacobian calculation function */
static void calcPredictionJacobian(phmatrix_t *F, phmatrix_t *state, float dt)
{
	float dt2 = dt / 2; /* helper value */

	/* diagonal matrix */
	float I33_data[9] = { 0 };
	phmatrix_t I33 = { .rows = 3, .cols = 3, .transposed = 0, .data = I33_data };

	/* derrivative submatrix of (dfq / dq) of size 4x4 */
	float wxdt2 = wx * dt2, wydt2 = wy * dt2, wzdt2 = wz * dt2;
	float data_dfqdq[16] = {
		1, -wxdt2, -wydt2, -wzdt2,
		wxdt2, 1, -wzdt2, wydt2,
		wydt2, wzdt2, 1, -wxdt2,
		wzdt2, -wydt2, wxdt2, 1
	};
	/* derrivative submatrix of (dfq / dw) of size 4x3 */
	float qadt2 = qa * dt2, qbdt2 = qb * dt2, qcdt2 = qc * dt2, qddt2 = qd * dt2;
	float data_dfqdw[12] = {
		-qbdt2, -qcdt2, -qddt2,
		qadt2, qddt2, -qcdt2,
		-qddt2, qadt2, qbdt2,
		qcdt2, -qbdt2, qadt2
	};
	/* differentials matrices */
	phmatrix_t dfqdq, dfqdw;

	phx_assign(&dfqdq, 4, 4, data_dfqdq);
	phx_assign(&dfqdw, 4, 3, data_dfqdw);

	phx_diag(&I33);

	/* set F to zeroes and add ones on diag */
	phx_zeroes(F);
	phx_diag(F);

	/* change I33 to (I * dt) matrix and write it to appropriate places */
	phx_scalar_product(&I33, dt);
	phx_writesubmatrix(F, ixx, ivx, &I33); /* dfx / dv */
	phx_writesubmatrix(F, ivx, iax, &I33); /* dfv / da */

	/* change I33 to (I * dt^2 / 2) matrix and write it to appropriate places */
	phx_scalar_product(&I33, dt);
	phx_scalar_product(&I33, 0.5);
	phx_writesubmatrix(F, ixx, iax, &I33); /* dfx / dv */

	/* write differentials matrices */
	phx_writesubmatrix(F, iqa, iqa, &dfqdq);
	phx_writesubmatrix(F, iqa, iwx, &dfqdw);

	F->data[ihz * F->cols + ihz] = 1;
	F->data[ihz * F->cols + ivz] = dt;
	F->data[ihv * F->cols + ihv] = 1;
}


/* initialization function for IMU update step matrices values */
update_engine_t imuUpdateInitializations(phmatrix_t *H, phmatrix_t *R)
{
	/* matrix initialization */
	phx_newmatrix(H, IMUMEAS_ROWS, STATE_ROWS);
	phx_newmatrix(R, IMUMEAS_ROWS, IMUMEAS_ROWS);

	/* init of measurement noise matrix R */
	R->data[R->cols * imax + imax] = init_values.R_acov;
	R->data[R->cols * imay + imay] = init_values.R_acov;
	R->data[R->cols * imaz + imaz] = init_values.R_acov;

	R->data[R->cols * imwx + imwx] = init_values.R_wcov;
	R->data[R->cols * imwy + imwy] = init_values.R_wcov;
	R->data[R->cols * imwz + imwz] = init_values.R_wcov;

	R->data[R->cols * immx + immx] = init_values.R_mcov;
	R->data[R->cols * immy + immy] = init_values.R_mcov;
	R->data[R->cols * immz + immz] = init_values.R_mcov;

	R->data[R->cols * imqa + imqa] = init_values.R_qcov;
	R->data[R->cols * imqb + imqb] = init_values.R_qcov;
	R->data[R->cols * imqc + imqc] = init_values.R_qcov;
	R->data[R->cols * imqd + imqd] = init_values.R_qcov;

	return setupImuUpdateEngine(H, R);
}


/* initialization function for barometer update step matrices values */
update_engine_t baroUpdateInitializations(phmatrix_t *H, phmatrix_t *R)
{
	phx_newmatrix(H, BAROMEAS_ROWS, STATE_ROWS);
	phx_newmatrix(R, BAROMEAS_ROWS, BAROMEAS_ROWS);

	//R->data[R->cols * impx + impx] = init_values.R_pcov;
	R->data[R->cols * imhz + imhz] = init_values.R_hcov;
	R->data[R->cols * imxz + imxz] = init_values.R_xzcov;
	R->data[R->cols * imhv + imhv] = init_values.R_hvcov;
	R->data[R->cols * imvz + imvz] = init_values.R_vzcov;

	return setupBaroUpdateEngine(H, R);
}


/* initialization of prediction step matrix values */
state_engine_t init_prediction_matrices(phmatrix_t *state, phmatrix_t *state_est, phmatrix_t *cov, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt)
{
	/* matrix initialization */
	phx_newmatrix(state, STATE_ROWS, STATE_COLS);
	phx_newmatrix(state_est, STATE_ROWS, STATE_COLS);

	phx_newmatrix(cov, STATE_ROWS, STATE_ROWS);
	phx_newmatrix(cov_est, STATE_ROWS, STATE_ROWS);
	phx_newmatrix(F, STATE_ROWS, STATE_ROWS);
	phx_newmatrix(Q, STATE_ROWS, STATE_ROWS);

	init_state_vector(state);
	init_cov_vector(cov);

	phx_zeroes(Q);

	/* acceleration process noise different for vertical and horizontal because different measurements are performed and different smoothing is neccessary */
	Q->data[Q->cols * iax + iax] = Q->data[Q->cols * iay + iay] = init_values.Q_ahoricov;
	Q->data[Q->cols * iaz + iaz] = init_values.Q_avertcov;

	Q->data[Q->cols * iwx + iwx] = Q->data[Q->cols * iwy + iwy] = Q->data[Q->cols * iwz + iwz] = init_values.Q_wcov;
	Q->data[Q->cols * imx + imx] = Q->data[Q->cols * imy + imy] = Q->data[Q->cols * imz + imz] = init_values.Q_mcov;
	Q->data[Q->cols * iqa + iqa] = init_values.Q_qcov;
	Q->data[Q->cols * iqb + iqb] = init_values.Q_qcov;
	Q->data[Q->cols * iqc + iqc] = init_values.Q_qcov;
	Q->data[Q->cols * iqd + iqd] = init_values.Q_qcov;
	Q->data[Q->cols * ihz + ihz] = init_values.Q_hcov;
	Q->data[Q->cols * ixz + ixz] = init_values.Q_hcov;
	Q->data[Q->cols * ihv + ihv] = init_values.Q_pvcov;

	return (state_engine_t) {
		.state = state,
		.state_est = state_est,
		.cov = cov,
		.cov_est = cov_est,
		.F = F,
		.Q = Q,
		.getJacobian = calcPredictionJacobian,
		.estimateState = calculateStateEstimation
	};
}
