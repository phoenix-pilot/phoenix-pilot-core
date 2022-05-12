/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * transition and observation matrices formulas
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
#include <time.h>
#include <math.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>


void calcPredictionJacobian(phmatrix_t *state, phmatrix_t *F, float dt)
{
	float dt2 = dt / 2, press_B; /* helper value */

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

}


void calcImuJacobian(phmatrix_t *state, phmatrix_t *H, float dt)
{
	float I33_data[9] = { 0 };
	phmatrix_t I33 = { .rows = 3, .cols = 3, .transposed = 0, .data = I33_data };
	phx_diag(&I33);

	phx_zeroes(H);
	phx_writesubmatrix(H, imax, iax, &I33);
	phx_writesubmatrix(H, imwx, iwx, &I33);
	phx_writesubmatrix(H, immx, imx, &I33);
	/* using I33 and one direct write to write I44 */
	phx_writesubmatrix(H, imqa, iqa, &I33);
	H->data[H->cols * imqd + iqd] = 1;
}


void calcBaroJacobian(phmatrix_t *state, phmatrix_t *H, float dt)
{
	H->data[H->cols * imhz + ihz] = 1;
	H->data[H->cols * imxz + ixz] = 1;

}
