/*
 * Phoenix-Pilot
 *
 * lma - generic Levenberg–Marquardt algorithm implementation
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "lma.h"


#define LMA_LAMBDA_START   1.f  /* Starting value of lambda parameter */
#define LMA_LAMBDA_REWARD  0.1f /* Lambda parameter is ultiplied by it if better solution was found */
#define LMA_LAMBDA_PENALTY 10.f /* Lambda parameter is ultiplied by it if worse solution was found */


static void lma_log(unsigned int type, unsigned int flags, fit_lma_t *lma, unsigned int step, float lambda, float residuum)
{
	int currLog = type & flags;
	int i;

	if (currLog == LMALOG_NONE) {
		return;
	}

	switch (currLog) {
		case LMALOG_DELTA:
			fprintf(stdout, "(%u)delta: (", step);
			for (i = 0; i < lma->nparams; i++) {
				fprintf(stdout, "%f ", MATRIX_DATA(&lma->delta, i, 0));
			}
			fprintf(stdout, ")\n");
			break;

		case LMALOG_PARAMS:
			fprintf(stdout, "params: (");
			for (i = 0; i < lma->nparams; i++) {
				fprintf(stdout, "%f ", MATRIX_DATA(&lma->paramsVec, 0, i));
			}
			fprintf(stdout, ")\n");
			break;

		case LMALOG_LAMBDA:
			fprintf(stdout, "lambda: %e\n", lambda);
			break;

		case LMALOG_RESIDUUM:
			fprintf(stdout, "res: %e\n", residuum);
			break;

		default:
			break;
	}
}


/*
* Solves the delta equation: lma->delta = inv(trp(J) * J + lambda * I) (trp(J) * R)
*
* This function manipulates sizes of matrices on its own to minimize the amount of initialized memory needed for calculations.
* Invalidates contents of: lma->helpPxP[0], lma->helPxP[1], lma->helpPx1 and lma->invBuf.
*/
static int lma_solveDelta(float lambda, fit_lma_t *lma)
{
	/* This matrix shares memory with lma->jacobian but is transposed. Made it const so it isn`t changed by accident */
	const matrix_t Jt = {
		.data = lma->jacobian.data,
		.transposed = 1,
		.rows = lma->jacobian.rows,
		.cols = lma->jacobian.cols
	};

	matrix_t localPx1;
	int i;

	/* Storing inverse of (J^t * J + lambda * I)^-1 into helpPxP[1] matrix */
	matrix_prod(&Jt, &lma->jacobian, &lma->helpPxP[0]);
	for (i = 0; i < lma->nparams; i++) {
		MATRIX_DATA(&lma->helpPxP[0], i, i) += lambda;
	}
	matrix_inv(&lma->helpPxP[0], &lma->helpPxP[1], lma->invBuf, lma->invBufLen);

	/* lma->helpPxP[0] is not used anymore. Aliasing it with localPx1 of smaller size */
	localPx1 = lma->helpPxP[0];
	localPx1.cols = 1;

	matrix_prod(&Jt, &lma->residua, &localPx1);
	matrix_prod(&lma->helpPxP[1], &localPx1, &lma->delta);

	return 0;
}


/*
* Writes full jacobian of residuals with respect to parameters into lma->jacobian matrix.
* Invalidates contents of lma->varsVec and lma->jacovanVec.
*/
static int lma_solveJacobian(fit_lma_t *lma, bool log)
{
	unsigned int smpl, var;

	for (smpl = 0; smpl < lma->nsamples; smpl++) {
		/* Write current sample to varsVec */
		for (var = 0; var < lma->nvars; var++) {
			MATRIX_DATA(&lma->varsVec, 0, var) = MATRIX_DATA(&lma->samples, smpl, var);
		}

		/* Solve Jacobian row with current variables and parameters */
		if (lma->solveJ(&lma->paramsVec, &lma->varsVec, &lma->jacobianVec, log) < 0) {
			fprintf(stderr, "Error in jacobian solver\n");
			return -1;
		}

		/* Write jacobian vector into full jacobian matrix */
		matrix_writeSubmatrix(&lma->jacobian, smpl, 0, &lma->jacobianVec);
	}

	return 0;
}


/*
* Writes full residua values into R matrix
* Invalidates contents of lma->varsVec.
*/
static float lma_solveResidua(matrix_t *R, matrix_t *P, fit_lma_t *lma, float *out, bool log)
{
	unsigned int smpl, var;
	double sum = 0; /* use double for sum to loose less accuracy on addition */
	float res;

	/* Calculate current redidua and sum of squares */
	for (smpl = 0; smpl < lma->nsamples; smpl++) {
		for (var = 0; var < lma->nvars; var++) {
			MATRIX_DATA(&lma->varsVec, 0, var) = MATRIX_DATA(&lma->samples, smpl, var);
		}

		/* solve residuum problem for each sample and with current parameters */
		if (lma->solveR(P, &lma->varsVec, &res, log) < 0) {
			fprintf(stderr, "Error in residuum solver\n");
			return -1;
		}

		/* Saving residuum as negative as later in `lma_solveDelta` it should be taken as negative */
		MATRIX_DATA(R, smpl, 0) = -res; /* save current residuum */
		sum += (double)res * res;       /* update sum of squares */
	}

	*out = sum;

	return 0;
}


/*
* LMA algorithm pseudocode:
*
* 1) P = initialGuess()
* 2) (R,r) = sumOfRes(V, P)
* 3) lambda = lambda0
* 4) while !STOP:
* 5)    J = solveJ(V, P)
* 6)    delta = solveDelta(J, R, lambda)
* 7)    P2 = P + lambda
* 8)    (RTmp, rTmp) = sumOfRes(V, P2)
* 9)    if (r2 < r):
*            R = R2, P = P2, lambda /= 10
*       else:
*            lambda *= 10
*/
int lma_fit(unsigned int maxSteps, fit_lma_t *lma, unsigned int logFlags)
{
	const bool logUserResiduum = ((logFlags & LMALOG_USER_RESIDUUM) != LMALOG_NONE);
	const bool logUserJacobian = ((logFlags & LMALOG_USER_JACOBIAN) != LMALOG_NONE);

	float r, rCand;
	float lambda;
	unsigned int i;
	matrix_t mTmp;

	lma->guess(&lma->paramsVec); /* (1) */

	/* (2) */
	if (lma_solveResidua(&lma->residua, &lma->paramsVec, lma, &r, logUserResiduum) < 0) {
		fprintf(stderr, "lma: residuum solver error\n");
		return -1;
	}

	/* (3) */
	lambda = LMA_LAMBDA_START;

	/* (4) */
	for (i = 0; i < maxSteps; i++) {
		lma_log(LMALOG_PARAMS, logFlags, lma, i, lambda, r);
		lma_log(LMALOG_DELTA, logFlags, lma, i, lambda, r);
		lma_log(LMALOG_LAMBDA, logFlags, lma, i, lambda, r);
		lma_log(LMALOG_RESIDUUM, logFlags, lma, i, lambda, r);

		/* (5) */
		if (lma_solveJacobian(lma, logUserJacobian) < 0) {
			fprintf(stderr, "lma: jacobian solver error\n");
			return -1;
		}

		/* (6) */
		lma_solveDelta(lambda, lma);

		/* (7) */
		matrix_trp(&lma->delta);
		matrix_add(&lma->paramsVec, &lma->delta, &lma->paramsCandVec);
		matrix_trp(&lma->delta);

		/* (8) */
		if (lma_solveResidua(&lma->residuaCandidate, &lma->paramsCandVec, lma, &rCand, logUserResiduum) < 0) {
			fprintf(stderr, "lma: residuum solver error\n");
			return -1;
		}

		/* (9) */
		if (rCand < r) {
			mTmp = lma->residua;
			lma->residua = lma->residuaCandidate;
			lma->residuaCandidate = mTmp;

			mTmp = lma->paramsVec;
			lma->paramsVec = lma->paramsCandVec;
			lma->paramsCandVec = mTmp;

			lambda *= LMA_LAMBDA_REWARD;
			r = rCand;
		}
		else {
			lambda *= LMA_LAMBDA_PENALTY;
		}
	}

	return i;
}


int lma_done(fit_lma_t *lma)
{
	matrix_bufFree(&lma->samples);
	matrix_bufFree(&lma->jacobian);
	matrix_bufFree(&lma->residua);
	matrix_bufFree(&lma->residuaCandidate);
	matrix_bufFree(&lma->delta);

	matrix_bufFree(&lma->paramsVec);
	matrix_bufFree(&lma->paramsCandVec);
	matrix_bufFree(&lma->varsVec);
	matrix_bufFree(&lma->jacobianVec);

	matrix_bufFree(&lma->helpPxP[0]);
	matrix_bufFree(&lma->helpPxP[1]);

	free(lma->invBuf);
	lma->invBuf = NULL;
	lma->invBufLen = 0;

	return 0;
}


int lma_init(unsigned int nvars, unsigned int nparams, unsigned int n, lmaJacobian solveJ, lmaResiduum solveR, lmaGuess guess, fit_lma_t *lma)
{
	/* Setting tmp to zeroes so unallocated pointers are NULL */
	fit_lma_t tmp = { 0 };
	int err = 0;

	tmp.nvars = nvars;
	tmp.nparams = nparams;
	tmp.nsamples = n;

	tmp.solveJ = solveJ;
	tmp.solveR = solveR;
	tmp.guess = guess;

	err |= matrix_bufAlloc(&tmp.samples, tmp.nsamples, tmp.nvars);
	err |= matrix_bufAlloc(&tmp.jacobian, tmp.nsamples, tmp.nparams);
	err |= matrix_bufAlloc(&tmp.residua, tmp.nsamples, 1);
	err |= matrix_bufAlloc(&tmp.residuaCandidate, tmp.nsamples, 1);
	err |= matrix_bufAlloc(&tmp.delta, tmp.nparams, 1);

	err |= matrix_bufAlloc(&tmp.paramsVec, 1, tmp.nparams);
	err |= matrix_bufAlloc(&tmp.paramsCandVec, 1, tmp.nparams);
	err |= matrix_bufAlloc(&tmp.varsVec, 1, tmp.nvars);
	err |= matrix_bufAlloc(&tmp.jacobianVec, 1, tmp.nparams);

	err |= matrix_bufAlloc(&tmp.helpPxP[0], tmp.nparams, tmp.nparams);
	err |= matrix_bufAlloc(&tmp.helpPxP[1], tmp.nparams, tmp.nparams);

	tmp.invBufLen = 2 * tmp.nparams * tmp.nparams;
	tmp.invBuf = calloc(tmp.invBufLen, sizeof(float));

	if (tmp.invBuf == NULL || err != 0) {
		lma_done(&tmp);
		return -1;
	}

	*lma = tmp;

	return 0;
}
