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

#ifndef PHOENIX_LIBALGEB_LMA_H
#define PHOENIX_LIBALGEB_LMA_H

#include <matrix.h>
#include <stdbool.h>


#define LMALOG_NONE          0
#define LMALOG_DELTA         (1 << 0)
#define LMALOG_PARAMS        (1 << 1)
#define LMALOG_LAMBDA        (1 << 2)
#define LMALOG_RESIDUUM      (1 << 3)
#define LMALOG_USER_JACOBIAN (1 << 4)
#define LMALOG_USER_RESIDUUM (1 << 5)


/*
* Function pointer to residuum jacobian solver for one sample point.
* Calculates d(r(V,P))/d(P) in the point (V, P) where:
* - r is a residuum function
* - V is variables vector for current sample point
* - P is parameters vector in current iteration
* - J is output jacobian vector (set to zeroes by default)
*
* This function shall return -1 if calculation is not possible for given data which halts the LMA.
* Otherwise returns 0;
*/
typedef int (*lmaJacobian)(const matrix_t *P, const matrix_t *V, matrix_t *J, bool log);

/*
* Function pointer to residuum solver.
* Returns function residuum r = r(V, P) where:
* - V is variables vector for current sample point
* - P is parameters vector in current iteration
*
* This function shall return -1 if calculation is not possible for given data which halts the LMA.
* Otherwise returns 0;
*/
typedef int (*lmaResiduum)(const matrix_t *P, const matrix_t *V, float *res, bool log);


/*
* Pointer to initial guess provider function.
* This function shall fill parameters P vector with initial guess values
*/
typedef void (*lmaGuess)(matrix_t *P);


typedef struct {
	unsigned int nvars;    /* number of variables of funcition `f` */
	unsigned int nparams;  /* number of parameters of function `f` */
	unsigned int nsamples; /* size of the sample table */

	matrix_t samples;
	matrix_t jacobian;
	matrix_t residua;
	matrix_t residuaCandidate;
	matrix_t delta;

	/* user passed matrices */
	matrix_t paramsVec;
	matrix_t paramsCandVec;
	matrix_t varsVec;
	matrix_t jacobianVec;

	/* helper matrices */
	matrix_t helpPxP[2];
	float *invBuf;
	unsigned int invBufLen;

	lmaJacobian solveJ;
	lmaResiduum solveR;
	lmaGuess guess;

} fit_lma_t;


/*
* Performs fitting operation on `lma` with at most `maxSteps` LMA iterations.
* Logs according to logFlags (multiple may be passed using bitewise OR).
* On success returns number of iterations taken, on error returns -1.
*/
extern int lma_fit(unsigned int maxSteps, fit_lma_t *lma, unsigned int logFlags);


/* Deinitializes all memory allocated by `lma` */
extern int lma_done(fit_lma_t *lma);


/*
 * Initializes `lma` structure for fitting of function which has `nvars` variables and
 * `nparams` parameters (fitted values). Fitting will be done for `n` samples.
 * User must provide pointers to jacobian, residuum and initial guess solving functions.
 * */
extern int lma_init(unsigned int nvars, unsigned int nparams, unsigned int n, lmaJacobian solveJ, lmaResiduum solveR, lmaGuess guess, fit_lma_t *lma);

#endif
