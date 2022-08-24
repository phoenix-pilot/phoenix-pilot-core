/*
 * Phoenix-Pilot
 *
 * Tools for matrix library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "tools.h"


#define SMALL_SHIFT 1
#define BIG_SHIFT   1234


void algebraTests_getRowColNum(matrix_t *M, int *rows, int *cols)
{
	if (M->transposed) {
		*rows = M->cols;
		*cols = M->rows;
	}
	else {
		*rows = M->rows;
		*cols = M->cols;
	}
}


void algebraTests_fillWithVal(matrix_t *M, float val)
{
	int rowsNum, colsNum, row, col;

	algebraTests_getRowColNum(M, &rowsNum, &colsNum);

	for (row = 0; row < rowsNum; row++) {
		for (col = 0; col < colsNum; col++) {
			*matrix_at(M, row, col) = val;
		}
	}
}
