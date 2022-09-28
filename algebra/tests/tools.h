/*
 * Phoenix-Pilot
 *
 * Tools for algebra library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef MATRIX_TEST_TOOLS_H
#define MATRIX_TEST_TOOLS_H

#include <matrix.h>
#include <vec.h>
#include <quat.h>

#include <unity_fixture.h>


/* ##############################################################################
 * ---------------------        defines used in tests       ---------------------
 * ############################################################################## */

/* --------------------------        matrix tests       ------------------------- */

/* Defines for `matrix_bufAlloc` results */
#define MAT_BUF_ALLOC_OK   0
#define MAT_BUF_ALLOC_FAIL -1

/* Defines for `matrix_product` results */
#define MAT_PRODUCT_OK   0
#define MAT_PRODUCT_FAIL -1

/* Defines for `matrix_sandwitch` results */
#define MAT_SANDWITCH_OK   0
#define MAT_SANDWITCH_FAIL -1

/* Defines for `matrix_add` results */
#define MAT_ADD_OK   0
#define MAT_ADD_FAIL -1

/* Defines for `matrix_sub` results */
#define MAT_SUB_OK   0
#define MAT_SUB_FAIL -1

/* Defines for `matrix_writeSubmatrix` results */
#define MAT_WRITE_SUBMAT_OK   0
#define MAT_WRITE_SUBMAT_FAIL -1

/* Define for `matrix_cmp` results */
#define MAT_CMP_OK 0

/* Defines for `matrix_inv` results */
#define MAT_INV_OK   0
#define MAT_INV_FAIL -1

/* --------------------------        vector tests       ------------------------- */

/* Define for `vec_cmp` results */
#define VEC_CMP_OK 0

/* -----------------------        quaternions tests       ----------------------- */

/* Define for `vec_cmp` results */
#define QUAT_CMP_OK 0

/* ------------------------        general defines       ------------------------ */

/* Defines for functions that validate object parameters */
#define CHECK_OK   0
#define CHECK_FAIL -1

/* Must be at least 1 */
#define SMALL_SHIFT 1

/* Must be bigger than SMALL_SHIFT */
#define BIG_SHIFT 1234

#define POS_SCALAR 2.5
#define NEG_SCALAR -3.75

/* ##############################################################################
 * ---------------------        macros used in tests       ----------------------
 * ############################################################################## */

/* Checks if `expected` is identical to `actual`. Difference between `matrix_cmp` is that .transposed flag must be equal in both matrices */
#define TEST_ASSERT_EQUAL_MATRIX(expected, actual) \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).transposed, (actual).transposed, "Transposition flag is not equal"); \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).rows, (actual).rows, "Different rowspan"); \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).cols, (actual).cols, "Different colspan"); \
	TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE((expected).data, (actual).data, (actual).rows *(actual).cols, "Different matrix element");

/* Checks if every element of `actual` is within +/- `delta` of the value from `expected` */
#define TEST_ASSERT_MATRIX_WITHIN(delta, expected, actual) \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).transposed, (actual).transposed, "Transposition flag is not equal"); \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).rows, (actual).rows, "Different rowspan"); \
	TEST_ASSERT_EQUAL_UINT_MESSAGE((expected).cols, (actual).cols, "Different colspan"); \
	test_assert_float_array_within((delta), (expected).data, (actual).data, (actual).rows *(actual).cols, __LINE__, "Different matrix element");

#define TEST_ASSERT_EQUAL_QUAT(expected, actual) \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).a, (actual).a, "Different real part of quaternion"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).i, (actual).i, "Different `i` part of quaternion"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).i, (actual).i, "Different `j` part of quaternion"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).i, (actual).i, "Different `k` part of quaternion");

#define TEST_ASSERT_EQUAL_VEC(expected, actual) \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).x, (actual).x, "Different `x` part of vectors"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).y, (actual).y, "Different `y` part of vectors"); \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE((expected).z, (actual).z, "Different `z` part of vectors");

#define TEST_ASSERT_PERPENDICULAR_VEC(delta, vector1, vector2) \
	TEST_ASSERT_FLOAT_WITHIN_MESSAGE(delta, 0.0, vec_dot(&(vector1), &(vector2)), "Vectors are not perpendicular to each other");

#define TEST_ASSERT_UNIT_VEC(vector) \
	TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1.0, vec_len(&(vector)), "Length of vector is not equal to 1")

#define TEST_ASSERT_QUAT_WITHIN(delta, expected, actual) \
	TEST_ASSERT_FLOAT_WITHIN_MESSAGE((delta), (expected).a, (actual).a, "Different real part of quaternion"); \
	TEST_ASSERT_FLOAT_WITHIN_MESSAGE((delta), (expected).i, (actual).i, "Different `i` part of quaternion"); \
	TEST_ASSERT_FLOAT_WITHIN_MESSAGE((delta), (expected).i, (actual).i, "Different `j` part of quaternion"); \
	TEST_ASSERT_FLOAT_WITHIN_MESSAGE((delta), (expected).i, (actual).i, "Different `k` part of quaternion");


/* ##############################################################################
 * ----------------------        assertions helpers       -----------------------
 * ############################################################################## */


/* This function is used in definition of TEST_ASSERT_MATRIX_WITHIN macro */
extern void test_assert_float_array_within(float delta, float *expected, float *actual, unsigned int elemNum, int line, char *message);


/* ##############################################################################
 * ---------------------        matrix modification       -----------------------
 * ############################################################################## */


/* Fill matrix buffer with n-length array vals. If n=1 whole matrix is filled with vals[0] */
extern void algebraTests_buffFill(matrix_t *M, const float *vals, unsigned int n);


/* Create and fill matrix with n-length array vals. If n=1 whole matrix is filled with vals[0]. M must be uninitialized */
extern int algebraTests_createAndFill(matrix_t *M, unsigned int rows, unsigned int cols, const float *vals, unsigned int n);


/* Copies src to des. Destination matrix have to be uninitiated */
extern int algebraTests_matrixCopy(matrix_t *des, matrix_t *src);


/* Transposes matrix by rearranging its elements. Does not change M->transposed */
extern int algebraTests_realTrp(matrix_t *M);


/* Transpose `M` using matrix_trp() and swap its memory so it is also transposed. Does not change `M` in mathematical meaning */
extern int algebraTests_transposeSwap(matrix_t *M);


/* ##############################################################################
 * ------------------------        matrix checks       --------------------------
 * ############################################################################## */


/* Function checks if every matrix element is zero */
extern int algebraTests_matrixZeroesCheck(matrix_t *A);


/* Function checks if matrix_at return a NULL, when we are trying to get matrix element outside matrix */
extern int algebraTests_invalidSeekCheck(matrix_t *M);


/* Checks if M2 corresponds to M1 physically transposed (with swapped data in buffer, and switched cols and rows) */
extern int algebraTests_diagCheck(matrix_t *M);


/* Checks if M2 is equal to M1 transposed */
extern int algebraTests_dataTrpCheck(matrix_t *M1, matrix_t *M2);


/* Checks if M1 is identical to M2. Different between matrix_cmp is that .transposed flag must be equal in both matrices */
extern int algebraTest_equalMatrix(const matrix_t *M1, const matrix_t *M2);


/* Checks if `M` is equal to `dst` with `src` printed in `col` and `row`. All matrices have to be non-transposed*/
extern int algebraTests_submatCheck(const matrix_t *dst, unsigned int row, unsigned int col, const matrix_t *src, const matrix_t *M);

#endif
