/*
 * Phoenix-Pilot
 *
 * EKF test runner result checking routines
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef RESULT_CHECK_H
#define RESULT_CHECK_H


/* Checks if final state from ekf logs matches expected one */
int ekftests_resultCheck(const char *logFile, const char *expectedResultFile);


#endif
