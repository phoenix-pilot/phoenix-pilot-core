/*
 * Phoenix-Pilot
 *
 * Max log volume estimation
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdlib.h>


/* Starts an estimation */
extern void maxLog_start(void);


/* End estimation */
extern void maxLog_end(void);


/* Report action of writing to file `n` bytes to file */
extern void maxLog_writeReport(size_t n);


/* Should be invoked just before functions, which suspends thread execution */
extern void maxLog_sleepReport(void);


/* Should be invoked just after function, which suspends thread execution */
extern void maxLog_wakeUpReport(void);


/* Prints to stdout estimation results. Must be invoked after `maxLog_end` */
extern void maxLog_resultsPrint(void);
