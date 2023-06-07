/*
 * Phoenix-Pilot
 *
 * Gettime function
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifdef __phoenix__

#include <sys/time.h>

#else


#include <time.h>

extern int gettime(time_t *raw, time_t *offs);


#endif
