/*
 * Phoenix-RTOS
 *
 * quad-control logging utility
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _QUADCONTROL_LOG_H_
#define _QUADCONTROL_LOG_H_


/* Log to standard output if logging is enabled */
extern void log_print(const char *format, ...);


/* enables logging */
extern void log_enable(void);


/* disables logging */
extern void log_disable(void);

#endif
