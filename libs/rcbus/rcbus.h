/*
 * Phoenix-Pilot
 *
 * Communication bus library supports I-Bus & S-Bus
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <time.h>


#define MAX_CHANNEL_VALUE 1000
#define MIN_CHANNEL_VALUE 0


typedef enum { rc_typeIbus = 0, rc_typeSbus } rcbus_type_t;


typedef struct {
	size_t channelsCnt;
	uint16_t *channels;
} rcbus_msg_t;


typedef void (*RcMsgHandler)(const rcbus_msg_t *msg);


/* New thread for reading data is launched. If the correct packet is received, the handler is invoked.
 * The timeout [ms] determines time for blocking the read function. */
extern int rcbus_run(RcMsgHandler handler, time_t timeout);


/* Close reading thread */
extern int rcbus_stop(void);


/* Close communication with a device. */
extern void rcbus_done(void);


/* Initialize communication with a device on specific protocol. */
extern int rcbus_init(const char *devPath, rcbus_type_t type);
