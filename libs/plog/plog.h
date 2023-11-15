/*
 * Phoenix-Pilot
 *
 * Generic Phoenix-Pilot binary logging utility
 *
 * The actual implementation of collection logs uses a two buffers.
 * The log producer saves data to the first one as long as there is enough space for new logs.
 * When there is not enough free memory, the producer marks it as dirty and starts using the next one.
 * A separate thread saves dirty buffers to a file and clears dirty flag.
 * This approach allows for collecting logs without blocking the main thread due to potentially
 * time-consuming file writes.
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 *
 */

#ifndef _PLOG_WRITER_
#define _PLOG_WRITER_

#include <unistd.h>
#include <time.h>
#include <stdint.h>


#define LOG_ID_SIZE         sizeof(uint32_t)
#define LOG_IDENTIFIER_SIZE sizeof(char)
#define LOG_TIMESTAMP_SIZE  sizeof(time_t)
#define LOG_PREFIX_SIZE     (LOG_ID_SIZE + LOG_IDENTIFIER_SIZE + LOG_TIMESTAMP_SIZE)

/*
 * Potentially slower implementation, but with no possibility to lose logs.
 *
 * By default log module priorities execution speed over logs consistency.
 * It is possible that not all logs will be stored in result file.
 * It that case appropriate warning is added to file.
 */
#define PLOG_STRICT_MODE (1 << 30)

typedef struct plog_ctx_t plog_t;

/*
 * Writes binary message `msg` of length `msgLen` prefixed with `logIndicator` and then `timestamp`
 */
extern int plog_write(plog_t * const ctx, const void *msg, size_t msgLen, char logIndicator, time_t timestamp);


/*
 * Deinitializes `plog_t` structure.
 * Deallocates `ctx` as if `free()` was used.
 */
extern int plog_done(plog_t * const ctx);


/*
 * Initializes plog module for `flags` log messages and `path` destination file.
 * Returns pointer to allocated `plog_t` structure or NULL on fail.
 */
extern plog_t *plog_init(const char *path, uint32_t flags);

#endif
