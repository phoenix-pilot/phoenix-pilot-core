#ifndef _LIBCALIB_CORR_H_
#define _LIBCALIB_CORR_H_

#include <calibcore.h>

/* registering new calibration/correction procedure */
extern void corr_register(calib_t *c);

/* starts the correction recalculation thread */
int corr_run(void);

/* initializes corrections and recalculation thread variables */
int corr_init(const char *path);

/* deinitializes correction module and stops the thread */
void corr_done(void);


#endif