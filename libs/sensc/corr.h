#ifndef _LIBCALIB_CORR_H_
#define _LIBCALIB_CORR_H_

#include <calibcore.h>

/* registering new calibration/correction procedure */
extern void corr_register(calib_t *c); 


extern hmap_t *corr_hashmapGet(void);

#endif