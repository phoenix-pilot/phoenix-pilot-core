#ifndef _LIBCALIB_CALIB_H_
#define _LIBCALIB_CALIB_H_

#include <calibcore.h>

/* registering new calibration/correction procedure */
extern void calib_register(calib_t *c); 


extern hmap_t *calib_hashmapGet(void);


#endif
