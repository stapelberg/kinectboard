#ifndef _MEDIAN_H
#define _MEDIAN_H

/* For uchar4 */
#include <cuda_gl_interop.h>

void median_filter_init(void);
void median_filter(uint16_t *host_depth, uchar4 *gpu_output);

#endif
