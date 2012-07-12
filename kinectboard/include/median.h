#ifndef _MEDIAN_H
#define _MEDIAN_H

/* For uchar4 */
#include <cuda_gl_interop.h>

void median_filter_init(void);
void median_filter(uint16_t *host_depth, uchar4 *gpu_output);
void median_mask(bool calibration, uchar4 *gpu_median_output, uchar4 *gpu_output);
void median_clear_calibration(void);

#endif
