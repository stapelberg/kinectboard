#ifndef _GLOW_H
#define _GLOW_H

/* For uchar4 */
#include <cuda_gl_interop.h>

void glow_filter_init(void);
void glow_filter(uchar4 *gpu_median_masked, uchar4 *gpu_output, uint16_t glow_start, uint16_t glow_end);

#endif
