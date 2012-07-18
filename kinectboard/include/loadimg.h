#ifndef _LOADIMG_H
#define _LOADIMG_H

/* For uchar4 */
#include <cuda_gl_interop.h>

void loadimg_init(void);
void loadimg_convert(uint8_t *host_pixels, uint8_t *gpu_buffer);

#endif
