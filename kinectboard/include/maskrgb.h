#ifndef _MASKRGB_H
#define _MASKRGB_H

/* For uchar4 */
#include <cuda_gl_interop.h>

void mask_rgb_init(void);
void mask_rgb(uchar4 *gpu_glow_output, uint8_t *rgb_image, uchar4 *gpu_output, uchar4 *gpu_raw_rgb_output, uchar4 *gpu_cont_rgb_output, float4 reference_color, float filter_distance, uint8_t *gpu_overlay_on);
void mask_rgb_clear_cont(void);

#endif
