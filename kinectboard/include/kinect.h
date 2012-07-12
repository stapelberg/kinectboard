#ifndef _KINECT_H
#define _KINECT_H

void kinect_init(void);
void kinect_shutdown(void);

uint16_t *take_depth_image(void);
uint8_t *take_rgb_image(void);
void done_depth_image(void);
void done_rgb_image(void);

#endif
