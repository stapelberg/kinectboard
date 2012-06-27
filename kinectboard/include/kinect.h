#ifndef _KINECT_H
#define _KINECT_H

void kinect_init(void);

uint16_t *take_depth_image(void);
void done_depth_image(void);

#endif
