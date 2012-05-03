// vim:ts=4:sw=4:expandtab
/*
 * Kinect Board Images 
 */

#ifndef __KINECTBOARD_IMAGES
#define __KINECTBOARD_IMAGES
 
 #include <sys/types.h>
 #include <SDL/SDL.h>
 #include <stdbool.h>

#include "queue.h"
 
/*
 * All the (RGB) Images are ordered in a row: The raw image is on the very
 * left, and each processing step (median filtering, glowing, etc.) is one step
 * further to the right.
 *
 * The user can change his view on this list by either using the arrow keys and
 * therefore scrolling his window, or by re-arranging items in the list (not
 * yet implemented).
 *
 */
typedef struct kb_image {
    const char *label;
    uint8_t **buffer;

    SDL_Surface *surface;
    SDL_Rect area;

    CIRCLEQ_ENTRY(kb_image) image;
} kb_image;

void kb_image_create(const char *label, uint8_t **buffer);

void kb_images_render(SDL_Surface *screen);

void kb_images_scroll_left(void);
void kb_images_scroll_right(void);

#endif
