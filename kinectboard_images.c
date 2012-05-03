// vim:ts=4:sw=4:expandtab

#include "kinectboard_images.h"

#if SDL_BYTEORDER == SDL_BIG_ENDIAN
#define RMASK 0xff000000
#define GMASK 0x00ff0000
#define BMASK 0x0000ff00
#else
#define RMASK 0x000000ff
#define GMASK 0x0000ff00
#define BMASK 0x00ff0000
#endif

CIRCLEQ_HEAD(image_head, kb_image) image_head =
    CIRCLEQ_HEAD_INITIALIZER(image_head);


/* ******************************************************************* */
/* KB (RGB) Image */

void kb_image_create(const char *label, uint8_t **buffer) {
    kb_image *new_img = calloc(sizeof(kb_image), 1);
    new_img->label = label;
    new_img->buffer = buffer;
    new_img->surface = SDL_CreateRGBSurface(SDL_SWSURFACE, 640, 480, 24, RMASK, GMASK, BMASK, 0);
    /* Wir unterscheiden hier nur zwischen 0 und 1 Elementen. Es werden nur die
     * ersten beiden Bilder gerendert standardmäßig (kein Platz mehr auf dem
     * Bildschirm). Wenn der Nutzer scrolled, werden die areas ohnehin neu
     * berechnet. */
    if (!CIRCLEQ_EMPTY(&image_head))
        new_img->area = (SDL_Rect){ 640, 0, 640, 480 };
    else new_img->area = (SDL_Rect){ 0, 0, 640, 480 };

    CIRCLEQ_INSERT_TAIL(&image_head, new_img, image);
}

void kb_images_render(SDL_Surface *screen) {
    int i = 0;
    kb_image *img;
    printf("rendering.\n");
    CIRCLEQ_FOREACH(img, &image_head, image) {
        printf("rendering %s\n", img->label);
        memcpy(img->surface->pixels, *(img->buffer), 640 * 480 * 3);
        SDL_BlitSurface(img->surface, NULL, screen, &(img->area));
        if (++i == 2)
            break;
    }
}
