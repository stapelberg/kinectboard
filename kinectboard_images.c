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

static int startidx = 0;

static enum {
    A_FROM_RIGHT_TO_LEFT,
    A_FROM_LEFT_TO_RIGHT
} animation_direction;

static int animation_step = 0;
int ANIMATION_ONE_STEP = 30;

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
    int i = 0, cnt = 0;
    SDL_Rect area;
    kb_image *img;
    printf("rendering.\n");
    CIRCLEQ_FOREACH(img, &image_head, image) {
        if (i++ < startidx)
            continue;
        printf("rendering %s\n", img->label);
        area = img->area;
        area.x += animation_step;
        if (animation_direction == A_FROM_RIGHT_TO_LEFT && animation_step > 0) {
            animation_step -= ANIMATION_ONE_STEP;
            if (animation_step < 0)
                animation_step = 0;
        }

        if (animation_direction == A_FROM_LEFT_TO_RIGHT && animation_step < 0) {
            animation_step += ANIMATION_ONE_STEP;
            if (animation_step >= 0)
                animation_step = 0;
        }

        memcpy(img->surface->pixels, *(img->buffer), 640 * 480 * 3);
        SDL_BlitSurface(img->surface, NULL, screen, &area);
        if (++cnt == 2)
            break;
    }
}

static void fix_areas(void) {
    kb_image *img;
    int i = 0;

    CIRCLEQ_FOREACH(img, &image_head, image) {
        if (i++ < startidx)
            continue;
        img->area = (SDL_Rect){ 0, 0, 640, 480 };
        img = CIRCLEQ_NEXT(img, image);
        if (img)
            img->area = (SDL_Rect){ 640, 0, 640, 480 };
        return;
    }
}

void kb_images_scroll_left(void) {
    // TODO: feedback, dass man bereits ganz links am ende ist
    if (startidx == 0)
        return;
    startidx--;

    fix_areas();
    animation_direction = A_FROM_LEFT_TO_RIGHT;
    animation_step = -640;
}

void kb_images_scroll_right(void) {
    // TODO: verhindern, dass man nach rechts raus-scrolled. passiert aber nichts.
    startidx++;

    fix_areas();
    animation_direction = A_FROM_RIGHT_TO_LEFT;
    animation_step = 640;
}
