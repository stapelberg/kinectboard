// vim:ts=4:sw=4:expandtab

#include "kinectboard_images.h"

// two kinect images (full resolution) next to each other
#define SCREEN_WIDTH (640 * 2)
// kinect image height (480) + space for controls
#define SCREEN_HEIGHT 480 + 300
#define SCREEN_DEPTH 32

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
static int queue_size = 0;

/* ******************************************************************* */
/* KB (RGB) Image */

void kb_image_create(const char *label, GLuint bufferID, GLuint textureID) {
    kb_image *new_img = calloc(sizeof(kb_image), 1);
    new_img->label = label;
    new_img->bufferID = bufferID;
    new_img->textureID = textureID;

#if 0
    TTF_Font* label_font = TTF_OpenFont("Vera.ttf", 28);
	SDL_Color color = (SDL_Color){255,255,255};
	new_img->labelText = TTF_RenderText_Solid(label_font, new_img->label, color);
	new_img->labelTextLocation = (SDL_Rect){ 120, 5,10,10 };
	SDL_BlitSurface(new_img->labelText, NULL, new_img->surface, &new_img->labelTextLocation);
#endif

    /* Wir unterscheiden hier nur zwischen 0 und 1 Elementen. Es werden nur die
     * ersten beiden Bilder gerendert standardmäßig (kein Platz mehr auf dem
     * Bildschirm). Wenn der Nutzer scrolled, werden die areas ohnehin neu
     * berechnet. */
    if (!CIRCLEQ_EMPTY(&image_head))
        new_img->area = (SDL_Rect){ 640, 0, 640, 480 };
    else new_img->area = (SDL_Rect){ 0, 0, 640, 480 };

    CIRCLEQ_INSERT_TAIL(&image_head, new_img, image);
    queue_size++;
}

void kb_images_render() {
    int i = 0, cnt = 0;
    SDL_Rect area;
    kb_image *img;
    CIRCLEQ_FOREACH(img, &image_head, image) {
        if (animation_step > 0) {
            if (i++ < startidx-1)
                continue;
        } else {
            if (i++ < startidx)
                continue;
        }
 
        if (animation_step > 0 && cnt == 0) {
            area = img->area;
            area.x -= (640-animation_step); 
        } else if (animation_step < 0 && cnt == 2) {
            area = img->area;
            area.x += (640-(-1*animation_step));
        } else {
            area = img->area;
            area.x += animation_step;
        }
        
        if (animation_step > 0) {
            animation_step -= ANIMATION_ONE_STEP;
            if (animation_step < 0)
                animation_step = 0;
        }

        if (animation_step < 0) {
            animation_step += ANIMATION_ONE_STEP;
            if (animation_step >= 0)
                animation_step = 0;
        }

        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, img->bufferID);
        glBindTexture(GL_TEXTURE_2D, img->textureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

        glBegin(GL_QUADS);
            glTexCoord2f(0.0f, 1.0f);   glVertex2f(area.x, 300.0f);
            glTexCoord2f(0.0f, 0.0f);   glVertex2f(area.x, SCREEN_HEIGHT * 1.0f);
            glTexCoord2f(1.0f, 0.0f);   glVertex2f(area.x + 640, SCREEN_HEIGHT * 1.0f);
            glTexCoord2f(1.0f, 1.0f);   glVertex2f(area.x + 640, 300.0f);
        glEnd();

        if (animation_step != 0) {
            if (++cnt == 3) break;
        } else {
            if (++cnt == 2) break;
        }        
    }
}

static void fix_areas(void) {
    kb_image *img;
    int i = 0;

    CIRCLEQ_FOREACH(img, &image_head, image) {
        if (i++ < startidx)
            continue;
    
        img->area = (SDL_Rect){ 0, 0, 640, 480 };
        kb_ui_call_javascript ("setLeftImageLabel", img->label);
        
        img = CIRCLEQ_NEXT(img, image);
        if (img) {
            kb_ui_call_javascript("setRightImageLabel", img->label);
            img->area = (SDL_Rect){ 640, 0, 640, 480 };
        }
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
    if(startidx == queue_size-2)
        return;
    startidx++;

    fix_areas();
    animation_direction = A_FROM_RIGHT_TO_LEFT;
    animation_step = 640;
}

void kb_images_current_buffers(GLuint *left, GLuint *right) {
    kb_image *img;
    int cnt = 0, i = 0;
    CIRCLEQ_FOREACH(img, &image_head, image) {
        if (animation_step > 0) {
            if (i++ < startidx-1)
                continue;
        } else {
            if (i++ < startidx)
                continue;
        }
        if (cnt++ == 0)
            *left = img->bufferID;
        else {
            *right = img->bufferID;
            break;
        }
    }
}
