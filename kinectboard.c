// vim:ts=4:sw=4:expandtab
/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <libfreenect.h>

#include <pthread.h>

#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

#define elem_type int
#include "quickselect.c"

#include "kinectboard_controls.h"

#define SCREEN_WIDTH (640 * 2)
#define SCREEN_HEIGHT 480
#define SCREEN_DEPTH 32

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned char depthPixelsLookupNearWhite[2048];

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"

// processed depth (with a median filter)
uint8_t *depth_mid, *depth_front;
// raw depth (as received from kinect)
uint8_t *raw_depth_mid, *raw_depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;

struct range {
    int min;
    int max;
    int latest;
};

struct range empty_canvas[640 * 480];
struct range empty_canvas_copy[640 * 480];

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    pthread_mutex_lock(&gl_backbuf_mutex);

    // swap buffers
    assert (rgb_back == rgb);
    rgb_back = rgb_mid;
    freenect_set_video_buffer(dev, rgb_back);
    rgb_mid = (uint8_t*)rgb;

    int i;
    for (i = 0; i < 640 * 480; i++) {
        int r = rgb_mid[3 * i + 0];
        int g = rgb_mid[3 * i + 1];
        int b = rgb_mid[3 * i + 2];
    }

    got_rgb++;
    pthread_cond_signal(&gl_frame_cond);
    pthread_mutex_unlock(&gl_backbuf_mutex);
}

#define row_col_to_px(row, col) ((row) * 640 + (col))

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    int i, col, row;
    uint16_t *depth = (uint16_t*)v_depth;
    pthread_mutex_lock(&gl_backbuf_mutex);
    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;
            raw_depth_mid[3*i+0] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+1] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+2] = depthPixelsLookupNearWhite[depth[i]];
            if (col < 10 || col > (640-10)) {
                depth_mid[3 * i + 0] = depthPixelsLookupNearWhite[depth[i]];
                depth_mid[3 * i + 1] = depthPixelsLookupNearWhite[depth[i]];
                depth_mid[3 * i + 2] = depthPixelsLookupNearWhite[depth[i]];
                continue;
            }

#if 0
            /* 3x3 */
        int neighbors[] = {
            /* links */
            depth[row_col_to_px(row-1, col-1)],
            depth[row_col_to_px(row, col-1)],
            depth[row_col_to_px(row+1, col-1)],
            /* mitte */
            depth[row_col_to_px(row-1, col)],
            depth[row_col_to_px(row, col)],
            depth[row_col_to_px(row+1, col)],
            /* rechts */
            depth[row_col_to_px(row-1, col+1)],
            depth[row_col_to_px(row, col+1)],
            depth[row_col_to_px(row+1, col+1)],
        };
#endif

            /* 5x5 */
        int neighbors[] = {
            depth[row_col_to_px(row-2, col-2)],
            depth[row_col_to_px(row-1, col-2)],
            depth[row_col_to_px(row, col-2)],
            depth[row_col_to_px(row+1, col-2)],
            depth[row_col_to_px(row+2, col-2)],
            /* links */
            depth[row_col_to_px(row-2, col-1)],
            depth[row_col_to_px(row-1, col-1)],
            depth[row_col_to_px(row, col-1)],
            depth[row_col_to_px(row+1, col-1)],
            depth[row_col_to_px(row+2, col-1)],
            /* mitte */
            depth[row_col_to_px(row-2, col)],
            depth[row_col_to_px(row-1, col)],
            depth[row_col_to_px(row, col)],
            depth[row_col_to_px(row+1, col)],
            depth[row_col_to_px(row+2, col)],
            /* rechts */
            depth[row_col_to_px(row-2, col+1)],
            depth[row_col_to_px(row-1, col+1)],
            depth[row_col_to_px(row, col+1)],
            depth[row_col_to_px(row+1, col+1)],
            depth[row_col_to_px(row+2, col+1)],

            depth[row_col_to_px(row-2, col+2)],
            depth[row_col_to_px(row-1, col+2)],
            depth[row_col_to_px(row, col+2)],
            depth[row_col_to_px(row+1, col+2)],
            depth[row_col_to_px(row+2, col+2)],
        };

        int pval = quick_select(neighbors, sizeof(neighbors) / sizeof(int));

        pval = depthPixelsLookupNearWhite[pval];
        depth_mid[3 * i + 0] = pval;
        depth_mid[3 * i + 1] = pval;
        depth_mid[3 * i + 2] = pval;
        }
    }
    got_depth++;
    pthread_cond_signal(&gl_frame_cond);
    pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
    int accelCount = 0;

    freenect_set_tilt_degs(f_dev,freenect_angle);
    freenect_set_led(f_dev,LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_set_video_buffer(f_dev, rgb_back);

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");

    while (!die && freenect_process_events(f_ctx) >= 0) {
        //Throttle the text output
        if (accelCount++ >= 2000)
        {
            accelCount = 0;
            freenect_raw_tilt_state* state;
            freenect_update_tilt_state(f_dev);
            state = freenect_get_tilt_state(f_dev);
            double dx,dy,dz;
            freenect_get_mks_accel(state, &dx, &dy, &dz);
            printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
            fflush(stdout);
        }

        if (requested_format != current_format) {
            freenect_stop_video(f_dev);
            freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
            freenect_start_video(f_dev);
            current_format = requested_format;
        }
    }

    printf("\nshutting down streams...\n");

    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    printf("-- done!\n");
    return NULL;
}

// Callback for button 1
void btn_test_funct(void* placeholder) {
    printf("Button pressed\n");
    fflush(stdout);
}

void kb_poll_events(kb_controls* list) {
    SDL_Event event;
    while ( SDL_PollEvent(&event) ) {
        switch (event.type) {
            case SDL_MOUSEMOTION:
                kb_process_mouse_motion(list, event.motion.x, event.motion.y); 
                break;
            case SDL_MOUSEBUTTONDOWN:
                kb_process_input(list, event.button.button, event.button.x, event.button.y);
            break;
            case SDL_KEYDOWN:
                if(event.key.keysym.sym == SDLK_ESCAPE) {
                    exit(0);	
                }
            break;
	    case SDL_QUIT:
            exit(0);	
        }
    }		
}

int main(int argc, char *argv[]) {

    SDL_Surface *screen;
    Uint8       *p;
    int         x = 10; //x coordinate of our pixel
    int         y = 20; //y coordinate of our pixel


    int i;
    for (i=0; i<2048; i++) {
        const float k1 = 1.1863;
        const float k2 = 2842.5;
        const float k3 = 0.1236;
        //const float depth = k3 * tanf(i/k2 + k1);
        //t_gamma[i] = depth;
        depthPixelsLookupNearWhite[i] = (float) (2048 * 256) / (i - 2048);
    }

    depth_mid = (uint8_t*)malloc(640*480*3);
    depth_front = (uint8_t*)malloc(640*480*3);
    raw_depth_mid = (uint8_t*)malloc(640*480*3);
    raw_depth_front = (uint8_t*)malloc(640*480*3);

    rgb_back = (uint8_t*)malloc(640*480*3);
    rgb_mid = (uint8_t*)malloc(640*480*3);
    rgb_front = (uint8_t*)malloc(640*480*3);

    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("freenect_init() failed\n");
        return 1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    int nr_devices = freenect_num_devices (f_ctx);
    printf ("Number of devices found: %d\n", nr_devices);

    int user_device_number = 0;
    if (argc > 1)
        user_device_number = atoi(argv[1]);

    if (nr_devices < 1) {
        freenect_shutdown(f_ctx);
        return 1;
    }

    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
        printf("Could not open device\n");
        freenect_shutdown(f_ctx);
        return 1;
    }

    int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        freenect_shutdown(f_ctx);
        return 1;
    }

    /* Initialize SDL */
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    /* Initialize the screen / window */
    screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_DEPTH, SDL_SWSURFACE);
    Uint32 rmask, gmask, bmask, amask;

    /* SDL interprets each pixel as a 32-bit number, so our masks must depend
       on the endianness (byte order) of the machine */
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    rmask = 0xff000000;
    gmask = 0x00ff0000;
    bmask = 0x0000ff00;
    amask = 0;
#else
    rmask = 0x000000ff;
    gmask = 0x0000ff00;
    bmask = 0x00ff0000;
    amask = 0;
#endif

    SDL_Surface *kinect_rgb = SDL_CreateRGBSurface(SDL_SWSURFACE, 640, 480, 24, rmask, gmask, bmask, amask);
    SDL_Surface *kinect_rgb_unfiltered = SDL_CreateRGBSurface(SDL_SWSURFACE, 640, 480, 24, rmask, gmask, bmask, amask);

    SDL_Rect targetarea_depth;
    SDL_Rect targetarea_raw_depth;
    targetarea_depth.x = 0;
    targetarea_depth.y = 0;
    targetarea_depth.w = kinect_rgb->w;
    targetarea_depth.h = kinect_rgb->h;

    targetarea_raw_depth.x = 640;
    targetarea_raw_depth.y = 0;
    targetarea_raw_depth.w = kinect_rgb->w;
    targetarea_raw_depth.h = kinect_rgb->h;

    //create Font
    TTF_Font* font = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 40);
    if (!font) {
        printf("font not found\n");
        return 1;
    }
    //Text colors
    SDL_Color foregroundColor = { 255, 255, 255 };
    SDL_Color backgroundColor = { 0, 0, 255 };

//    SDL_Surface* textSurface = TTF_RenderText_Shaded(font, "This is my text.", foregroundColor, backgroundColor);
    SDL_Surface* textSurface = TTF_RenderText_Solid(font, "This is my text.", foregroundColor);
    SDL_Rect textLocation = { 10, 10, 0, 0 };

    kb_controls* list = kb_controls_create();
    
    // Some Buttons
    kb_button* btn = kb_button_create(list,100,25,10,10, &btn_test_funct);
    kb_controls_add_control(list, KB_BUTTON, btn);
    
    kb_button* btn1 = kb_button_create(list,100,25,120,10, &btn_test_funct);
    kb_controls_add_control(list, KB_BUTTON, btn1);
    
    kb_button* btn2 = kb_button_create(list,100,25,230,10, &btn_test_funct);
    kb_controls_add_control(list, KB_BUTTON, btn2);
    
    while (1) {
        pthread_mutex_lock(&gl_backbuf_mutex);

        // When using YUV_RGB mode, RGB frames only arrive at 15Hz, so we shouldn't force them to draw in lock-step.
        // However, this is CPU/GPU intensive when we are receiving frames in lockstep.
        if (current_format == FREENECT_VIDEO_YUV_RGB) {
            while (!got_depth && !got_rgb) {
                pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
            }
        } else {
            while ((!got_depth || !got_rgb) && requested_format != current_format) {
                pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
            }
        }

        if (requested_format != current_format) {
            pthread_mutex_unlock(&gl_backbuf_mutex);
            return;
        }

        uint8_t *tmp;

        if (got_depth) {
            tmp = depth_front;
            depth_front = depth_mid;
            depth_mid = tmp;

            tmp = raw_depth_front;
            raw_depth_front = raw_depth_mid;
            raw_depth_mid = tmp;

            got_depth = 0;
        }
        if (got_rgb) {
            tmp = rgb_front;
            rgb_front = rgb_mid;
            rgb_mid = tmp;
            got_rgb = 0;
        }

        pthread_mutex_unlock(&gl_backbuf_mutex);

        //memcpy(kinect_rgb->pixels, rgb_front, 640 * 480 * 3);
        memcpy(kinect_rgb->pixels, depth_mid, 640 * 480 * 3);
        SDL_BlitSurface(kinect_rgb, NULL, screen, &targetarea_depth);

        memcpy(kinect_rgb_unfiltered->pixels, raw_depth_mid, 640 * 480 * 3);
        SDL_BlitSurface(kinect_rgb_unfiltered, NULL, screen, &targetarea_raw_depth);

        kb_poll_events(list);
        
        kb_controls_render(list, screen);

        SDL_BlitSurface(textSurface, NULL, screen, &textLocation);
        
    
        /* update the screen (aka double buffering) */
        SDL_Flip(screen);
    }
    
    kb_button_destroy(list, btn);
    kb_button_destroy(list, btn1);
    kb_button_destroy(list, btn2);
    kb_controls_destroy(list);
}
