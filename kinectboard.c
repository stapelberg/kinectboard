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

// processed depth as an RGB image (with a median filter)
uint8_t *depth_mid, *depth_front;
// processed depth as raw values (with a median filter)
uint8_t *depth_median_filtered;
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

int animation_step = 0;
int ANIMATION_ONE_STEP = 30;
// Idealerweise auf 13, sobald wir CUDA haben. Bis dahin auf 9.
int MEDIAN_FILTER_SIZE = 5;
pthread_mutex_t median_filter_mutex = PTHREAD_MUTEX_INITIALIZER;

// Die Referenz-Farbe (vom Nutzer ausgewählt). Wird normiert gespeichert:
// Jeder Farbanteil wird durch √(r² + g² + b²) geteilt.
double reference_r = -1;
double reference_g = -1;
double reference_b = -1;

double FILTER_DISTANCE = 0.2f;

double DEPTH_MASK_MULTIPLIER = 0.0f;

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    pthread_mutex_lock(&gl_backbuf_mutex);

    // swap buffers
    assert (rgb_back == rgb);
    rgb_back = rgb_mid;
    freenect_set_video_buffer(dev, rgb_back);
    rgb_mid = (uint8_t*)rgb;

    /* Wenn es eine Referenzefarbe gibt, filtern wir das Farbbild. */
    if (reference_r != -1) {
        int i;
        for (i = 0; i < 640 * 480; i++) {
            double r = rgb_mid[3 * i + 0];
            double g = rgb_mid[3 * i + 1];
            double b = rgb_mid[3 * i + 2];

            double nom = sqrt((r * r) + (g * g) + (b * b));
            r /= nom;
            g /= nom;
            b /= nom;

            /* depth_mid ist das Tiefenbild. */
            double distance = sqrt(pow((reference_r - r), 2) + pow((reference_g - g), 2) + pow((reference_b - b), 2));

            //printf("median_filtered = %d\n", depth_median_filtered[i]);
            distance += (depth_median_filtered[i] / 255.0) * DEPTH_MASK_MULTIPLIER;

            if (distance > FILTER_DISTANCE) {
                rgb_mid[3 * i + 0] = 0;
                rgb_mid[3 * i + 1] = 0;
                rgb_mid[3 * i + 2] = 0;
            }
        }
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

    pthread_mutex_lock(&median_filter_mutex);
    int nneighbors[MEDIAN_FILTER_SIZE * MEDIAN_FILTER_SIZE];

    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;
            raw_depth_mid[3*i+0] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+1] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+2] = depthPixelsLookupNearWhite[depth[i]];
            if (col < MEDIAN_FILTER_SIZE || col > (640 - MEDIAN_FILTER_SIZE)) {
                depth_mid[3 * i + 0] = 255;
                depth_mid[3 * i + 1] = 0;
                depth_mid[3 * i + 2] = 0;
                continue;
            }
            if (row < MEDIAN_FILTER_SIZE || row > (480-MEDIAN_FILTER_SIZE)) {
                //depth_mid[3 * i + 0] = depthPixelsLookupNearWhite[depth[i]];
                //depth_mid[3 * i + 1] = depthPixelsLookupNearWhite[depth[i]];
                //depth_mid[3 * i + 2] = depthPixelsLookupNearWhite[depth[i]];
                depth_mid[3 * i + 0] = 0;
                depth_mid[3 * i + 1] = 0;
                depth_mid[3 * i + 2] = 255;
                continue;
            }

            int ic, ir;
            int ni = 0;
            for (ic = (col - (MEDIAN_FILTER_SIZE / 2));
                 ic <= (col + (MEDIAN_FILTER_SIZE / 2));
                 ic++) {
                for (ir = (row - (MEDIAN_FILTER_SIZE / 2));
                     ir <= (row + (MEDIAN_FILTER_SIZE / 2));
                     ir++) {
                    nneighbors[ni++] = depth[row_col_to_px(ir, ic)];
                }

            }

            int pvaln = quick_select(nneighbors, ni);
            depth_median_filtered[i] = pvaln;
            pvaln = depthPixelsLookupNearWhite[pvaln];
            depth_mid[3 * i + 0] = pvaln;
            depth_mid[3 * i + 1] = pvaln;
            depth_mid[3 * i + 2] = pvaln;
        }
    }

    pthread_mutex_unlock(&median_filter_mutex);

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

// Callback for slider
void slider_test_funct(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    pthread_mutex_lock(&median_filter_mutex);
    MEDIAN_FILTER_SIZE = slider_val * 100.f;
    if ((MEDIAN_FILTER_SIZE % 2) == 0)
        MEDIAN_FILTER_SIZE += 1;
    pthread_mutex_unlock(&median_filter_mutex);
    fflush(stdout);
}

void modify_distance(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    FILTER_DISTANCE = slider_val;
}

void modify_depth_mask_multiplier(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    DEPTH_MASK_MULTIPLIER = slider_val;
}


void kb_poll_events(kb_controls* list) {
    SDL_Event event;
    while ( SDL_PollEvent(&event) ) {
        switch (event.type) {
            case SDL_MOUSEMOTION:
                kb_process_mouse_motion(list, event.button.button, event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel); 
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (!kb_process_input(list, event.button.button, event.button.x, event.button.y)) {
                    /* Der Klick ging nicht auf irgendein Control. Wir prüfen,
                     * ob er im Bereich des Farbbilds ist und lesen dann die
                     * entsprechende Farbe aus. */
                    // TODO: eher einen callback nutzen
                    if (event.button.x > 640) {
                        int pixelidx = event.button.y * 640 + (event.button.x - 640);
                        printf("clicked on x = %d, y = %d, this is pixelidx %d\n",
                                event.button.x, event.button.y, pixelidx);
                        double r = rgb_mid[3 * pixelidx + 0];
                        double g = rgb_mid[3 * pixelidx + 1];
                        double b = rgb_mid[3 * pixelidx + 2];
                        double nominator = sqrt((r * r) + (g * g) + (b * b));
                        printf("nominator = %f\n", nominator);
                        reference_r = r / nominator;
                        reference_g = g / nominator;
                        reference_b = b / nominator;
                        printf("r = %f, g = %f, b = %f\n", r, g, b);
                        printf("reference: r = %f, g = %f, b = %f\n",
                                reference_r, reference_g, reference_b);
                    }
                }
            break;
            case SDL_KEYDOWN:
                if(event.key.keysym.sym == SDLK_ESCAPE) {
                    exit(0);	
                }
                if (event.key.keysym.sym == SDLK_RIGHT) {
                    animation_step = 640;
                }
                break;
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
    depth_median_filtered = (uint8_t*)malloc(640*480);
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
    SDL_Rect textLocation = { 10, 10, 0, 0 };
    SDL_Rect textLocation2 = { 10, 40, 0, 0 };
    SDL_Rect textLocation3 = { 10, 60, 0, 0 };

    kb_controls* list = kb_controls_create();
    
    // Some Buttons
    kb_button* btn = kb_button_create(list,100,25,10,10, &btn_test_funct);
    kb_button* btn1 = kb_button_create(list,100,25,120,10, &btn_test_funct);
    kb_button* btn2 = kb_button_create(list,100,25,230,10, &btn_test_funct);    
    
    // A slider
    kb_slider* slider = kb_slider_create(list, 300,25,10,400,&slider_test_funct, 5.f);
    kb_slider* distance_slider = kb_slider_create(list, 300,25,10,200, &modify_distance, .2f);
    kb_slider* depth_multiplier = kb_slider_create(list, 300,25,10,300, &modify_depth_mask_multiplier, .2f);
    
    char mediantextbuffer[256];

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
        memcpy(kinect_rgb->pixels, depth_front, 640 * 480 * 3);
        SDL_BlitSurface(kinect_rgb, NULL, screen, &targetarea_depth);

    targetarea_raw_depth.x = 640 + animation_step;
    targetarea_raw_depth.y = 0;
    targetarea_raw_depth.w = kinect_rgb->w;
    targetarea_raw_depth.h = kinect_rgb->h;

    if (animation_step)
        animation_step -= ANIMATION_ONE_STEP;
    if (animation_step < 0)
        animation_step = 0;


        memcpy(kinect_rgb_unfiltered->pixels, rgb_front, 640 * 480 * 3);
        //memcpy(kinect_rgb_unfiltered->pixels, raw_depth_mid, 640 * 480 * 3);
        SDL_BlitSurface(kinect_rgb_unfiltered, NULL, screen, &targetarea_raw_depth);

        kb_poll_events(list);
        
        kb_controls_render(list, screen);

        snprintf(mediantextbuffer, sizeof(mediantextbuffer), "Median: %d pixel", MEDIAN_FILTER_SIZE);
        SDL_Surface* textSurface = TTF_RenderText_Solid(font, mediantextbuffer, foregroundColor);
        SDL_BlitSurface(textSurface, NULL, screen, &textLocation);
        
        snprintf(mediantextbuffer, sizeof(mediantextbuffer), "Distance: %f", FILTER_DISTANCE);
        SDL_Surface* textSurface2 = TTF_RenderText_Solid(font, mediantextbuffer, foregroundColor);
        SDL_BlitSurface(textSurface2, NULL, screen, &textLocation2);

        snprintf(mediantextbuffer, sizeof(mediantextbuffer), "Depth mask multiplier: %f", DEPTH_MASK_MULTIPLIER);
        SDL_Surface* textSurface3 = TTF_RenderText_Solid(font, mediantextbuffer, foregroundColor);
        SDL_BlitSurface(textSurface3, NULL, screen, &textLocation3);


        /* update the screen (aka double buffering) */
        SDL_Flip(screen);
    }
    
    kb_button_destroy(list, btn);
    kb_button_destroy(list, btn1);
    kb_button_destroy(list, btn2);
    kb_slider_destroy(list, slider);
    
    kb_controls_destroy(list);
}
