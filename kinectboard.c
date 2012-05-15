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
#include <stdint.h>
#include <math.h>

#include <opencv2/core/core_c.h>

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
#include "kinectboard_images.h"
#include <sys/time.h>

// two kinect images (full resolution) next to each other
#define SCREEN_WIDTH (640 * 2)
// kinect image height (480) + space for controls
#define SCREEN_HEIGHT 480 + 300
#define SCREEN_DEPTH 32

/* Tauscht den Front- und Mid-Buffer aus */
#define swap_buffer(name) do { \
    uint8_t *tmp = name ## _front; \
    name ## _front = name ## _mid; \
    name ## _mid = tmp; \
} while (0)

#define pushrgb(name, idx, r, g, b) do { \
    name ## _mid[3 * idx + 0] = r; \
    name ## _mid[3 * idx + 1] = g; \
    name ## _mid[3 * idx + 2] = b; \
} while (0)

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
uint8_t *depth_median_filtered_rgb;
uint8_t *depth_median_filtered_masked_rgb;
// processed and masked depth as an RGB image
uint8_t *masked_depth_mid, *masked_depth_front;
// processed and masked depth as an RGB image (masked area retains the depth info)
uint8_t *masked_depth_detail_mid, *masked_depth_detail_front;
// Glow area
uint8_t *glow_mid, *glow_front;
// raw depth (as received from kinect)
uint8_t *raw_depth_mid, *raw_depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;
uint8_t *rgb_masked_mid, *rgb_masked_front;

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
static bool EICH_MODUS = false;

extern int ANIMATION_ONE_STEP;
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

int DEPTH_MASK_THRESHOLD = 2;

/* asta-raum */
int GLOW_START = 126;
int GLOW_END = 230;

//int GLOW_START = 0;
//int GLOW_END = 1024;

kb_label *median_slider_value;
kb_label *depth_slider_value;
kb_label *distance_slider_value;
kb_label *depth_difference_value;
kb_label *glow_start_value;
kb_label *glow_end_value;

SDL_Surface *chosen_color_surface;

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
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
                pushrgb(rgb, i, 0, 0, 0);
            }
        }
    }

    int col, row, i;
    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;
            int depth = masked_depth_detail_mid[3 * i + 0];

            /* Koordinaten umrechnen in Koordinaten auf dem RGB-Bild */
            double x = ((col - 3.3930780975300314e+02) * depth) / 5.9421434211923247e+02;
            double y = ((row - 2.4273913761751615e+02) * depth) / 5.9104053696870778e+02;
            double z = depth;

            /* Rotation und Translation durchführen */
            CvMat *rotationsmatrix = cvCreateMat(3, 3, CV_32FC1);
            CV_MAT_ELEM(*rotationsmatrix, float, 0, 0) = 9.9984628826577793e-01;
            CV_MAT_ELEM(*rotationsmatrix, float, 0, 1) = 1.2635359098409581e-03;
            CV_MAT_ELEM(*rotationsmatrix, float, 0, 2) = -1.7487233004436643e-02;

            CV_MAT_ELEM(*rotationsmatrix, float, 1, 0) = -1.4779096108364480e-03;
            CV_MAT_ELEM(*rotationsmatrix, float, 1, 1) = 9.9992385683542895e-01;
            CV_MAT_ELEM(*rotationsmatrix, float, 1, 2) = -1.2251380107679535e-02;

            CV_MAT_ELEM(*rotationsmatrix, float, 2, 0) = 1.7470421412464927e-02;
            CV_MAT_ELEM(*rotationsmatrix, float, 2, 1) = 1.2275341476520762e-02;
            CV_MAT_ELEM(*rotationsmatrix, float, 2, 2) = 9.9977202419716948e-01;

            CvMat *translationsmatrix = cvCreateMat(3, 1, CV_32FC1);
            CV_MAT_ELEM(*translationsmatrix, float, 0, 0) = 1.9985242312092553e-02;
            CV_MAT_ELEM(*translationsmatrix, float, 1, 0) = -7.4423738761617583e-04;
            CV_MAT_ELEM(*translationsmatrix, float, 2, 0) = -1.0916736334336222e-02;

            CvMat *result = cvCreateMat(3, 1, CV_32FC1);
            CvMat *r2 = cvCreateMat(3, 1, CV_32FC1);

            CvMat *P3D = cvCreateMat(3, 1, CV_32FC1);
            CV_MAT_ELEM(*P3D, float, 0, 0) = x;
            CV_MAT_ELEM(*P3D, float, 1, 0) = y;
            CV_MAT_ELEM(*P3D, float, 2, 0) = z;

            CvMat *P3DT = cvCreateMat(1, 3, CV_32FC1);
            cvMatMul(rotationsmatrix, P3D, result);
            cvAdd(result, translationsmatrix, r2, NULL);
            cvTranspose(r2, P3DT);

            double xb = (CV_MAT_ELEM(*P3DT, float, 0, 0) * 5.2921508098293293e+02 / CV_MAT_ELEM(*P3DT, float, 0, 2)) + 3.2894272028759258e+02;
            double yb = (CV_MAT_ELEM(*P3DT, float, 0, 1) * 5.2556393630057437e+02 / CV_MAT_ELEM(*P3DT, float, 0, 2)) + 2.6748068171871557e+02;

            int di = ((int)yb * 640) + (int)xb;
            if (glow_mid[3 * i + 1] != 255 && glow_mid[3 * i + 1] != 0) {
                pushrgb(rgb_masked, i, rgb_mid[3 * di + 0], rgb_mid[3 * di + 1], rgb_mid[3 * di + 2]);
            } else {
                pushrgb(rgb_masked, i, 255, 0, 0);
            }

            cvReleaseMat(&r2);
            cvReleaseMat(&P3D);
            cvReleaseMat(&P3DT);
            cvReleaseMat(&result);
            cvReleaseMat(&translationsmatrix);
            cvReleaseMat(&rotationsmatrix);
        }
    }

#if 0
    /* RGB-Bild maskieren */
    int col, row, i;
    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;

            if (glow_mid[3 * i + 1] != 255 && glow_mid[3 * i + 1] != 0) {
                pushrgb(rgb_masked, i, rgb_mid[3 * i + 0], rgb_mid[3 * i + 1], rgb_mid[3 * i + 2]);
            } else {
                pushrgb(rgb_masked, i, 255, 0, 0);
            }

        }
    }
#endif

    /* Buffer austauschen (double-buffering) */
    swap_buffer(rgb);
    swap_buffer(rgb_masked);
}

#define row_col_to_px(row, col) ((row) * 640 + (col))

static time_t last_time;
int fps = 0;
int frames = 0;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    time_t current_time = time(NULL);
    if (current_time != last_time) {
        fps = frames;
        frames = 0;
        last_time = current_time;
    } else {
        frames++;
    }

    int i, col, row;
    uint16_t *depth = (uint16_t*)v_depth;

    pthread_mutex_lock(&median_filter_mutex);
    int nneighbors[MEDIAN_FILTER_SIZE * MEDIAN_FILTER_SIZE];

    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;
            raw_depth_mid[3*i+0] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+1] = depthPixelsLookupNearWhite[depth[i]];
            raw_depth_mid[3*i+2] = depthPixelsLookupNearWhite[depth[i]];
            if (col < MEDIAN_FILTER_SIZE || col > (640 - MEDIAN_FILTER_SIZE)) {
                pushrgb(depth, i, 255, 0, 0);
                continue;
            }
            if (row < MEDIAN_FILTER_SIZE || row > (480-MEDIAN_FILTER_SIZE)) {
                //depth_mid[3 * i + 0] = depthPixelsLookupNearWhite[depth[i]];
                //depth_mid[3 * i + 1] = depthPixelsLookupNearWhite[depth[i]];
                //depth_mid[3 * i + 2] = depthPixelsLookupNearWhite[depth[i]];
                pushrgb(depth, i, 0, 0, 255);
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
            depth_median_filtered_rgb[i] = pvaln;
            pushrgb(depth, i, pvaln, pvaln, pvaln);

            if (EICH_MODUS) {
                if (pvaln < empty_canvas[i].min)
                    empty_canvas[i].min = pvaln;
                if (pvaln > empty_canvas[i].max)
                    empty_canvas[i].max = pvaln;
            }

            int difference = 0;
            if (pvaln < empty_canvas[i].min) {
                difference = (empty_canvas[i].min - pvaln);
                //printf("difference: %d\n", difference);
            }
            if (pvaln > empty_canvas[i].max) {
                //printf("difference: %d\n", (empty_canvas[i].max - pvaln));
                if ((pvaln - empty_canvas[i].max) > difference)
                    difference = (pvaln - empty_canvas[i].max);
            }
            if (difference > DEPTH_MASK_THRESHOLD) {
                pushrgb(masked_depth, i, 255, 0, 0);
                depth_median_filtered_masked_rgb[i] = pvaln;
            } else {
                pushrgb(masked_depth, i, pvaln, pvaln, pvaln);
                depth_median_filtered_masked_rgb[i] = 255;
            }

            if (difference > DEPTH_MASK_THRESHOLD) {
                pushrgb(masked_depth_detail, i, pvaln, pvaln, pvaln);

                if (depth_median_filtered[i] >= GLOW_START &&
                    depth_median_filtered[i] <= GLOW_END) {
                    pushrgb(glow, i, pvaln, pvaln, pvaln);

                    /* Glow-Effekt anwenden (einen Block an Pixeln außenrum
                     * auch markieren) */
                    int gcol, grow, gi;
                    for (grow = (row-20); grow < (row+20); grow++) {
                        for (gcol = (col-20); gcol < (col+20); gcol++) {
                            gi = grow * 640 + gcol;
                            pushrgb(glow, gi, glow_mid[3 * i + 0], glow_mid[3 * i + 1], glow_mid[3 * i + 2]);
                        }
                    }
                } else {
                    pushrgb(glow, i, 0, 0, 0);
                }
            } else {
                pushrgb(masked_depth_detail, i, 0, 255, 0);
                pushrgb(glow, i, 0, 255, 0);
            }

        }
    }

    pthread_mutex_unlock(&median_filter_mutex);

    swap_buffer(depth);
    swap_buffer(raw_depth);
    swap_buffer(masked_depth);
    swap_buffer(masked_depth_detail);
    swap_buffer(glow);
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
void toggle_eichen(uint8_t state) {
    printf("Button pressed, state = %d\n", state);
    fflush(stdout);
    EICH_MODUS = !EICH_MODUS;
    if (EICH_MODUS) {
        int i;
        for (i = 0; i < 640 * 480; i++) {
            empty_canvas[i].min = 99999;
            empty_canvas[i].max = -99999;
        }
    }
}

// Callback for slider
void slider_test_funct(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    pthread_mutex_lock(&median_filter_mutex);
    MEDIAN_FILTER_SIZE = slider_val * 100.f;
    if ((MEDIAN_FILTER_SIZE % 2) == 0)
        MEDIAN_FILTER_SIZE += 1;
    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%d px", MEDIAN_FILTER_SIZE);
    kb_label_changeText(median_slider_value, buffer);
    pthread_mutex_unlock(&median_filter_mutex);
    fflush(stdout);
}

void modify_distance(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    FILTER_DISTANCE = slider_val;

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%f px", FILTER_DISTANCE);
    kb_label_changeText(distance_slider_value, buffer);
}

void modify_depth_mask_multiplier(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    DEPTH_MASK_MULTIPLIER = slider_val;

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%f px", DEPTH_MASK_MULTIPLIER);
    kb_label_changeText(depth_slider_value, buffer);
}

void modify_depth_difference_threshold(float slider_val) {
    printf("Slider at %f percent.\n", slider_val*100.f);
    DEPTH_MASK_THRESHOLD = slider_val * 100.f;

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%d px", DEPTH_MASK_THRESHOLD);
    kb_label_changeText(depth_difference_value, buffer);
}

void modify_glow_start(float slider_val) {
    printf("glow start Slider at %f percent.\n", slider_val*100.f);
    GLOW_START = slider_val * 1024.f;

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%d", GLOW_START);
    kb_label_changeText(glow_start_value, buffer);
}

void modify_glow_end(float slider_val) {
    printf("glow end Slider at %f percent.\n", slider_val*100.f);
    GLOW_END = slider_val * 1024.f;

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), "%d", GLOW_END);
    kb_label_changeText(glow_end_value, buffer);
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

                        SDL_Color kb_background_color = { r, g, b };
                        printf("rendering rect in %d, %d, %d\n",
                                kb_background_color.r, kb_background_color.g, kb_background_color.b);
                        SDL_Rect chosen_color_rect = { 0, 0, 200, 20 };
                        free(chosen_color_surface);
                        chosen_color_surface = kb_surface_fill_color(&chosen_color_rect, &kb_background_color);

                        printf("eich_werte = min %d, max %d, current %d\n",
                                empty_canvas[pixelidx].min,
                                empty_canvas[pixelidx].max,
                                depth_median_filtered_rgb[pixelidx]
                                );
                    }
                }
            break;
            case SDL_KEYDOWN:
                if(event.key.keysym.sym == SDLK_ESCAPE) {
                    exit(0);	
                } else if (event.key.keysym.sym == SDLK_LEFT) {
                    kb_images_scroll_left();
                } else if (event.key.keysym.sym == SDLK_RIGHT) {
                    kb_images_scroll_right();
                } else {
                    kb_process_keyboard(list, event.key.keysym.sym);
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

    int i;
    for (i=0; i<2048; i++) {
        //const float k1 = 1.1863;
        //const float k2 = 2842.5;
        //const float k3 = 0.1236;
        //const float depth = k3 * tanf(i/k2 + k1);
        //t_gamma[i] = depth;
        depthPixelsLookupNearWhite[i] = (float) (2048 * 256) / (i - 2048);
    }

    depth_mid = (uint8_t*)malloc(640*480*3);
    depth_front = (uint8_t*)malloc(640*480*3);
    depth_median_filtered = (uint8_t*)malloc(640*480);
    depth_median_filtered_rgb = (uint8_t*)malloc(640*480);
    depth_median_filtered_masked_rgb = (uint8_t*)malloc(640*480);
    masked_depth_mid = (uint8_t*)malloc(640*480*3);
    masked_depth_front = (uint8_t*)malloc(640*480*3);
    masked_depth_detail_mid = (uint8_t*)malloc(640*480*3);
    masked_depth_detail_front = (uint8_t*)malloc(640*480*3);
    glow_mid = (uint8_t*)malloc(640*480*3);
    glow_front = (uint8_t*)malloc(640*480*3);
    raw_depth_mid = (uint8_t*)malloc(640*480*3);
    raw_depth_front = (uint8_t*)malloc(640*480*3);

    rgb_back = (uint8_t*)malloc(640*480*3);
    rgb_mid = (uint8_t*)malloc(640*480*3);
    rgb_front = (uint8_t*)malloc(640*480*3);
    rgb_masked_mid = (uint8_t*)malloc(640*480*3);
    rgb_masked_front = (uint8_t*)malloc(640*480*3);

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
    SDL_WM_SetCaption("kinectboard", "");

    //create Font
    TTF_Font *slider_label_font = TTF_OpenFont("/usr/share/fonts/truetype/ttf-bitstream-vera/Vera.ttf", 14);
    if (!slider_label_font) {
        printf("font not found\n");
        return 1;
    }

    kb_controls* list = kb_controls_create();
    
    // Some Buttons
    kb_button_create(list,100,25,10,10, &toggle_eichen, SDLK_e, "Eichen (e)", slider_label_font);
    
    // Median pixel values
    kb_label_create(list, 5, 500, "Median pixels:", slider_label_font);
    median_slider_value = kb_label_create(list, 500, 500, "5 px", slider_label_font);
    kb_slider_create(list, 300, 25, 175, 500, &slider_test_funct, 5.f);

    // Distance slider
    kb_label_create(list, 5, 540, "Distance threshold:", slider_label_font);
    distance_slider_value = kb_label_create(list, 500, 540, "0.2", slider_label_font);
    kb_slider_create(list, 300,25,175,540, &modify_distance, 20);

    // Depth multiplier
    kb_label_create(list, 5, 580, "Depth multiplier:", slider_label_font);
    depth_slider_value = kb_label_create(list, 500, 580, "0", slider_label_font);
    kb_slider_create(list, 300,25,175,580, &modify_depth_mask_multiplier, .2f);

    // depth difference threshold
    kb_label_create(list, 5, 620, "Depth difference threshold:", slider_label_font);
    depth_difference_value = kb_label_create(list, 500, 620, "0", slider_label_font);
    modify_depth_difference_threshold(DEPTH_MASK_THRESHOLD / 100.0);
    kb_slider_create(list, 300,25,175,620, &modify_depth_difference_threshold, DEPTH_MASK_THRESHOLD);

    // glow area start
    kb_label_create(list, 5, 660, "Glow area start", slider_label_font);
    glow_start_value = kb_label_create(list, 500, 660, "0", slider_label_font);
    modify_glow_start((GLOW_START / 1024.0));
    kb_slider_create(list, 300,25,175,660, &modify_glow_start, (GLOW_START / 1024.0) * 100);

    // glow area end
    kb_label_create(list, 5, 700, "Glow area end", slider_label_font);
    glow_end_value = kb_label_create(list, 500, 700, "1024", slider_label_font);
    modify_glow_end((GLOW_END / 1024.0));
    kb_slider_create(list, 300,25,175,700, &modify_glow_end, (GLOW_END / 1024.0) * 100);

    kb_image_create("Raw depth image", &raw_depth_front);
    kb_image_create("Median-filtered depth image", &depth_front);
    kb_image_create("Masked depth image", &masked_depth_front);
    kb_image_create("Masked depth detail image", &masked_depth_detail_front);
    kb_image_create("Glowing depth", &glow_front);
    kb_image_create("Raw kinect RGB image", &rgb_front);
    kb_image_create("Masked kinect RGB image", &rgb_masked_front);

    SDL_Rect kb_screen_rect = {0,0,SCREEN_WIDTH, SCREEN_HEIGHT};
    SDL_Color kb_background_color = {0,0,0};
    // Reference color
    kb_label_create(list, 640, 500, "Reference color:", slider_label_font);
    /* Rect to fill *within* the rectangle, needs to start at 0 x 0 */
    SDL_Rect origin_rect = { 0, 0, 200, 20 };
    /* Where the rectangle ends up on the screen */
    SDL_Rect chosen_color_rect = { 800, 500, 200, 20};
    chosen_color_surface = kb_surface_fill_color(&origin_rect, &kb_background_color);
    SDL_Surface* kb_background = kb_surface_fill_color(&kb_screen_rect, &kb_background_color);
    /*FPS Anzeige*/
    char fpsBuffer[2048];
    kb_label* fpsLabel = kb_label_create(list, 1010, 500, "? FPS", slider_label_font);

    while (1) {
        /* Schwarzer Hintergrund */
        SDL_BlitSurface(kb_background, NULL, screen, &kb_screen_rect);

        kb_poll_events(list);
        kb_images_render(screen);
        kb_controls_render(list, screen);

        /* Für manche Dinge in der GUI haben wir keine eigenen Controls, z.B.
         * für den gewählten Farbwert. */
        SDL_BlitSurface(chosen_color_surface, NULL, screen, &chosen_color_rect);

		// Refresh fps
		snprintf(fpsBuffer, sizeof(fpsBuffer), "%d FPS", fps);
		kb_label_changeText(fpsLabel, fpsBuffer);

        /* update the screen (aka double buffering) */
        SDL_Flip(screen);
    }
}
