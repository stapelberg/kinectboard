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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pthread.h>

#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

#include <sys/time.h>

#include "kinect.h"
#include "median.h"
#include "glow.h"

// Cuda
#include <cuda.h>
#include <cutil_inline.h>
#include <cuda_gl_interop.h>
#include <drvapi_error_string.h>

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

GLuint medianBufferID;
GLuint maskedMedianBufferID;
GLuint glowBufferID;
GLuint medianTextureID;
GLuint maskedMedianTextureID;
GLuint glowTextureID;

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned char depthPixelsLookupNearWhite[2048];

uint16_t *depth_buffer;

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

#if 0
    static CvMat *rotationsmatrix = NULL;
    static CvMat *translationsmatrix = NULL;
    static CvMat *result = NULL;
    static CvMat *P3D = NULL;
    if (!rotationsmatrix) {
        rotationsmatrix = cvCreateMat(3, 3, CV_32FC1);

        CV_MAT_ELEM(*rotationsmatrix, float, 0, 0) = 9.9984628826577793e-01;
        CV_MAT_ELEM(*rotationsmatrix, float, 0, 1) = 1.2635359098409581e-03;
        CV_MAT_ELEM(*rotationsmatrix, float, 0, 2) = -1.7487233004436643e-02;

        CV_MAT_ELEM(*rotationsmatrix, float, 1, 0) = -1.4779096108364480e-03;
        CV_MAT_ELEM(*rotationsmatrix, float, 1, 1) = 9.9992385683542895e-01;
        CV_MAT_ELEM(*rotationsmatrix, float, 1, 2) = -1.2251380107679535e-02;

        CV_MAT_ELEM(*rotationsmatrix, float, 2, 0) = 1.7470421412464927e-02;
        CV_MAT_ELEM(*rotationsmatrix, float, 2, 1) = 1.2275341476520762e-02;
        CV_MAT_ELEM(*rotationsmatrix, float, 2, 2) = 9.9977202419716948e-01;
    }

    if (!translationsmatrix) {
        translationsmatrix = cvCreateMat(3, 1, CV_32FC1);

        CV_MAT_ELEM(*translationsmatrix, float, 0, 0) = 1.9985242312092553e-02;
        CV_MAT_ELEM(*translationsmatrix, float, 1, 0) = -7.4423738761617583e-04;
        CV_MAT_ELEM(*translationsmatrix, float, 2, 0) = -1.0916736334336222e-02;
    }

    if (!P3D)
        P3D = cvCreateMat(3, 1, CV_32FC1);

    if (!result)
        result = cvCreateMat(3, 1, CV_32FC1);

    int col, row, i;
    for (i = 0; i < (640*480); i++) {
        pushrgb(rgb_masked, i, 250, 250, 0);
    }
    for (row = 0; row < (480); row++) {
        for (col = 0; col < 640; col++) {
            i = row * 640 + col;
            int depth = glow_mid[3 * i + 0];
            if (depth == 0)
                continue;

            /* Koordinaten umrechnen in Koordinaten auf dem RGB-Bild */
            // X
            CV_MAT_ELEM(*P3D, float, 0, 0) = ((col - 3.3930780975300314e+02) * depth) / 5.9421434211923247e+02;
            // Y
            CV_MAT_ELEM(*P3D, float, 1, 0) = ((row - 2.4273913761751615e+02) * depth) / 5.9104053696870778e+02;
            // depth
            CV_MAT_ELEM(*P3D, float, 2, 0) = depth;

            cvMatMul(rotationsmatrix, P3D, result);
            cvAdd(result, translationsmatrix, result, NULL);

            double xb = (CV_MAT_ELEM(*result, float, 0, 0) * 5.2921508098293293e+02 / CV_MAT_ELEM(*result, float, 2, 0)) + 3.2894272028759258e+02;
            double yb = (CV_MAT_ELEM(*result, float, 1, 0) * 5.2556393630057437e+02 / CV_MAT_ELEM(*result, float, 2, 0)) + 2.6748068171871557e+02;

            int di = ((int)yb * 640) + (int)xb;
            if (glow_mid[3 * i + 1] != 255 && glow_mid[3 * i + 1] != 0) {
                pushrgb(rgb_masked, i, rgb_mid[3 * di + 0], rgb_mid[3 * di + 1], rgb_mid[3 * di + 2]);
            } else {
                pushrgb(rgb_masked, i, 255, 0, 0);
            }
        }
    }
#endif
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

// must be a multiple of 16
#define GRID_X 40
#define GRID_Y 32

// must be a multiple of 8
#define BLOCK_X 16
#define BLOCK_Y 15

static time_t last_time;
int fps = 0;
int frames = 0;

bool calibration = false;

static void kb_poll_events(void) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        kinect_shutdown();
                        exit(0);
                    case SDLK_e:
                        calibration = !calibration;
                        if (calibration)
                            median_clear_calibration();
                        break;
                    default:
                        printf("Unknown key pressed.\n");
                        break;
                }
        }
    }
}

int main(int argc, char *argv[]) {
    SDL_Surface *screen;

    median_filter_init();
    glow_filter_init();
    kinect_init();
    
#if 0
    cudaDeviceReset();

    /* init cuda */
    CUresult error_id = cuInit(0);
    if (error_id != CUDA_SUCCESS) {
        printf("cuInit(0) returned %d\n-> %s\n", error_id, getCudaDrvErrorString(error_id));
        exit(1);
    }
#endif

#if 0
    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("freenect_init() failed\n");
        return 1;
    }
    
    //freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
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
#endif
    /* Initialize SDL */
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    SDL_GL_SetAttribute( SDL_GL_RED_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE, 8 );
    SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 16 );
    SDL_GL_SetAttribute( SDL_GL_BUFFER_SIZE, 32 );
    SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
    SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL, 1);

    /* Initialize the screen / window */
    screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_DEPTH, SDL_OPENGL | SDL_HWSURFACE | SDL_NOFRAME | SDL_DOUBLEBUF);
    if (screen == 0) {
        printf("set failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_WM_SetCaption("kinectboard", "");

    glewInit();

    /* Setup viewport */
    glEnable(GL_TEXTURE_2D);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* Allocate textures and buffers to draw into (from the GPU) */
    glGenBuffers(1, &medianBufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, medianBufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 640 * 480 * 4 * sizeof(GLubyte), NULL, GL_DYNAMIC_COPY);
    cudaGLRegisterBufferObject(medianBufferID);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &medianTextureID);
    glBindTexture(GL_TEXTURE_2D, medianTextureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 640, 480, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenBuffers(1, &maskedMedianBufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, maskedMedianBufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 640 * 480 * 4 * sizeof(GLubyte), NULL, GL_DYNAMIC_COPY);
    cudaGLRegisterBufferObject(maskedMedianBufferID);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &maskedMedianTextureID);
    glBindTexture(GL_TEXTURE_2D, maskedMedianTextureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 640, 480, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenBuffers(1, &glowBufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, glowBufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 640 * 480 * 4 * sizeof(GLubyte), NULL, GL_DYNAMIC_COPY);
    cudaGLRegisterBufferObject(glowBufferID);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &glowTextureID);
    glBindTexture(GL_TEXTURE_2D, glowTextureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 640, 480, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);



    printf("gl set up.\n");
 
#if 0
    int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        freenect_shutdown(f_ctx);
        return 1;
    }
#endif

    //query_cuda_device(list);

    uchar4 *gpu_median_output,
           *gpu_masked_median_output,
           *gpu_glow_output;

    while (1) {
        //kb_poll_events(list);
        struct timeval begin, end;
        gettimeofday(&begin, NULL);

#if 0

        /* depth in den Speicher der GPU kopieren */
        static uint16_t *gpu_depth = NULL;
        static uint8_t *gpu_raw_depth = NULL;
        uchar4 *gpu_output = NULL;
        static uint8_t *gpu_out2 = NULL;
        static uint8_t *gpu_table = NULL;
        if (!gpu_depth) {
            cudaMalloc(&gpu_depth, 640*480 * sizeof(uint16_t));
            cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<uint16_t>();
            cudaError_t err = cudaBindTexture2D(NULL, &texRef, gpu_depth, &channelDesc, 640, 480, 640 * 2);
            if (err != cudaSuccess) {
                printf("nope: %s.\n", cudaGetErrorString(err));
                exit(1);
            }
        }
        if (!gpu_raw_depth)
            cudaMalloc(&gpu_raw_depth, 640*480 * 3);
        if (!gpu_out2)
            cudaMalloc(&gpu_out2, 640*480*3);
        if (!gpu_table) {
            cudaMalloc(&gpu_table, 2048);
            cudaMemcpy(gpu_table, depthPixelsLookupNearWhite, 2048, cudaMemcpyHostToDevice);
        }

        gettimeofday(&begin, NULL);
        cudaMemcpyAsync(gpu_depth, depth_buffer, 640*480 * sizeof(uint16_t), cudaMemcpyHostToDevice, 0);

        cudaThreadSynchronize();

        gettimeofday(&end, NULL);

        if (end.tv_usec < begin.tv_usec) {
            end.tv_usec += 1000000;
            begin.tv_sec += 1;
        }
        size_t usecs = (end.tv_sec - begin.tv_sec) * 1000000;
        usecs += (end.tv_usec - begin.tv_usec);
        printf("memcpy = %d\n", usecs);
        gettimeofday(&begin, NULL);

        cudaGLMapBufferObject((void**)&gpu_output, bufferID);
        cudaThreadSynchronize();
        gettimeofday(&end, NULL);

        if (end.tv_usec < begin.tv_usec) {
            end.tv_usec += 1000000;
            begin.tv_sec += 1;
        }
        usecs = (end.tv_sec - begin.tv_sec) * 1000000;
        usecs += (end.tv_usec - begin.tv_usec);
        printf("map = %d\n", usecs);
        cudaThreadSynchronize();
        gettimeofday(&begin, NULL);


        // blocksize vielfaches von 8
        // gridsize vielfaches von 16
        dim3 blocksize(BLOCK_X, BLOCK_Y);
        dim3 gridsize(GRID_X, GRID_Y);

        median_filter_gpu<<<1, 1>>>(gpu_depth, gpu_table, gpu_output);

        //cudaThreadSynchronize();
        cudaMemcpy(masked_depth_mid, gpu_out2, 640*480*3, cudaMemcpyDeviceToHost);


        gettimeofday(&end, NULL);

        if (end.tv_usec < begin.tv_usec) {
            end.tv_usec += 1000000;
            begin.tv_sec += 1;
        }
        usecs = (end.tv_sec - begin.tv_sec) * 1000000;
        usecs += (end.tv_usec - begin.tv_usec);
        printf(" usecs = %d\n", usecs);

        cudaGLUnmapBufferObject(bufferID);
        sleep(1);
#endif

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        kb_poll_events();

        gpu_median_output = NULL;
        gpu_masked_median_output = NULL;
        gpu_glow_output = NULL;

        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_median_output, medianBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_masked_median_output, maskedMedianBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_glow_output, glowBufferID));

        median_filter(take_depth_image(), gpu_median_output);
        done_depth_image();

        median_mask(calibration, gpu_median_output, gpu_masked_median_output);
        glow_filter(gpu_masked_median_output, gpu_glow_output);

        cutilSafeCall(cudaGLUnmapBufferObject(maskedMedianBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(medianBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(glowBufferID));

        //glBindBuffer(GL_PIXEL_UNPACK_BUFFER, medianBufferID);
        //glBindTexture(GL_TEXTURE_2D, medianTextureID);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, maskedMedianBufferID);
        glBindTexture(GL_TEXTURE_2D, maskedMedianTextureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

        glBegin(GL_QUADS);
          glTexCoord2f(0, 1.0f);
          glVertex2f(0, 300.0f);

          glTexCoord2f(0, 0);
          glVertex2f(0, SCREEN_HEIGHT * 1.0f);

          glTexCoord2f(1.0f, 0);
          glVertex2f(640 * 1.0f, SCREEN_HEIGHT * 1.0f);

          glTexCoord2f(1.0f, 1.0f);
          glVertex2f(640 * 1.0f, 300.0f);
        glEnd();

        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, glowBufferID);
        glBindTexture(GL_TEXTURE_2D, glowTextureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 640, 480, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

        glBegin(GL_QUADS);
          glTexCoord2f(0, 1.0f);
          glVertex2f(640.0f, 300.0f);

          glTexCoord2f(0, 0);
          glVertex2f(640.0f, SCREEN_HEIGHT * 1.0f);

          glTexCoord2f(1.0f, 0);
          glVertex2f(1280 * 1.0f, SCREEN_HEIGHT * 1.0f);

          glTexCoord2f(1.0f, 1.0f);
          glVertex2f(1280 * 1.0f, 300.0f);
        glEnd();

        SDL_GL_SwapBuffers();
    }
}
