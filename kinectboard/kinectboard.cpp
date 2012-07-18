/*
 * vim:ts=4:sw=4:expandtab
 *
 * kinectboard © 2012 Michael Stapelberg, Felix Bruckner, Pascal Krause
 * See LICENSE for licensing details.
 *
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
#include <unistd.h>
#include <getopt.h>

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
#include "maskrgb.h"
#include "loadimg.h"
#include "kinectboard_ui.h"
#include "cudadeviceinfo.h"

#include "kinectboard_images.h"

// Cuda
#include <cuda.h>
#include <cutil_inline.h>
#include <cuda_gl_interop.h>
#include <drvapi_error_string.h>

// kinect image height (480) + space for controls
#define SCREEN_HEIGHT 480 + 288
#define SCREEN_DEPTH 32

int SCREEN_WIDTH = 1280;

GLuint rawDepthBufferID;
GLuint rawRgbBufferID;
GLuint medianBufferID;
GLuint maskedMedianBufferID;
GLuint glowBufferID;
GLuint maskRgbBufferID;
GLuint contRgbBufferID;
GLuint calibrationBufferID;

GLuint rawDepthTextureID;
GLuint rawRgbTextureID;
GLuint medianTextureID;
GLuint maskedMedianTextureID;
GLuint glowTextureID;
GLuint maskRgbTextureID;
GLuint contRgbTextureID;
GLuint calibrationTextureID;

uint16_t *depth_buffer;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

extern int ANIMATION_ONE_STEP;

// Die Referenz-Farbe (vom Nutzer ausgewählt). Wird normiert gespeichert:
// Jeder Farbanteil wird durch √(r² + g² + b²) geteilt.
float4 reference_color = {
    0.006413,
    0.333484,
    0.942734,
    -1
};

bool calibration_mode = false;
int calibration_step = 0;

uint2 calibrated_offset;

struct calibration_values {
    uint2 tl;
    uint2 tr;
    uint2 bl;
    uint2 br;
} calibration_values;


uint16_t glow_start = 0;
uint16_t glow_end = 78;

double FILTER_DISTANCE = 0.2f;

double DEPTH_MASK_MULTIPLIER = 0.0f;

int DEPTH_MASK_THRESHOLD = 2;

SDL_Surface *chosen_color_surface;

bool calibration = false;
bool fullscreen_canvas = false;

static void select_reference_color(int x, int y) {
    static float offset = SCREEN_WIDTH/1280.0f;
    printf("(After scaling) Handling click on x = %d, y = %d\n", x, y);
    GLuint left_buffer, right_buffer, buffer;
    kb_images_current_buffers(&left_buffer, &right_buffer);
    printf("left texture id = %d, right = %d\n", left_buffer, right_buffer);
    uchar4 *gpu_buffer = NULL;
    if (x < 640*offset) {
        /* The clicked pixel is in the left image */
        buffer = left_buffer;
    } else {
        /* The clicked pixel is in the right image */
        buffer = right_buffer;
        x -= 640*offset;
    }
    cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_buffer, buffer));
    uchar4 pixel;
    cudaMemcpy(&pixel, gpu_buffer + (y * 640) + x, sizeof(uchar4), cudaMemcpyDeviceToHost);
    printf("pixel-value: %d, %d, %d (%d)\n", pixel.x, pixel.y, pixel.z, pixel.w);
    static char rgbbuffer[4096];
    snprintf(rgbbuffer, sizeof(rgbbuffer), "%d,%d,%d", pixel.x, pixel.y, pixel.z);
    kb_ui_call_javascript("SetRGB", rgbbuffer);

    double r = pixel.z;
    double g = pixel.y;
    double b = pixel.x;
    double nominator = sqrt((r * r) + (g * g) + (b * b));
    printf("nominator = %f\n", nominator);
    reference_color.x = r / nominator;
    reference_color.y = g / nominator;
    reference_color.z = b / nominator;
    printf("reference_color final: %f %f %f\n",
            reference_color.x, reference_color.y, reference_color.z);

    cutilSafeCall(cudaGLUnmapBufferObject(buffer));
}

static void handle_calibration_click(int x, int y) {
    printf("Calibration click on x = %d, y = %d\n", x, y);
    if (calibration_step == 0) {
        printf("top left\n");
        kb_ui_call_javascript("MarkAsClicked", "tl");
        calibration_values.tl = (uint2){ x, y };
    } else if (calibration_step == 1) {
        printf("top right\n");
        kb_ui_call_javascript("MarkAsClicked", "tr");
        calibration_values.tr = (uint2){ x, y };
    } else if (calibration_step == 2) {
        printf("bottom left\n");
        kb_ui_call_javascript("MarkAsClicked", "bl");
        calibration_values.bl = (uint2){ x, y };
    } else {
        printf("bottom right\n");
        kb_ui_call_javascript("MarkAsClicked", "br");
        calibration_values.br = (uint2){ x, y };

        int x_sum = 0;
        int y_sum = 0;

        x_sum += abs(150 - calibration_values.tl.x);
        x_sum += abs(550 - calibration_values.tr.x);
        x_sum += abs(150 - calibration_values.bl.x);
        x_sum += abs(550 - calibration_values.br.x);

        y_sum += abs(300 - calibration_values.tl.y);
        y_sum += abs(300 - calibration_values.tr.y);
        y_sum += abs(400 - calibration_values.bl.y);
        y_sum += abs(400 - calibration_values.br.y);

        x_sum /= 4;
        y_sum /= 4;

        printf("average offset: %d x, %d y\n",
                x_sum, y_sum);
    }

    calibration_step++;
}

// Callbacks

// Calibration button callback
static void run_calibration_callback(void) {
    calibration = !calibration;
    if (calibration) 
        median_clear_calibration();


} 
static void start_calibration_callback(void) {
    calibration_mode = true;
    calibration_step = 0;
} 

static void end_calibration_callback(void) {
    calibration_mode = false;
} 

// Exit Callback
static void exit_callback() {
    kinect_shutdown();
    exit(0);    
}

static void set_distance_threshold_callback(float val) {
    printf("%f", val);
    FILTER_DISTANCE = val;
}

static void set_depth_multiplier_callback(float val) {
    printf("%f", val);
}
static void set_depth_difference_threshold_callback(float val) {
    printf("%f", val);
}
static void set_glow_area_start_callback(float val) {
    printf("%f", val);
    glow_start = val;
}
static void set_glow_area_end_callback(float val) {
    printf("%f", val);
    glow_end = val;
}

static void kb_poll_events(void) {
    SDL_Event event;
    static float offset = SCREEN_WIDTH/1280.0f;
    int x, y;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {  
            case SDL_MOUSEMOTION:
                kb_ui_inject_mouse(event.motion.x, event.motion.y);
                break;
            case SDL_MOUSEBUTTONUP:
                kb_ui_inject_mouse_button(event.button.button, false);
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.y < 480 * (SCREEN_WIDTH/1280.f) || fullscreen_canvas) {
                    x = event.button.x;
                    y = event.button.y;
                    printf("Handling click on x = %d, y = %d\n", x, y);
                    if (fullscreen_canvas) {
                        x = x * 0.625f;
                        y = y * 0.625f;
                    } else {
                        x *= (1280.0f / SCREEN_WIDTH);
                        y *= (1280.0f / SCREEN_WIDTH);
                    }
                    printf("after scaling: %d, %d\n", x, y);
                    if (calibration_mode) {
                        handle_calibration_click(x, y);
                    } else {
                        select_reference_color(x, y);
                    }
                } else {
                    kb_ui_inject_mouse_button(event.button.button, true);
                }
                break;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        exit_callback();
                    case SDLK_LEFT:
                        kb_images_scroll_left();
                        break;
                    case SDLK_RIGHT:
                        kb_images_scroll_right();
                        break;
                    case SDLK_e:
                        run_calibration_callback();
                        break;
                    case SDLK_f:
                        fullscreen_canvas = !fullscreen_canvas;
                        break;
                    case SDLK_c:
                        mask_rgb_clear_cont();
                        break;
                    default:
                        printf("Unknown key pressed.\n");
                        break;
                }
                break;
        }
    }
}

/*
 * Allocates a GL buffer and texture to be used on the GPU.
 *
 */
static void allocateGLTexture(GLuint *bufferID, GLuint *textureID) {
    glGenBuffers(1, bufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, *bufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, 640 * 480 * 4 * sizeof(GLubyte), NULL, GL_DYNAMIC_COPY);
    cudaGLRegisterBufferObject(*bufferID);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, textureID);
    glBindTexture(GL_TEXTURE_2D, *textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 640, 480, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

static void loadTextureFromFile(const char* file, GLuint *bufferID, GLuint *textureID) {
    SDL_Surface* surface = SDL_LoadBMP(file);

    if(!surface)
        return;

    glGenBuffers(1, bufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, *bufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, surface->w * surface->h * surface->format->BytesPerPixel 
        * sizeof(GLubyte), surface->pixels, GL_STATIC_DRAW);
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, textureID);
    glBindTexture(GL_TEXTURE_2D, *textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, surface->w, surface->h, 0, GL_BGR, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

int main(int argc, char *argv[]) {
    SDL_Surface *screen;
    static struct option long_options[] = {
        {"no-kinect", no_argument, 0, 'k'},
        {"fullscreen", optional_argument, 0, 'f'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };
    int option_index = 0, opt;
    bool init_kinect = true;
    bool fullscreen_mode = false;
    char *fullscreen_resolution = NULL;

    while ((opt = getopt_long(argc, argv, "khf:", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'k':
                init_kinect = false;
                printf("Not initializing kinect (-k passed)\n");
                break;
            case 'f':
                printf("Starting in fullscreen mode\n");
                fullscreen_mode = true;
                if (optarg)
                    fullscreen_resolution = strdup(optarg);
                break;
            case 'h':
                printf("Syntax: %s [-k] [-h]\n", argv[0]);
                printf("\t--no-kinect\tDisables initializing kinect\n");
                printf("\t--fullscreen\tEnable fullscreen mode (default is windowed)\n");
                printf("\t\t\t(--fullscreen=1024x768 to overwrite the resolution)\n");
                exit(0);
                break;
        }
    }

    median_filter_init();
    glow_filter_init();
    if (init_kinect)
        kinect_init();
    mask_rgb_init();
    loadimg_init();
    
    /* Initialize SDL */
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL, 1);

    /* Initialize the screen / window */
    if (fullscreen_mode && fullscreen_resolution != NULL) {
        if (sscanf(fullscreen_resolution, "%dx", &SCREEN_WIDTH) != 1) {
            fprintf(stderr, "Invalid resolution specified: %s (needs to be WxH, e.g. 1024x768)\n", fullscreen_resolution);
            exit(1);
        }
        printf("Setting width to %d\n", SCREEN_WIDTH);
    }
    int flags = SDL_OPENGL | SDL_HWSURFACE | SDL_NOFRAME | SDL_DOUBLEBUF;
    if (fullscreen_mode)
        flags |= SDL_FULLSCREEN;
    screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_DEPTH, flags);
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
    
    kb_ui_init();
    
    // Register callbacks
    kb_ui_register_void_callback("Exit",exit_callback);
    kb_ui_register_void_callback("Calibrate",run_calibration_callback);
    kb_ui_register_void_callback("StartCalibration",start_calibration_callback);
    kb_ui_register_void_callback("EndCalibration",end_calibration_callback);
    kb_ui_register_void_callback("ImageRight",kb_images_scroll_right);
    kb_ui_register_void_callback("ImageLeft",kb_images_scroll_left);
    kb_ui_register_value_callback("SetDistanceThreshold", set_distance_threshold_callback);
    kb_ui_register_value_callback("SetDepthMultiplier", set_depth_multiplier_callback);
    kb_ui_register_value_callback("SetDepthDifferenceThreshold", set_depth_difference_threshold_callback);
    kb_ui_register_value_callback("SetGlowAreaStart", set_glow_area_start_callback);
    kb_ui_register_value_callback("SetGlowAreaEnd", set_glow_area_end_callback);

    kb_ui_call_javascript("SetRGB", "142,51,19");

    // The CUDA Device Info requires a valid UI since the info is displayed there
    print_cuda_device_info();

    /* Allocate textures and buffers to draw into (from the GPU) */
    allocateGLTexture(&rawDepthBufferID, &rawDepthTextureID);
    allocateGLTexture(&medianBufferID, &medianTextureID);
    allocateGLTexture(&maskedMedianBufferID, &maskedMedianTextureID);
    allocateGLTexture(&glowBufferID, &glowTextureID);
    allocateGLTexture(&rawRgbBufferID, &rawRgbTextureID);
    allocateGLTexture(&maskRgbBufferID, &maskRgbTextureID);
    allocateGLTexture(&contRgbBufferID, &contRgbTextureID);

    kb_image_create("Raw depth image", rawDepthBufferID, rawDepthTextureID);
    kb_image_create("Median-filtered depth image", medianBufferID, medianTextureID);
    kb_image_create("Masked depth image", maskedMedianBufferID, maskedMedianTextureID);
    kb_image_create("Glowing depth", glowBufferID, glowTextureID);
    kb_image_create("Raw RGB image", rawRgbBufferID, rawRgbTextureID);
    kb_image_create("Masked kinect RGB image", maskRgbBufferID, maskRgbTextureID);
    kb_image_create("Cont RGB image", contRgbBufferID, contRgbTextureID);

    // Load a Texture
    //loadTextureFromFile("../data/calibration.bmp", &calibrationBufferID, &calibrationTextureID);
    //kb_image_create("Calibration", calibrationBufferID, calibrationTextureID);

    SDL_Surface* surface = SDL_LoadBMP("../data/calibration.bmp");
    printf("bpp = %d\n", surface->format->BytesPerPixel);
    uint8_t *gpu_calibration_buffer;
    cudaMalloc((void**)&gpu_calibration_buffer, 640 * 480 * 3 * sizeof(uint8_t));
    //cudaMemcpy(gpu_calibration_buffer, surface->pixels, 640*480*3*sizeof(uint8_t), cudaMemcpyHostToDevice);
    loadimg_convert((uint8_t*)surface->pixels, gpu_calibration_buffer);


    printf("gl set up.\n");
 
    uchar4 *gpu_median_output,
           *gpu_masked_median_output,
           *gpu_glow_output,
           *gpu_mask_rgb_output,
           *gpu_raw_depth_output,
           *gpu_raw_rgb_output,
           *gpu_cont_rgb_output;

    int fps = 0;
    int last_time = 0;
    int current_time;

    while (1) {
        /* FPS counter */
        current_time = SDL_GetTicks();
        if ((current_time - last_time) >= 1000) {
            static char buffer[20] = {0};
            sprintf(buffer, "%d FPS", fps);
            SDL_WM_SetCaption(buffer, 0);
            kb_ui_call_javascript("SetFPS",buffer);
            fps = 0;
            last_time = current_time;
        }

        //kb_poll_events(list);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        /* Reset viewport for rendering our images, it was modified by
         * kb_ui_render(). */
        glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        kb_poll_events();

        gpu_median_output = NULL;
        gpu_masked_median_output = NULL;
        gpu_glow_output = NULL;
        gpu_mask_rgb_output = NULL;
        gpu_raw_depth_output = NULL;
        gpu_raw_rgb_output = NULL;
        gpu_cont_rgb_output = NULL;

        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_raw_depth_output, rawDepthBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_median_output, medianBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_masked_median_output, maskedMedianBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_glow_output, glowBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_mask_rgb_output, maskRgbBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_raw_rgb_output, rawRgbBufferID));
        cutilSafeCall(cudaGLMapBufferObject((void**)&gpu_cont_rgb_output, contRgbBufferID));

        // XXX: Potential for optimization: We currently call functions like
        // median_filter(), median_mask() and mask_rgb() which are all
        // blocking. However, we could launch the kernel and perform more work
        // on the CPU while waiting for the kernel to complete (or maybe even
        // launch some in parallel and/or use async events).

        median_filter(take_depth_image(), gpu_median_output, gpu_raw_depth_output);
        done_depth_image();

        median_mask(calibration, gpu_median_output, gpu_masked_median_output);
        glow_filter(gpu_masked_median_output, gpu_glow_output, glow_start, glow_end);

        mask_rgb(gpu_glow_output, take_rgb_image(), gpu_mask_rgb_output, gpu_raw_rgb_output, gpu_cont_rgb_output, reference_color, FILTER_DISTANCE, gpu_calibration_buffer);
        done_rgb_image();

        cutilSafeCall(cudaGLUnmapBufferObject(maskedMedianBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(medianBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(glowBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(maskRgbBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(rawDepthBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(rawRgbBufferID));
        cutilSafeCall(cudaGLUnmapBufferObject(contRgbBufferID));

        if(fullscreen_canvas) {
           kb_images_render_canvas_only();
        } else {
            kb_images_render();
            kb_ui_update();
            kb_ui_render();
        }
            SDL_GL_SwapBuffers();
        fps++;
    }
}
