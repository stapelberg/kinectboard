/*
 * vim:ts=4:sw=4:expandtab
 *
 * kinectboard © 2012 Michael Stapelberg, Felix Bruckner, Pascal Krause
 * See LICENSE for licensing details.
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <libfreenect.h>
#include <err.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

/*******************************************************************************
 * RGB/depth image queues and functions
 ******************************************************************************/

#define KINECT_RGB_SIZE (640 * 480 * 3 * sizeof(uint8_t))
#define KINECT_DEPTH_SIZE (640 * 480 * sizeof(uint16_t))

struct queue_entry {
    void *data;

    /* Initialized with false */
    bool used;
};

static struct queue_entry rgb_image_queue[2];
static pthread_mutex_t rgb_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct queue_entry depth_image_queue[2];
static pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;

static void queue_init(void) {
    rgb_image_queue[0].data = malloc(KINECT_RGB_SIZE);
    rgb_image_queue[1].data = malloc(KINECT_RGB_SIZE);

    depth_image_queue[0].data = malloc(KINECT_DEPTH_SIZE);
    depth_image_queue[1].data = malloc(KINECT_DEPTH_SIZE);
    printf("init: 0 = %p, 1 = %p\n",
            depth_image_queue[0].data,
            depth_image_queue[1].data);
}

/*
 * Puts new image into the queue, possibly replacing the currently waiting
 * image.
 *
 */
static void enqueue_rgb_image(uint8_t *new_rgb_image) {
    pthread_mutex_lock(&rgb_mutex);
    /* There still is an image in the queue, we just replace it. */
    if (rgb_image_queue[0].used) {
        memcpy(rgb_image_queue[0].data, new_rgb_image, KINECT_RGB_SIZE);
    } else {
        memcpy(rgb_image_queue[0].data, new_rgb_image, KINECT_RGB_SIZE);
        rgb_image_queue[0].used = true;
    }
    pthread_mutex_unlock(&rgb_mutex);
}

static void enqueue_depth_image(uint16_t *new_depth_data) {
    pthread_mutex_lock(&depth_mutex);
    /* There still is an image in the queue, we just replace it. */
    if (depth_image_queue[0].used) {
        memcpy(depth_image_queue[0].data, new_depth_data, KINECT_DEPTH_SIZE);
    } else {
        memcpy(depth_image_queue[0].data, new_depth_data, KINECT_DEPTH_SIZE);
        depth_image_queue[0].used = true;
    }
    pthread_mutex_unlock(&depth_mutex);
}

uint8_t *take_rgb_image(void) {
    return (uint8_t*)rgb_image_queue[1].data;
}

uint16_t *take_depth_image(void) {
    return (uint16_t*)depth_image_queue[1].data;
}

/*
 * Signals that the thread which called take_depth_image() before is now done
 * with the data and the data should be discarded. This makes room for one new
 * entry in the queue.
 *
 */
void done_rgb_image(void) {
    if (rgb_image_queue[0].used) {
        pthread_mutex_lock(&rgb_mutex);
        void *tmp = rgb_image_queue[1].data;
        rgb_image_queue[1].data = rgb_image_queue[0].data;
        rgb_image_queue[0].data = tmp;
        rgb_image_queue[0].used = false;
        pthread_mutex_unlock(&rgb_mutex);
    }
}

/*
 * Signals that the thread which called take_depth_image() before is now done
 * with the data and the data should be discarded. This makes room for one new
 * entry in the queue.
 *
 */
void done_depth_image(void) {
    if (depth_image_queue[0].used) {
        pthread_mutex_lock(&depth_mutex);
        void *tmp = depth_image_queue[1].data;
        depth_image_queue[1].data = depth_image_queue[0].data;
        depth_image_queue[0].data = tmp;
        depth_image_queue[0].used = false;
        pthread_mutex_unlock(&depth_mutex);
    }
}

/*******************************************************************************
 * Freenect variables and functions
 ******************************************************************************/

static bool shutdown_requested = false;
static freenect_context *f_ctx;
static freenect_device *f_dev;
static pthread_t freenect_thread;
//volatile int die = 0;

static void *freenect_threadfunc(void *arg);

void kinect_init(void) {
    queue_init();

    if (freenect_init(&f_ctx, NULL) < 0)
        err(EXIT_FAILURE, "freenect_init() failed\n");

    //freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    int nr_devices = freenect_num_devices(f_ctx);
    printf ("Number of devices found: %d\n", nr_devices);

    if (nr_devices < 1) {
        freenect_shutdown(f_ctx);
        exit(EXIT_FAILURE);
    }

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
        printf("Could not open device\n");
        freenect_shutdown(f_ctx);
        exit(EXIT_FAILURE);
    }

    int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if (res) {
        freenect_shutdown(f_ctx);
        err(EXIT_FAILURE, "pthread_create() failed\n");
    }
}

void kinect_shutdown(void) {
    /* Set a flag so that freenect_threadfunc stops looping (that could trigger
     * a segfault. Not that we care about it, since we’re exiting anyways, but
     * it’s cleaner). */
    shutdown_requested = true;

    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
}

static uint8_t *rgb_back, *rgb_mid;

static void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp) {
    rgb_back = rgb_mid;
    freenect_set_video_buffer(dev, rgb_back);
    rgb_mid = (uint8_t*)rgb;

    enqueue_rgb_image(rgb_mid);
}

static void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
    uint16_t *depth = (uint16_t*)v_depth;

    enqueue_depth_image(depth);
}

static void *freenect_threadfunc(void *arg) {
    int accelCount = 0;

    //freenect_set_tilt_degs(f_dev, 0);
    freenect_set_led(f_dev, LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_set_video_buffer(f_dev, rgb_back);

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");

    while (!shutdown_requested && freenect_process_events(f_ctx) >= 0) {
        //Throttle the text output
        if (accelCount++ >= 2000) {
            accelCount = 0;
            freenect_raw_tilt_state* state;
            freenect_update_tilt_state(f_dev);
            state = freenect_get_tilt_state(f_dev);
            double dx,dy,dz;
            freenect_get_mks_accel(state, &dx, &dy, &dz);
            printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
            fflush(stdout);
        }
    }

    printf("-- done!\n");
    return NULL;
}
