// vim:ts=4:sw=4:expandtab
#include <stdint.h>
#include <stdio.h>

#include <cuda.h>

#define GRID_X 40
#define GRID_Y 32

#define BLOCK_X 16
#define BLOCK_Y 15

static uint8_t *gpu_rgb_image;
static uint16_t *gpu_depth_image;
static uchar4 blank_image[640*480];

/*
 * Translate the coordinates from depth to RGB image, then apply the values
 * from the glow filter as a mask to the RGB image. In gpu_output we will have
 * the RGB values for all areas which are relevant.
 */
__global__ void mask_rgb_gpu(uchar4 *gpu_glow, uint8_t *gpu_rgb_image, uchar4 *gpu_output, uchar4 *gpu_raw_rgb_output) {
    const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
    const int i = (y * 640) + x;

    const float rotationsmatrix[9] = {
        9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02,
        -1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02,
        1.7470421412464927e-02, 1.2275341476520762e-02, 9.9977202419716948e-01
    };

    const float translationsmatrix[3] = {
        1.9985242312092553e-02,
        -7.4423738761617583e-04,
        -1.0916736334336222e-02
    };

    gpu_raw_rgb_output[i].w = 0;
    gpu_raw_rgb_output[i].x = gpu_rgb_image[3 * i + 0];
    gpu_raw_rgb_output[i].y = gpu_rgb_image[3 * i + 1];
    gpu_raw_rgb_output[i].z = gpu_rgb_image[3 * i + 2];

    uint16_t depth = gpu_glow[i].z;
    /* Punkte mit Tiefenwert == 0 müssen wir überspringen, denn damit
     * funktioniert die Berechnung nicht. Da die relevanten Werte in der Praxis
     * != 0 sind, ist das egal. */
    if (depth == 0) {
        return;
    }

    float P3D[3] = {
        (((x - 3.3930780975300314e+02) * depth) / 5.9421434211923247e+02),
        (((y - 2.4273913761751615e+02) * depth) / 5.9104053696870778e+02),
        depth
    };
    float result0;
    float result1;
    float result2;

    /* P3D mit der Rotationsmatrix multiplizieren */
    result0 = ((P3D[0] * rotationsmatrix[0]) + (P3D[1] * rotationsmatrix[1]) + (P3D[2] * rotationsmatrix[2]));
    result1 = ((P3D[0] * rotationsmatrix[3]) + (P3D[1] * rotationsmatrix[4]) + (P3D[2] * rotationsmatrix[5]));
    result2 = ((P3D[0] * rotationsmatrix[6]) + (P3D[1] * rotationsmatrix[7]) + (P3D[2] * rotationsmatrix[8]));

    /* Die Translationsmatrix addieren */
    result0 = result0 + translationsmatrix[0];
    result1 = result1 + translationsmatrix[1];
    result2 = result2 + translationsmatrix[2];

    float xb = (result0 * 5.2921508098293293e+02 / result2) + 3.2894272028759258e+02;
    float yb = (result1 * 5.2556393630057437e+02 / result2) + 2.6748068171871557e+02;

    int di = ((int)yb * 640) + (int)xb;

    gpu_output[i].w = 0;
    gpu_output[i].x = gpu_rgb_image[3 * di + 0];
    gpu_output[i].y = gpu_rgb_image[3 * di + 1];
    gpu_output[i].z = gpu_rgb_image[3 * di + 2];
}

void mask_rgb_init(void) {
    cudaMalloc(&gpu_rgb_image, 640 * 480 * 3 * sizeof(uint8_t));
    cudaMalloc(&gpu_depth_image, 640 * 480 * sizeof(uint16_t));
    memset(blank_image, '\0', 640 * 480 * sizeof(uchar4));
}

void mask_rgb(uchar4 *gpu_glow_output, uint8_t *rgb_image, uchar4 *gpu_output, uchar4 *gpu_raw_rgb_output) {
    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    cudaError_t err = cudaMemcpy(gpu_rgb_image, rgb_image, 640 * 480 * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
        printf("malloc? %s\n", cudaGetErrorString(err));

    cudaMemcpy(gpu_output, blank_image, 640*480 * sizeof(uchar4), cudaMemcpyHostToDevice);

    mask_rgb_gpu<<<gridsize, blocksize>>>(gpu_glow_output, gpu_rgb_image, gpu_output, gpu_raw_rgb_output);
    err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize? %s\n", cudaGetErrorString(err));

    cudaThreadSynchronize();
}