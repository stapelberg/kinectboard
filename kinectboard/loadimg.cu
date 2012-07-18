/*
 * vim:ts=4:sw=4:expandtab
 *
 * kinectboard © 2012 Michael Stapelberg, Felix Bruckner, Pascal Krause
 * See LICENSE for licensing details.
 *
 */
#include <stdint.h>
#include <stdio.h>

#include <cuda.h>

#define GRID_X 40
#define GRID_Y 32

#define BLOCK_X 16
#define BLOCK_Y 15

static uint8_t *gpu_temp;

__global__ void load_gpu(uint8_t *gpu_in, uint8_t *gpu_out) {
    const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
    const int i = (y * 640) + x;

    gpu_out[3 * i + 0] = gpu_in[3 * i + 2];
    gpu_out[3 * i + 1] = gpu_in[3 * i + 1];
    gpu_out[3 * i + 2] = gpu_in[3 * i + 0];
}

void loadimg_init(void) {
    cudaMalloc(&gpu_temp, 640 * 480 * 3 * sizeof(uint8_t));
}

void loadimg_convert(uint8_t *host_pixels, uint8_t *gpu_buffer) {
    cudaMemcpy(gpu_temp, host_pixels, 640 * 480 * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);

    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    load_gpu<<<gridsize, blocksize>>>(gpu_temp, gpu_buffer);
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize? %s\n", cudaGetErrorString(err));

    cudaThreadSynchronize();
}
