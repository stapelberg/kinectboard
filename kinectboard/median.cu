// vim:ts=4:sw=4:expandtab
#include <stdint.h>
#include <stdio.h>

#include <cuda.h>

#define GRID_X 40
#define GRID_Y 32

#define BLOCK_X 16
#define BLOCK_Y 15

texture<uint16_t, 2> gpu_depth_tex;
static uint16_t *gpu_depth;

__global__ void median_filter_gpu(uint16_t *gpu_depth, uchar4 *gpu_output) {
    const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
    const int i = (y * 640) + x;

    float nneighbors[9] = {
        tex2D(gpu_depth_tex, x-1, y-1),
        tex2D(gpu_depth_tex, x-0, y-1),
        tex2D(gpu_depth_tex, x+1, y-1),

        tex2D(gpu_depth_tex, x-1, y-0),
        tex2D(gpu_depth_tex, x-0, y-0),
        tex2D(gpu_depth_tex, x+1, y-0),

        tex2D(gpu_depth_tex, x-1, y+1),
        tex2D(gpu_depth_tex, x-0, y+1),
        tex2D(gpu_depth_tex, x+1, y+1)
    };

    float ma, mi;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            ma = fmaxf(nneighbors[j], nneighbors[j+1]);
            mi = fminf(nneighbors[j], nneighbors[j+1]);
            nneighbors[j] = mi;
            nneighbors[j+1] = ma;
        }
    }

    int pvaln = (2048.0f * 256.0f) / (nneighbors[9/2] - 2048.0f);

    gpu_output[i].w = 0;
    gpu_output[i].x = pvaln;
    gpu_output[i].y = pvaln;
    gpu_output[i].z = pvaln;
}

void median_filter_init(void) {
    // TODO: error handling
    cudaMalloc(&gpu_depth, 640 * 480 * sizeof(uint16_t));
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<uint16_t>();
    cudaError_t cerr = cudaBindTexture2D(NULL, &gpu_depth_tex, gpu_depth, &channelDesc, 640, 480, 640 * 2);
    if (cerr != cudaSuccess) {
        printf("nope: %s.\n", cudaGetErrorString(cerr));
        exit(1);
    }
}

void median_filter(uint16_t *host_depth, uchar4 *gpu_output) {
    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    cudaMemcpy(gpu_depth, host_depth, 640 * 480 * sizeof(uint16_t), cudaMemcpyHostToDevice);

    median_filter_gpu<<<gridsize, blocksize>>>(gpu_depth, gpu_output);
    if (cudaGetLastError() != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize?\n");

    cudaThreadSynchronize();
}
