// vim:ts=4:sw=4:expandtab
#include <stdint.h>
#include <stdio.h>

#include <cuda.h>
#include <cutil_inline.h>

#define GRID_X 40
#define GRID_Y 32

#define BLOCK_X 16
#define BLOCK_Y 15

texture<uchar4, 2> gpu_median_masked_tex;
static cudaChannelFormatDesc channelDesc;

/*
 * This filter adds a "safety net" of 10 pixels around each pixel which was
 * detected as different in the previous step (the median masking).
 *
 */
__global__ void glow_gpu(uchar4 *gpu_median_masked, uchar4 *gpu_output) {
    const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
    const int i = (y * 640) + x;

    for (int grow = (y-10); grow < (y+10); grow++) {
        for (int gcol = (x-10); gcol < (x+10); gcol++) {
            /* TODO: check whether it’s within glow_start and glow_end */
            if (tex2D(gpu_median_masked_tex, gcol, grow).x == 255) {
                gpu_output[i].w = 0;
                gpu_output[i].x = 0;
                gpu_output[i].y = 0;
                gpu_output[i].z = 255;
                return;
            }
        }
    }

    gpu_output[i].w = 0;
    gpu_output[i].x = 0;
    gpu_output[i].y = 0;
    gpu_output[i].z = 0;
}

void glow_filter_init(void) {
    channelDesc = cudaCreateChannelDesc<uchar4>();
}

void glow_filter(uchar4 *gpu_median_masked, uchar4 *gpu_output) {
    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    // XXX: Maybe we could refactor the code so that we don’t need to bind the
    // texture all over again? Nevertheless, the runtime savings are worth
    // doing it.
    cutilSafeCall(cudaBindTexture2D(NULL, &gpu_median_masked_tex, gpu_median_masked, &channelDesc, 640, 480, 640 * sizeof(uchar4)));

    glow_gpu<<<gridsize, blocksize>>>(gpu_median_masked, gpu_output);
    if (cudaGetLastError() != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize?\n");

    cudaThreadSynchronize();
    cutilSafeCall(cudaUnbindTexture(&gpu_median_masked_tex));
}
