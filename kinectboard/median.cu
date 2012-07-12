// vim:ts=4:sw=4:expandtab
#include <stdint.h>
#include <stdio.h>

#include <cuda.h>

#define GRID_X 40
#define GRID_Y 32

#define BLOCK_X 16
#define BLOCK_Y 15

struct range {
    float min_val;
    float max_val;
};

texture<uint16_t, 2> gpu_depth_tex;
//texture<float2, 2> gpu_ranges_tex;
static uint16_t *gpu_depth;
static float2 *gpu_ranges;

/*
 * Runs a 3x3 median filter (5x5 is too slow) over the data in gpu_depth
 * (accessed through the texture gpu_depth_tex). The (median-filtered) depth
 * image is stored in gpu_output.
 *
 */
__global__ void median_filter_gpu(uchar4 *gpu_output) {
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

/*
 * Masks the (median-filtered) depth image from median_output. When
 * 'calibration' is true, the value ranges of each pixel are saved. When it’s
 * false, only pixels whose values are within the stored range will be left
 * untouched in the image. The rest will be marked red (0xFF0000).
 *
 */
__global__ void median_mask_gpu(bool calibration, uchar4 *median_output, uchar4 *gpu_output, float2 *gpu_ranges) {
    const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
    const int i = (y * 640) + x;

    /* We use .x, but x == y == z in this case. */
    float median = median_output[i].x;

    if (calibration) {
        gpu_ranges[i].x = fminf(gpu_ranges[i].x, median);
        gpu_ranges[i].y = fmaxf(gpu_ranges[i].y, median);

        gpu_output[i].w = 0;
        gpu_output[i].x = median;
        gpu_output[i].y = median;
        gpu_output[i].z = median;
    } else {
        float difference = 0;
        /* Handles the case in which median < gpu_ranges[i].x (smaller than the
         * minimum) */
        difference = fmaxf(difference, (gpu_ranges[i].x - median));
        difference = fmaxf(difference, (median - gpu_ranges[i].y));
        if (difference > 2) {
            gpu_output[i].w = 0;
            gpu_output[i].x = 255;
            gpu_output[i].y = 0;
            gpu_output[i].z = 0;
        } else {
            gpu_output[i].w = 0;
            gpu_output[i].x = median;
            gpu_output[i].y = median;
            gpu_output[i].z = median;
        }
    }
}

void median_filter_init(void) {
    // TODO: error handling
    cudaMalloc(&gpu_depth, 640 * 480 * sizeof(uint16_t));
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<uint16_t>();
    cudaError_t cerr = cudaBindTexture2D(NULL, &gpu_depth_tex, gpu_depth, &channelDesc, 640, 480, 640 * sizeof(uint16_t));
    if (cerr != cudaSuccess) {
        printf("nope: %s.\n", cudaGetErrorString(cerr));
        exit(1);
    }

    cudaMalloc(&gpu_ranges, 640 * 480 * sizeof(float2));
#if 0
    channelDesc = cudaCreateChannelDesc<float2>();
    cerr = cudaBindTexture2D(NULL, &gpu_ranges_tex, gpu_ranges, &channelDesc, 640, 480, 640 * sizeof(float2));
    if (cerr != cudaSuccess) {
        printf("nope: %s.\n", cudaGetErrorString(cerr));
        exit(1);
    }
#endif
}

void median_clear_calibration(void) {
    float2 clear[640 * 480];
    for (int i = 0; i < (640 * 480); i++) {
        clear[i].x = 999999;
        clear[i].y = -999999;
    }

    cudaMemcpy(gpu_ranges, clear, 640 * 480 * sizeof(float2), cudaMemcpyHostToDevice);
    cudaThreadSynchronize();
}

void median_filter(uint16_t *host_depth, uchar4 *gpu_output) {
    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    cudaMemcpy(gpu_depth, host_depth, 640 * 480 * sizeof(uint16_t), cudaMemcpyHostToDevice);

    median_filter_gpu<<<gridsize, blocksize>>>(gpu_output);
    if (cudaGetLastError() != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize?\n");

    cudaThreadSynchronize();
}

void median_mask(bool calibration, uchar4 *gpu_median_output, uchar4 *gpu_output) {
    dim3 blocksize(BLOCK_X, BLOCK_Y);
    dim3 gridsize(GRID_X, GRID_Y);

    median_mask_gpu<<<gridsize, blocksize>>>(calibration, gpu_median_output, gpu_output, gpu_ranges);
    if (cudaGetLastError() != cudaSuccess)
        printf("Could not call kernel. Wrong gridsize/blocksize?\n");

    cudaThreadSynchronize();
}
