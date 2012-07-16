// Cuda
#include <cuda.h>
#include <drvapi_error_string.h>
// undefine macros which are in OpenCV *and* CUDA
#undef MAX
#undef MIN
// Shared Utilities (QA Testing)
//#include <shrUtils.h>
//#include <shrQATest.h>
#include <string>
#include <sstream>
#include "kinectboard_ui.h"

/* ************************************************************************************************************* */
inline int ConvertSMVer2Cores(int major, int minor)
{
    // Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
    typedef struct {
        int SM; // 0xMm (hexidecimal notation), M = SM Major version, and m = SM minor version
        int Cores;
    } sSMtoCores;

    sSMtoCores nGpuArchCoresPerSM[] = 
    { { 0x10,  8 },
      { 0x11,  8 },
      { 0x12,  8 },
      { 0x13,  8 },
      { 0x20, 32 },
      { 0x21, 48 },
      {   -1, -1 }
    };

    int index = 0;
    while (nGpuArchCoresPerSM[index].SM != -1) {
        if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor) ) {
            return nGpuArchCoresPerSM[index].Cores;
        }
        index++;
    }
    printf("MapSMtoCores undefined SMversion %d.%d!\n", major, minor);
    return -1;
}

/* ************************************************************************************************************* */
inline void get_cuda_attrib(int *attribute, CUdevice_attribute device_attribute, int device)
{
    CUresult error =    cuDeviceGetAttribute( attribute, device_attribute, device );

    if( CUDA_SUCCESS != error) {
        fprintf(stderr, "cuSafeCallNoSync() Driver API error = %04d from file <%s>, line %i.\n",
                error, __FILE__, __LINE__);
        exit(-1);
    }
}

/* ************************************************************************************************************* */
void print_cuda_device_info() {
    int deviceCount = 0;
    int dev = 0, driverVersion = 0, runtimeVersion = 0;     
    cudaDeviceProp deviceProp;
    std::stringstream sstream;

    /* init cuda */
    CUresult error_id = cuInit(0);
    if (error_id != CUDA_SUCCESS) {
        printf("cuInit(0) returned %d\n-> %s\n", error_id, getCudaDrvErrorString(error_id));
        exit(1);
    }

    error_id = cuDeviceGetCount(&deviceCount);
    if (error_id != CUDA_SUCCESS) {
        printf( "cuDeviceGetCount returned %d\n-> %s\n", (int)error_id, getCudaDrvErrorString(error_id) );
        exit(1);
    }

    /* Query Cuda Device */
    cudaGetDeviceProperties(&deviceProp, dev);

    char buf[256];
    
    sprintf(buf, "Device %d:|\"%s\":", dev, deviceProp.name);

    #if CUDART_VERSION >= 2020
    // Console log
    cudaDriverGetVersion(&driverVersion);
    cudaRuntimeGetVersion(&runtimeVersion);
    
    
    sprintf(buf, "CUDA Driver Version / Runtime Version:|%d.%d / %d.%d", 
            driverVersion/1000, (driverVersion%100)/10, runtimeVersion/1000, (runtimeVersion%100)/10);
    sstream << buf << "|";

    #endif
    sprintf(buf, "CUDA Compute Capability:|%d.%d", deviceProp.major, deviceProp.minor);
    sstream << buf << "|";
    
    
    sprintf(buf, "Total amount of global memory:|%.0f MBytes", 
          (float)deviceProp.totalGlobalMem/1048576.0f);
    sstream << buf << "|";

    #if CUDART_VERSION >= 2000
    sprintf(buf, "(<b>%2d</b>) Multiprocessors x (<b>%3d</b>) CUDA Cores/MP:|%d CUDA Cores", 
            deviceProp.multiProcessorCount,
            ConvertSMVer2Cores(deviceProp.major, deviceProp.minor),
            ConvertSMVer2Cores(deviceProp.major, deviceProp.minor) * deviceProp.multiProcessorCount);
    //CUDA_CORES = ConvertSMVer2Cores(deviceProp.major, deviceProp.minor) * deviceProp.multiProcessorCount;
    sstream << buf << "|";
    #endif
    sprintf(buf, "GPU Clock rate|%.0f MHz (%0.2f GHz)", 
            deviceProp.clockRate * 1e-3f, deviceProp.clockRate * 1e-6f);
    sstream << buf << "|";
    
    int memoryClock;
    get_cuda_attrib( &memoryClock, CU_DEVICE_ATTRIBUTE_MEMORY_CLOCK_RATE, dev );
    sprintf(buf, "Memory Clock rate:|%.0f Mhz", memoryClock * 1e-3f);
    sstream << buf << "|";
    
    int memBusWidth;
    get_cuda_attrib( &memBusWidth, CU_DEVICE_ATTRIBUTE_GLOBAL_MEMORY_BUS_WIDTH, dev );
    sprintf(buf, "Memory Bus Width|%d-bit", memBusWidth);
    sstream << buf << "|";

    kb_ui_call_javascript("SetCUDAInfo", sstream.str().c_str());
     
}