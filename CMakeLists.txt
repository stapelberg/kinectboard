# Kinectboard CMake Build system 
cmake_minimum_required (VERSION 2.8) 
project (kinectboard) 

SET(CUDA_SDK_ROOT_DIR "~/NVIDIA_GPU_Computing_SDK" CACHE STRING "Path to the NVIDIA_GPU_Computing_SDK directory.")

add_subdirectory (util) 
add_subdirectory (kinectboard)
