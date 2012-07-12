######################################################################################
# Kinectboard CMake
######################################################################################

#Include FindCUDA script
INCLUDE(FindCUDA)

if (WIN32)
  set_source_files_properties(kinectboard.cu kinectboard_controls.c kinectboard_images.c PROPERTIES LANGUAGE CXX)

  set(THREADS_USE_PTHREADS_WIN32 true)
  find_package(Threads REQUIRED)

  include_directories(${THREADS_PTHREADS_INCLUDE_DIR})
endif()

# Mac just has everything already
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
  target_link_libraries(kinectboard freenect)
# Linux, not so much
else()

  find_package(Threads REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  find_package(SDL REQUIRED)
  find_package(SDL_ttf REQUIRED)
  find_package(CUDA)

  link_directories (${CMAKE_SOURCE_DIR}/lib) 

  set(CMAKE_C_FLAGS "-Wall -g -O3")

  
  # CUDA Utility flags
  # LINK_LIBRARIES("-L/${CUDA_SDK_ROOT_DIR}/C/lib")
  include_directories(${CUDA_SDK_ROOT_DIR}/C/common/inc)
  include_directories(${CUDA_SDK_ROOT_DIR}/shared/inc)

  include(FindPkgConfig)
  pkg_check_modules(FREENECT "libfreenect")

  # CUDA NVCC Flags
  set(CUDA_NVCC_FLAGS "-O3" "--compiler-bindir" "/usr/bin/gcc-4.4" "--library-path" "${CMAKE_SOURCE_DIR}/lib" ${FREENECT_CFLAGS}) 

  add_definitions(${FREENECT_CFLAGS})
  link_libraries(${FREENECT_LDFLAGS})

  pkg_check_modules(GLEW "glew")
  add_definitions(${GLEW_CFLAGS})
  link_libraries(${GLEW_LDFLAGS})

  include_directories(../util ../common include ${OPENGL_INCLUDE_DIRS} ${SDL_INCLUDE_DIR} ${SDLTTF_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS})

  CUDA_ADD_EXECUTABLE(kinectboard kinectboard.cpp median.cu glow.cu kinect.cpp)
  target_link_libraries(kinectboard freenect kinectboard_util ${CUDA_LIBRARIES} ${SDLTTF_LIBRARY} ${SDL_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})
endif()

install (TARGETS kinectboard DESTINATION bin)
