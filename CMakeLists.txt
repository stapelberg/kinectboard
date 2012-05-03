######################################################################################
# Kinectboard CMake
######################################################################################
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_FLAGS "-Wall -g")

if (WIN32)
  set_source_files_properties(kinectboard.c kinectboard_controls.c PROPERTIES LANGUAGE CXX)

  set(THREADS_USE_PTHREADS_WIN32 true)
  find_package(Threads REQUIRED)

  include_directories(${THREADS_PTHREADS_INCLUDE_DIR})
endif()

add_executable(kinectboard kinectboard.c kinectboard_controls.c kinectboard_images.c)

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

  include(FindPkgConfig)
  pkg_check_modules(FREENECT "libfreenect")

  add_definitions(${FREENECT_CFLAGS})
  link_libraries(${FREENECT_LDFLAGS})

  include_directories(${OPENGL_INCLUDE_DIRS} ${SDL_INCLUDE_DIR} ${SDLTTF_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS} ${USB_INCLUDE_DIRS})

  target_link_libraries(kinectboard freenect ${SDLTTF_LIBRARY} ${SDL_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})
endif()

install (TARGETS kinectboard DESTINATION bin)
