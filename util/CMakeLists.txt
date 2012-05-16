# Kinectboard utility library
find_package(SDL REQUIRED)
find_package(SDL_ttf REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

include_directories(../common ${SDL_INCLUDE_DIR} ${SDLTTF_INCLUDE_DIR})

add_library(kinectboard_util kinectboard_controls.c kinectboard_images.c)
target_link_libraries(kinectboard_util ${SDLTTF_LIBRARY} ${SDL_LIBRARY})

