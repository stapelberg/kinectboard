kinectboard
===========

An awesome kinect board.

Required Librarys
-----------------

* libfreenect https://github.com/OpenKinect/libfreenect
* SDL 1.2
* SDLTTF
* OpenGL 3
* Glut 3

Install libfreenect
-------------------
If you have ibfreenect already installed, please check the modules path.

1. Download libfrenect
2. Extract libfreenect
3. Switch into libfreenect directory
4. cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr CMakeLists.txt 
5. make
6. make install

Build with cmake
----------------

1. cd pathToKinectboard
2. cmake .
3. make
4. make install

