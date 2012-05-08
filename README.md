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
* TTF Bitstream-Vera

On Debian, try:

    apt-get install build-essential cmake libsdl1.2-dev libsdl-ttf2.0-dev

Install libfreenect
-------------------

Install the latest git version of libfreenect. We used version
`efd073eacfc54a7c028729596858d1618e134b16`:

    git clone git://github.com/OpenKinect/libfreenect.git
    cd libfreenect
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr CMakeLists.txt
    make
    sudo make install

Build with cmake
----------------

Afterwards, build kinectboard:

    git clone git://github.com/mstap/kinectboard.git
    cd kinectboard
    cmake .
    make
