kinectboard
===========

An awesome kinect board.

Required Librarys
-----------------

* pthreads
* libfreenect https://github.com/OpenKinect/libfreenect
* Nvidia CUDA SDK >= 4.2
* OpenGL 3
* Glut 3
* SDL 1.2

On Debian, try:

    apt-get install build-essential cmake libsdl1.2-dev freeglut3-dev libglew-dev

Install CUDA
-------------------

Download and install the CUDA SDK >= 4.2 and the NVidia GPU Computing SDK >= 4.2 for your platform from
http://www.nvidia.com/content/cuda/cuda-downloads.html

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
    
The binaries will be placed in either bin (64 Bit) or bin32 (32 Bit legacy support) depending on your platform.
To start kinectboard you either need to add the corresponding bin folder to your LD_LIBRARY_PATH or run
    ./kinectboard.sh 
in the bin folders.
