# g2o - General Graph Optimization

Linux: [![Build Status](https://travis-ci.org/RainerKuemmerle/g2o.svg?branch=master)](https://travis-ci.org/RainerKuemmerle/g2o)
Windows: [![Build status](https://ci.appveyor.com/api/projects/status/9w0cpb9krc6t4nt7/branch/master?svg=true)](https://ci.appveyor.com/project/RainerKuemmerle/g2o/branch/master)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/e87df92948b747d58591372dd425fc59)](https://app.codacy.com/manual/rainer.kuemmerle/g2o?utm_source=github.com&utm_medium=referral&utm_content=RainerKuemmerle/g2o&utm_campaign=Badge_Grade_Dashboard)

g2o is an open-source C++ framework for optimizing graph-based nonlinear error
functions. g2o has been designed to be easily extensible to a wide range of
problems and a new problem typically can be specified in a few lines of code.
The current implementation provides solutions to several variants of SLAM and
BA.

A wide range of problems in robotics as well as in computer-vision involve the
minimization of a non-linear error function that can be represented as a graph.
Typical instances are simultaneous localization and mapping (SLAM) or bundle
adjustment (BA). The overall goal in these problems is to find the
configuration of parameters or state variables that maximally explain a set of
measurements affected by Gaussian noise. g2o is an open-source C++ framework
for such nonlinear least squares problems. g2o has been designed to be easily
extensible to a wide range of problems and a new problem typically can be
specified in a few lines of code. The current implementation provides solutions
to several variants of SLAM and BA. g2o offers a performance comparable to
implementations of state-of-the-art approaches for the specific problems
(02/2011).

## Papers Describing the Approach

Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat,
Kurt Konolige, and Wolfram Burgard
[g2o: A General Framework for Graph Optimization](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf)
IEEE International Conference on Robotics and Automation (ICRA), 2011

## Documentation

A detailed description of how the library is structured and how to use and extend it can be found in /doc/g2o.pdf
The API documentation can be generated as described in doc/doxygen/readme.txt

## License

g2o is licensed under the BSD License. However, some libraries are available
under different license terms. See below.

The following parts are licensed under LGPL3+:

-   csparse_extension

The following parts are licensed under GPL3+:

-   g2o_viewer
-   g2o_incremental
-   slam2d_g2o (example for 2D SLAM with a QGLviewer GUI)

Please note that some features of CHOLMOD (which may be used by g2o, see
libsuitesparse below) are licensed under the GPL. To avoid the GPL, you may
have to re-compile CHOLMOD without including its GPL features. The CHOLMOD
library distributed with, for example, Ubuntu or Debian includes the GPL
features. The supernodal factorization is considered by g2o, if it is
available.

Within the folder EXTERNAL we include software not written by us to
guarantee easy compilation.

-   ceres: BSD (see EXTERNAL/ceres/LICENSE)
    Headers to perform Automatic Differentiation

-   freeglut: X Consortium (Copyright (c) 1999-2000 Pawel W. Olszta)
    We use a stripped down version for drawing text in OpenGL.

See the doc folder for the full text of the licenses.

g2o is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
licenses for more details.

## Requirements

-   cmake             <http://www.cmake.org>
-   Eigen3            <http://eigen.tuxfamily.org>

On Ubuntu / Debian these dependencies are resolved by installing the
following packages.

-   cmake
-   libeigen3-dev

### Optional requirements

-   suitesparse       <http://faculty.cse.tamu.edu/davis/suitesparse.html>
-   Qt5               <http://qt-project.org>
-   libQGLViewer      <http://www.libqglviewer.com>

On Ubuntu / Debian these dependencies are resolved by installing the
following packages.

-   libsuitesparse-dev
-   qtdeclarative5-dev
-   qt5-qmake
-   libqglviewer-dev-qt5

## Mac OS X

If using [Homebrew](http://brew.sh/), then

`brew install brewsci/science/g2o`

will install g2o together with its required dependencies. In this case no manual compilation is necessary.

## Windows

If using [vcpkg](https://github.com/Microsoft/vcpkg), then

`scripts\install-deps-windows.bat`

will build and install the required dependencies. The location of `vcpkg` and required triplet are determined by the environment variables `VCPKG_ROOT_DIR` and `VCPKG_DEFAULT_TRIPLET`.

## Compilation

Our primary development platform is Linux. Experimental support for
Mac OS X, Android and Windows (MinGW or MSVC).
We recommend a so-called out of source build which can be achieved
by the following command sequence.

-   `mkdir build`
-   `cd build`
-   `cmake ../`
-   `make`

The binaries will be placed in bin and the libraries in lib which
are both located in the top-level folder.

On Windows with `vcpkg` the following two commands will generate build scripts for Visual Studio 2017 MSVC 14 tool set:

-   `mkdir build`
-   `cd build`
-   `cmake -G "Visual Studio 14 2017 Win64" -DG2O_BUILD_APPS=ON -DG2O_BUILD_EXAMPLES=ON -DVCPKG_TARGET_TRIPLET="%VCPKG_DEFAULT_TRIPLET%" -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT_DIR%\scripts\buildsystems\vcpkg.cmake" ..`

If you are compiling on Windows and you are for some reasons **not** using `vcpkg` please download Eigen3 and extract it.
Within cmake-gui set the variable G2O_EIGEN3_INCLUDE to that directory.

## Cross-Compiling for Android

-   `mkdir build`
-   `cd build`
-   `cmake -DCMAKE_TOOLCHAIN_FILE=../script/android.toolchain.cmake -DANDROID_NDK=<YOUR_PATH_TO_ANDROID_NDK_r10d+> -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON" -DEIGEN3_INCLUDE_DIR="<YOUR_PATH_TO_EIGEN>" -DEIGEN3_VERSION_OK=ON .. && cmake --build .`

## Acknowledgments

We thank the following contributors for providing patches:

-   Simon J. Julier: patches to achieve compatibility with Mac OS X and others.
-   Michael A. Eriksen for submitting patches to compile with MSVC.
-   Mark Pupilli for submitting patches to compile with MSVC.

## Projects using g2o

-   [g2opy](https://github.com/uoip/g2opy): Python binding
-   [.Net wrapper](https://github.com/fugro/g2o)

## Contact information

-   [Rainer Kuemmerle](mailto:kuemmerl@informatik.uni-freiburg.de)
-   [Giorgio Grisetti](mailto:grisetti@dis.uniroma1.it)
-   [Hauke Strasdat](mailto:strasdat@gmail.com)
-   [Kurt Konolige](mailto:konolige@willowgarage.com)
-   [Wolfram Burgard](mailto:burgard@informatik.uni-freiburg.de)
