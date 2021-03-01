# PNT Integrity Library
The PNT Integrity Library provides users a method to verify the integrity of the received GPS data and ranging signals, thereby improving resiliency against potential GPS signal loss. The software is a scalable framework for GNSS-based PNT manipulation detection that offers varying levels of protection based on the available data. The library is to be provided to GNSS receiver and GNSS-based timing server OEMs for use in future development or integration into existing products and platforms. More details on the PNT Integrity Library can be found under the [PNT_Integrity_Library_Guide.pdf](https://github.com/cisagov/PNT-Integrity/blob/develop/PNT_Integrity_Library_Guide.pdf) and [PNT ReadMe on system components, algorithms, and checks](https://github.com/cisagov/PNT-Integrity/blob/develop/pnt_integrity/pnt_integrity/README.md).

## License
This library is licensed under the BSD 3-Clause License. The library contains source code developed by IS4S and third parties. Refer to the invidual source files for applicable copyright information.
```
Copyright (c) 2020 Integrated Solutions for Systems (IS4S), Inc
Copyright (c) 2017, ETHZ ASL (geodetic converter)
Copyright (c) 2009, Ben Hoyt (inih)
```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of IS4S, Ben Hoyt, ETHZ ASL, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Build Instructions for the PNT Integrity Library

This repository contains the top level CMake project for building the IS4S PNT Integrity Library, User Interface, and their dependencies.

<b> System Requirements </b>

The PNT Integrity Library is written in a cross-platform manner using C++. While it is expected to work on a wide variety of platforms, it has been tested with on the following operating systems:
* Ubuntu Linux 16.04, 18.04 & 20.04
* MacOS 10.15
* Windows 10

The following additional tools are needed to build the library:
* CMake 3.5 or greater
* C++14 compliant compiler (e.g. Clang 3.3+, GCC 4.7+, MSVC 2015+)

<b> Dependencies </b>

The PNT Integrity Library is designed to require as few third party dependencies as possible to support building on a wide variety of platforms. Two dependencies are required in addition to the libraries provided in the package. The Eigen (https://eigen.tuxfamily.org) C++ template library for linear algebra is required by the base PNT Integrity library. The FFTW (http://fftw.org) package is optionally required and is needed to use the acquisition check. QT (https://www.qt.io) is required to build and run the user interface

Both packages can be installed following instructions on their respective websites. Eigen is a header-only package and can be installed by downloading a release from the project web site and extracting to a local folder. FFTW binaries are available for a range of platforms from the project web site.

Alternatively, a package manager can be used to install the dependencies. For MacOS the Homebrew (https://brew.sh) package manager is recommended. The Chocolatey (https://chocolatey.org) package manager is recommended for Windows. Instructions on installing the required and optional dependencies using package managers on the supported
operating systems are provided in the following sections.

<b>Ubuntu / Debian</b>

Install Eigen by running:
```
sudo apt install libeigen3-dev
```
Optionally install FFTW by running:
```
sudo apt install libfftw3-dev
```
Install Qt5 on Ubuntu by running:
```
sudo apt install qtdeclarative5-dev qtwebengine5-dev libqt5charts5-dev
```

<b>MacOS </b>

Install Eigen by running:
```
brew install eigen
```
Optionally install FFTW by running:
```
brew install fftw
```
Install Qt5 on MacOS by running:
```
brew install qt
```
If you encounter CMake build errors when finding QT this may help:
```
export CMAKE_PREFIX_PATH=/usr/local/Cellar/qt/[version]/
```

<b>Windows</b>

Install Eigen by running:
```
choco install eigen
```
Chocolatey does not support provide binaries for FFTW. They can be downloaded and installed
directly from the project website: http://www.fftw.org/install/windows.html
(http://www.fftw.org/install/windows.html)
For instructions on installing Qt on Windows platform [click here](https://doc.qt.io/qt-5/windows.html).

<b>Building</b>

Extract the release archive:
```
unzip release.zip
```
Generate build files using cmake
```
cd release
mkdir build
cmake ../
```
By default, this will generate Unix Makefiles for the package. Project files can be generated for
other build systems or IDEs by selecting an alternative CMake generator
(https://cmake.org/cmake/help/v3.15/manual/cmake-generators.7.html).
Build the libraries by running:
```
make
```
The libraries can be optionally installed to the user's system by running:
```
make install
```
