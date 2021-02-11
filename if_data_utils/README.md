# IF Data Utilities Package

This package contains a set of data structure definitions and utilities for loading and transmitting intermediate frequency (IF) data files.  

## Dependencies

The [Eigen](http://eigen.tuxfamily.org) C++ template library for linear algebra must be installed prior to building the if_data_utils package. 

On Ubuntu/Debian:

    sudo apt install libeigen3-dev

On MacOS:

    brew install eigen

On Windows:

Download and extract an Eigen release or install using a package manager such as [Chocolatey](https://chocolatey.org).
    
## Building

The if_data_utils package can be built using CMake directly or using a build tool such as [Colcon](https://colcon.readthedocs.io/).  To build using CMake:

	mkdir build
	cd build
	cmake ../
	make
	
The library can then be installed to the system by running:

	sudo make install