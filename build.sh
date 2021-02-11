#!/bin/bash
mkdir -p build
cd build
cmake ../ -DBUILD_KIT=FALSE
make
