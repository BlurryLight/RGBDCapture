#!/bin/bash 

# update all submodules
git submodule update --init --recursive
# build OpenNI2
cd ./OpenNI2/
make -j4
cd -

# build libfreenect
cd ./libfreenect
mkdir -p build
cd build
cmake .. -DBUILD_OPENNI2_DRIVER=ON
make -j4
cp -L lib/OpenNI2-FreenectDriver/libFreenectDriver.so ../../OpenNI2/Bin/x64-Release/OpenNI2/Drivers/
cd ../../
