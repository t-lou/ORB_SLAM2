#!/bin/bash

set -e

REPO=$(dirname $(realpath $0))

echo "Configuring and building Thirdparty/DBoW2 ..."

cd $REPO/Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building Thirdparty/g2o ..."

cd $REPO/Thirdparty/g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Uncompress vocabulary ..."

cd $REPO/Vocabulary
tar -xf ORBvoc.txt.tar.gz

echo "Configuring and building ORB_SLAM2 ..."

cd $REPO
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
