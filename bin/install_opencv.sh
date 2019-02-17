#!/bin/bash


echo "Installing reqiured packages for OpenCV..."
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

if [ ! -e opencv ]; then
	echo "Cloning OpenCV..."
	git clone https://github.com/opencv/opencv.git
fi
echo "Installing..."
cd opencv
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
