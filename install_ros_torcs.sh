#!/usr/bin/env bash

sudo apt-get install libglib2.0-dev  libgl1-mesa-dev libglu1-mesa-dev  freeglut3-dev  libplib-dev  libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev  libxrandr-dev libpng-dev
git clone https://github.com/fmirus/torcs-1.3.7

cd torcs-1.3.7
git clone https://github.com/barisdemirdelen/scr-torcs-1.3.7
mv  scr-torcs-1.3.7/scr-patch/ ./scr-patch
rm -r scr-torcs-1.3.7
cd scr-patch
./do_patch.sh
cd ..
export CFLAGS="-fPIC"
export CPPFLAGS=$CFLAGS
export CXXFLAGS=$CFLAGS
./configure
make
sudo make install
make datainstall
