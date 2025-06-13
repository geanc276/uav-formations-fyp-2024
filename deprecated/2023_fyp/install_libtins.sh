#!/bin/bash

git clone https://github.com/mfontanini/libtins.git

sudo apt-get install libpcap-dev libssl-dev cmake

cd libtins

mkdir build

cd build

cmake ../

make

sudo make install

ldconfig