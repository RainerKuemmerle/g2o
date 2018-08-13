#!/bin/sh

set -ev

sudo apt-get update -qq
sudo apt-get install -qq qtdeclarative5-dev qt5-qmake libqglviewer-dev libsuitesparse-dev

# download eigen3, unpack and install it. The installation is required to install the cmake files
cd /tmp
wget -O eigen3.zip http://bitbucket.org/eigen/eigen/get/3.3.4.zip
unzip -q eigen3.zip
ls -l eigen*
mkdir eigen-eigen-5a0156e40feb-build
cd eigen-eigen-5a0156e40feb-build
cmake ../eigen-eigen-5a0156e40feb -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/usr/include
sudo make install
cd ..
