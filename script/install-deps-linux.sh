#!/bin/sh

set -ev

# source for new gcc
#sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test

sudo apt-get update -qq

# dependencies for building g2o
sudo apt-get install -qq qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libsuitesparse-dev

# install new gcc
#if [ "$CC" = "gcc" ]; then
  #apt-cache search -n "^gcc-[0-9]+$"
  #sudo apt-get install -qq gcc-8 g++-8
#fi

# download eigen3 and unpack it
cd /tmp
wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.zip
unzip -q eigen3.zip
ls -l eigen*
sudo mv /tmp/eigen-3.3.8 /usr/include/eigen3
