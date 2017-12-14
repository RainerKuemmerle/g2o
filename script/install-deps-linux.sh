#!/bin/sh

set -ev

sudo apt-get update -qq
sudo apt-get install -qq qtdeclarative5-dev qt5-qmake libqglviewer-dev libsuitesparse-dev

# download eigen3 and unpack it
cd /tmp
wget -O eigen3.zip http://bitbucket.org/eigen/eigen/get/3.3.4.zip
unzip -q eigen3.zip
ls -l eigen*
sudo mv /tmp/eigen-eigen-5a0156e40feb /usr/include/eigen3
