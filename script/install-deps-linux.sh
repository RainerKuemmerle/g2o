#!/bin/sh

set -ev

sudo apt-get update -qq
sudo apt-get install -qq qtdeclarative5-dev qt5-qmake libqglviewer-dev libeigen3-dev libsuitesparse-dev
