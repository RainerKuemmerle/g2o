#!/bin/sh

set -x

sudo apt-get update -qq || exit 1
sudo apt-get install -qq libqt4-dev libqt4-opengl-dev libqglviewer-qt4-dev libeigen3-dev libsuitesparse-dev || exit 1
