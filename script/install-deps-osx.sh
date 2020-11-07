#!/bin/bash

set -ev

brew update

if brew outdated | grep -qx cmake; then
  brew upgrade cmake;
fi

if brew outdated | grep -qx egen; then
  brew upgrade eigen
else
  brew install eigen
fi
