#!/bin/bash

set -ev

brew update

if brew outdated | grep -qx cmake; then
  brew upgrade cmake;
fi

brew install eigen
