#!/bin/bash

SCRIPTS_PATH="$(dirname $(readlink -f ${0}))"
G2O_BIN_DIR=$SCRIPTS_PATH/../bin

STATE=0

for t in $G2O_BIN_DIR/unittest_*; do
  echo $t
  $t
  STATE=$((STATE || $?))
done

if [[ $STATE ]]; then
  echo "Some tests failed"
fi

exit $STATE
