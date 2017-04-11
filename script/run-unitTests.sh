#!/bin/bash

SCRIPTS_PATH="$(dirname $(readlink -f ${0}))"
G2O_BIN_DIR=$SCRIPTS_PATH/../bin

STATE=0

for t in $G2O_BIN_DIR/unittest_*; do
  echo ""
  echo "RUNNING $t"
  $t
  if [ $? -ne 0 ]; then
    echo "$t FAILED"
    STATE=1
  fi
done

if [ $STATE -ne 0 ]; then
  echo "Some tests failed"
fi

exit $STATE
