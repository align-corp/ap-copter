#!/bin/bash
# build for all copters boards

set -e

# specify boards
BOARDS="AP3-M460 AP3-M460-dev AP3-M460-G3P"

# prepare align-build folder
if [ -d "align-build" ]; then
    rm -rf align-build
fi
mkdir align-build

# compile
for b in $BOARDS; do
    echo "Testing $b build"
    ./waf configure --board $b
    ./waf clean
    ./waf copter
    cp build/$b/bin/arducopter.apj align-build/$b.apj
done

echo "align builds completed"

exit 0
