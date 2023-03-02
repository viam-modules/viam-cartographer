#!/bin/bash
set -o errexit

if [ "$(uname)" == "Linux" ]; then
    cd $(dirname $(readlink -f "${BASH_SOURCE}"))/..
elif [ "$(uname)" == "Darwin" ]; then
    readlinkorreal() { readlink "$1" || echo "$1"; }
    cd $(dirname string readlinkorreal "${BASH_SOURCE}")/..
else
    echo ERROR: your OS is not handled yet
    exit 1
fi

# Build and install Cartographer with the viam wrapper.
DIR=`pwd`/build/
echo $DIR
if [ ! -d "$DIR" ]; then
    mkdir build
else 
    echo "Build Directory already exists"
fi

pushd build

echo building viam-cartographer
cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix` -DQt5_DIR=$(brew --prefix qt5)/lib/cmake/Qt5
ninja
popd
