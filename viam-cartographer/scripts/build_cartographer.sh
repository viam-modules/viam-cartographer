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
pushd cartographer
cp ../cartographer_build_utils/CMakeLists.txt CMakeLists.txt
rm -rf build
mkdir build
pushd build

echo building cartographer
cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix`
ninja
popd
popd
