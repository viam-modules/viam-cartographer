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

echo "Running tests..."
./build/unit_tests
