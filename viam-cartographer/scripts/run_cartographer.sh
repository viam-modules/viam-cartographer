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

# ---- Edit based on your needs:
DATA_BASE_DIRECTORY="$HOME/viam/lidar_data"
DATA_DIR="$DATA_BASE_DIRECTORY/data-carto"
# ----

/usr/local/bin/carto_grpc_server  \
    -data_dir=${DATA_DIR}  \
    -config_param="{mode=2D,}"  \
    -port=localhost:8083  \
    -sensors=""  \
    -data_rate_ms=200 \
    -map_rate_sec=60 
