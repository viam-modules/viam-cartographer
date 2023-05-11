#!/bin/bash
which brew && export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:`brew --prefix openssl@3`/lib/pkgconfig"
echo "Installing cartographer external dependencies"
brew update
brew upgrade
brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3 pkg-config cmake go@1.20 grpc
brew link lua@5.3
brew install openssl eigen gflags glog suite-sparse sphinx-doc pcl
brew link protobuf
