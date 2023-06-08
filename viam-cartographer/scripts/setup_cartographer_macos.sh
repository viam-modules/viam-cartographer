#!/bin/bash
echo "Installing cartographer external dependencies"
brew update
brew install abseil boost ceres-solver protobuf@21 ninja cairo googletest lua@5.3 pkg-config cmake go@1.20 grpc 
brew link lua@5.3
brew install openssl@3 eigen gflags glog suite-sparse sphinx-doc pcl
brew unlink protobuf@3
brew unlink protobuf
brew unlink protobuf@21 && brew link protobuf@21
