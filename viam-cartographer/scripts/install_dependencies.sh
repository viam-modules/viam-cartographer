#!/bin/bash

if which brew; then
	echo "Installing cartographer external dependencies"
	brew update
	brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3 pkg-config cmake go@1.20 grpc
	brew link lua@5.3
	brew install openssl@3 eigen gflags glog suite-sparse sphinx-doc pcl
elif which apt; then
	echo "WARNING"
	echo "Installing cartographer external dependencies via APT."
	echo "Packages may be too old to work with this project."
	sudo apt update
	sudo apt install -y cmake ninja-build libgmock-dev libboost-iostreams-dev liblua5.3-dev libcairo2-dev python3-sphinx libabsl-dev libceres-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc libpcl-dev
else
	echo "Unsupported system. Only apt and brew currently supported."
	exit 1
fi
