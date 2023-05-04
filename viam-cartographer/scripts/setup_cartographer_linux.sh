#!/bin/bash
echo "Installing cartographer external dependencies"
sudo apt update
sudo apt install -y cmake ninja-build libgmock-dev libboost-iostreams-dev liblua5.3-dev libcairo2-dev python3-sphinx libabsl-dev libceres-dev libprotobuf-dev protobuf-compiler libpcl-dev protobuf-compiler-grpc libgrpc-dev libgrpc++-dev
