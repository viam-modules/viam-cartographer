# Cartographer


## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview
This directory contains the C++ code that wraps [Cartographer](https://github.com/cartographer-project/cartographer) into a [SLAM Service](https://github.com/viamrobotics/rdk/blob/c9bbdf1452eea5c3ef5fc33112d26510f664dae0/services/slam/slam.go). 

## Installation instructions
Make sure to follow all steps as outlined in [the setup section here](../../README.md#setup). 

### Automatic Dependency Installation (x64, arm64, or macOS)
To automatically install dependencies, use the target 
```
./setup_cartographer_linux.sh
```
or
```
./setup_cartographer_macos.sh
```
### Manual Dependency Installation (x64, arm64, or macOS)

#### Setup: OSx

**Install Xcode**
1. Install/Update Xcode from App Store
1. Install Xcode cmdline tools: `xcode-select --install`

**Install dependencies**
```bash
# Update and upgrade Brew
brew update
brew upgrade
```

```bash
# Install dependencies
brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3
brew link lua@5.3
brew link protobuf
brew install openssl eigen gflags glog suite-sparse sphinx-doc pcl

```

#### Setup: Raspberry Pi OS Lite (64-bit)

**Install dependencies**
```bash
sudo apt update
# Install dependencies
sudo apt install -y cmake ninja-build libgmock-dev libboost-iostreams-dev liblua5.3-dev libcairo2-dev python3-sphinx libabsl-dev libceres-dev libprotobuf-dev protobuf-compiler libpcl-dev
 ```

 ### Building cartographer
**Build cartographer**

Run: `./scripts/build_cartographer.sh`

Installation & building tested on:
- [X] Raspberry Pi OS Lite (64-bit)
- [X] 2.4 GHz 8-Core Intel Core i9; macOS Monterey
- [X] linux/amd64
- [X] M1

This needs to be built only once.

**Build viam-cartographer**

Run: `./scripts/build_viam_cartographer.sh`

Installation & building tested on:
- [X] Raspberry Pi OS Lite (64-bit)
- [X] 2.4 GHz 8-Core Intel Core i9; macOS Monterey
- [X] linux/amd64
- [X] M1

This will build the binary and save it at `./build/carto_grpc_server`. Move this binary into `usr/local/bin` by running:
```bash
sudo cp build/carto_grpc_server /usr/local/bin/
```

In your desired data directory, move the configuration files from cartographer into `/usr/local/share`:  
```bash
sudo mkdir -p /usr/local/share/cartographer/lua_files
sudo cp lua_files/* /usr/local/share/cartographer/lua_files/
sudo cp cartographer/configuration_files/* /usr/local/share/cartographer/lua_files/
```
You only have to do this once per data directory. Note cartographer will fail if the configuration files cannot be found.

To run tests, after building, run the following
```bash
./scripts/test_cartographer.sh
```
