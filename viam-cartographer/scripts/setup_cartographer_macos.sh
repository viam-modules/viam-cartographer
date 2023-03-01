#!/bin/bash
echo "Installing cartographer external dependencies"
brew update
brew upgrade
brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3
brew link lua@5.3
brew install openssl eigen gflags glog suite-sparse sphinx-doc pcl
brew link protobuf
brew link openssl
echo 'export PKG_CONFIG_PATH="/usr/local/opt/openssl@3/lib/pkgconfig"' >> ~/.viamdevrc

echo -e "\033[0;32m""Dev environment setup is complete!""\033[0m"
echo -e "Don't forget to restart your shell, or execute: ""\033[41m""source ~/.viamdevrc""\033[0m"
exit 0