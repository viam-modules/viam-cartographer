echo "Installing cartographer external dependencies"
brew update
brew upgrade
brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3
brew link lua@5.3
brew install openssl eigen gflags glog suite-sparse sphinx-doc pcl
brew link protobuf
brew link openssl
