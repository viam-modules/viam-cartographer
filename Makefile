BUILD_CHANNEL?=local
TOOL_BIN := $(shell pwd)/bin/tools/$(shell uname -s)-$(shell uname -m)
GIT_REVISION := $(shell git rev-parse HEAD | tr -d '\n')
TAG_VERSION ?= $(shell git tag --points-at | sort -Vr | head -n1)
GO_BUILD_LDFLAGS := -ldflags "-X 'main.Version=${TAG_VERSION}' -X 'main.GitRevision=${GIT_REVISION}'"
SHELL := /usr/bin/env bash
export PATH := $(TOOL_BIN):$(PATH)
export GOBIN := $(TOOL_BIN)

ifneq (, $(shell which brew))
	EXTRA_CMAKE_FLAGS := -DCMAKE_PREFIX_PATH=$(shell brew --prefix) -DQt5_DIR=$(shell brew --prefix qt5)/lib/cmake/Qt5
	export PKG_CONFIG_PATH := $(shell brew --prefix openssl@3)/lib/pkgconfig
endif

default: build

artifact-pull: $(TOOL_BIN)/artifact
	artifact pull

$(TOOL_BIN)/artifact:
	 go install go.viam.com/utils/artifact/cmd/artifact

$(TOOL_BIN)/buf:
	go install github.com/bufbuild/buf/cmd/buf@v1.8.0

$(TOOL_BIN)/protoc-gen-grpc-cpp:
	mkdir -p "$(TOOL_BIN)"
	which grpc_cpp_plugin && ln -sf `which grpc_cpp_plugin` $(TOOL_BIN)/protoc-gen-grpc-cpp

buf: $(TOOL_BIN)/buf $(TOOL_BIN)/protoc-gen-grpc-cpp
	echo ${PATH}
	buf generate --template ./buf/buf.gen.yaml buf.build/viamrobotics/api
	buf generate --template ./buf/buf.grpc.gen.yaml buf.build/viamrobotics/api
	buf generate --template ./buf/buf.gen.yaml buf.build/googleapis/googleapis

clean:
	rm -rf grpc bin viam-cartographer/build

clean-all:
	git clean -fxd
	cd viam-cartographer/cartographer && git checkout . && git clean -fxd

ensure-submodule-initialized:
	@if [ ! -d "viam-cartographer/cartographer/cartographer" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi
	grep -q viam-patched viam-cartographer/cartographer/CMakeLists.txt || \
	(cd viam-cartographer/cartographer && git checkout . && git apply ../cartographer_patches/carto.patch)

lint-cpp:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*api*' \
		-and ! -path '*grpc*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

lint-go: $(TOOL_BIN)/combined $(TOOL_BIN)/golangci-lint $(TOOL_BIN)/actionlint
	go vet -vettool=$(TOOL_BIN)/combined ./...
	GOGC=50 golangci-lint run -v --fix --config=./etc/golangci.yaml
	actionlint

$(TOOL_BIN)/combined $(TOOL_BIN)/golangci-lint $(TOOL_BIN)/actionlint:
	go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/rhysd/actionlint/cmd/actionlint

lint: ensure-submodule-initialized lint-cpp lint-go

setup: install-dependencies ensure-submodule-initialized artifact-pull

install-dependencies:
ifneq (, $(shell which brew))
	brew update
	brew install abseil boost ceres-solver protobuf ninja cairo googletest lua@5.3 pkg-config cmake go@1.20 grpc clang-format
	brew link lua@5.3
	brew install openssl@3 eigen gflags glog suite-sparse sphinx-doc pcl nlopt-static
else ifneq (, $(shell which apt-get))
	$(warning  "Installing cartographer external dependencies via APT.")
	$(warning "Packages may be too old to work with this project.")
	sudo apt-get update
	sudo apt-get install -y cmake ninja-build libgmock-dev libboost-iostreams-dev liblua5.3-dev libcairo2-dev python3-sphinx libnlopt-dev \
		libabsl-dev libceres-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc libpcl-dev libgrpc-dev libgrpc++-dev clang-format
else
	$(error "Unsupported system. Only apt and brew currently supported.")
endif

build: ensure-submodule-initialized buf build-module
	cd viam-cartographer && cmake -Bbuild -G Ninja ${EXTRA_CMAKE_FLAGS} && cmake --build build

build-debug: EXTRA_CMAKE_FLAGS += -DCMAKE_BUILD_TYPE=Debug -DFORCE_DEBUG_BUILD=True
build-debug: build

build-module:
	mkdir -p bin && go build $(GO_BUILD_LDFLAGS) -o bin/cartographer-module module/main.go

test-cpp:
	viam-cartographer/build/unit_tests -p -l all

# Linux only
setup-cpp-full-mod: 
	sudo apt install -y valgrind gdb

# Linux only
test-cpp-full-mod-valgrind: build-debug
	valgrind --error-exitcode=1 --leak-check=full -s viam-cartographer/build/unit_tests -p -l all -t CartoFacade_io -t CartoFacadeCPPAPI

# Linux only
test-cpp-full-mod-gdb: build-debug
	gdb --batch --ex run --ex bt --ex q --args viam-cartographer/build/unit_tests -p -l all -t CartoFacadeCPPAPI

test-cpp-full-mod: build-debug
	viam-cartographer/build/unit_tests -p -l all -t CartoFacadeCPPAPI

test-go:
	go test -race ./...

test: test-cpp test-go

install-lua-files:
	sudo mkdir -p /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/lua_files/* /usr/local/share/cartographer/lua_files/

install: install-lua-files
	sudo rm -f /usr/local/bin/carto_grpc_server
	sudo rm -f /usr/local/bin/cartographer-module
	sudo cp viam-cartographer/build/carto_grpc_server /usr/local/bin/carto_grpc_server
	sudo cp bin/cartographer-module /usr/local/bin/cartographer-module

appimage: build
	cd etc/packaging/appimages && BUILD_CHANNEL=${BUILD_CHANNEL} appimage-builder --recipe cartographer-module-`uname -m`.yml
	mkdir -p etc/packaging/appimages/deploy/
	mv etc/packaging/appimages/*.AppImage* etc/packaging/appimages/deploy/
	chmod 755 etc/packaging/appimages/deploy/*.AppImage

appimage-ci: build
	cd etc/packaging/appimages && ./package_release_module.sh
	mkdir -p etc/packaging/appimages/deploy/
	mv etc/packaging/appimages/*.AppImage* etc/packaging/appimages/deploy/
	chmod 755 etc/packaging/appimages/deploy/*.AppImage
