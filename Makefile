BUILD_CHANNEL?=local
TOOL_BIN := $(shell pwd)/bin/tools/$(shell uname -s)-$(shell uname -m)
GIT_REVISION := $(shell git rev-parse HEAD | tr -d '\n')
TAG_VERSION ?= $(shell git tag --points-at | sort -Vr | head -n1)
GO_BUILD_LDFLAGS := -ldflags "-X 'main.Version=${TAG_VERSION}' -X 'main.GitRevision=${GIT_REVISION}'"
SHELL := /bin/env bash
export PATH := $(TOOL_BIN):$(PATH)
export GOBIN := $(TOOL_BIN)

ARTIFACT="~/go/bin/artifact"

ifneq (, $(shell which brew))
	EXTRA_CMAKE_FLAGS := -DCMAKE_PREFIX_PATH=$(shell brew --prefix) -DQt5_DIR=$(shell brew --prefix qt5)/lib/cmake/Qt5
	export PKG_CONFIG_PATH := $(shell brew --prefix openssl@3)/lib/pkgconfig
endif

default: build

artifact-pull: $(TOOL_BIN)/artifact
	artifact pull

$(TOOL_BIN)/artifact:
	 go install go.viam.com/utils/artifact/cmd/artifact

bufinstall:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet

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

ensure-submodule-initialized:
	@if [ ! -d "viam-cartographer/cartographer/cartographer" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi
	cd viam-cartographer/cartographer && git checkout . && git apply ../cartographer_patches/carto.patch

lint-setup-cpp:
ifeq ("Darwin", "$(shell uname -s)")
	brew install clang-format
else
	sudo apt-get install -y clang-format
endif

lint-setup-go:
	go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/rhysd/actionlint/cmd/actionlint

lint-setup: lint-setup-cpp lint-setup-go

lint-cpp:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*api*' \
		-and ! -path '*grpc*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

lint-go:
	go vet -vettool=combined ./...
	GOGC=50 golangci-lint run -v --fix --config=./etc/golangci.yaml
	actionlint

lint: ensure-submodule-initialized lint-cpp lint-go

setup: ensure-submodule-initialized artifact-pull
	viam-cartographer/scripts/setup_cartographer.sh

build: ensure-submodule-initialized buf build-module
	cd viam-cartographer && cmake -Bbuild -G Ninja ${EXTRA_CMAKE_FLAGS} && cmake --build build

build-debug: EXTRA_CMAKE_FLAGS+=" -DCMAKE_BUILD_TYPE=Debug"
build-debug: build

build-module:
	mkdir -p bin && go build $(GO_BUILD_LDFLAGS) -o bin/cartographer-module module/main.go

install-lua-files:
	sudo mkdir -p /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/lua_files/* /usr/local/share/cartographer/lua_files/

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

install:
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
