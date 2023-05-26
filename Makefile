BUILD_CHANNEL?=local
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"
GIT_REVISION = $(shell git rev-parse HEAD | tr -d '\n')
TAG_VERSION?=$(shell git tag --points-at | sort -Vr | head -n1)
GO_BUILD_LDFLAGS = -ldflags "-X 'main.Version=${TAG_VERSION}' -X 'main.GitRevision=${GIT_REVISION}'"

bufinstall:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet

bufsetup:
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@v1.8.0
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp

buf: bufsetup
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.grpc.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.gen.yaml buf.build/googleapis/googleapis

clean:
	rm -rf grpc
	rm -rf bin
	rm -rf viam-cartographer/build
	rm -rf viam-cartographer/cartographer/build

clean-all:
	git clean -fxd

ensure-submodule-initialized:
	@if [ ! -d "viam-cartographer/cartographer/cartographer" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi

lint-setup-cpp:
ifeq ("Darwin", "$(shell uname -s)")
	brew install clang-format
else
	sudo apt-get install -y clang-format
endif

lint-setup-go:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
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
	go vet -vettool=$(TOOL_BIN)/combined ./...
	GOGC=50 $(TOOL_BIN)/golangci-lint run -v --fix --config=./etc/golangci.yaml
	PATH=$(PATH_WITH_TOOLS) actionlint

lint: ensure-submodule-initialized lint-cpp lint-go

setup: ensure-submodule-initialized
ifeq ("Darwin", "$(shell uname -s)")
	cd viam-cartographer/scripts && ./setup_cartographer_macos.sh
else
	cd viam-cartographer/scripts && ./setup_cartographer_linux.sh
endif
	
build: build-module
ifneq ($(wildcard viam-cartographer/cartographer/build/.),)
	cd viam-cartographer && ./scripts/build_viam_cartographer.sh 
else 
	cd viam-cartographer && ./scripts/build_cartographer.sh && ./scripts/build_viam_cartographer.sh
endif

build-module:
	mkdir -p bin && go build $(GO_BUILD_LDFLAGS) -o bin/cartographer-module module/main.go

install-lua-files:
	sudo mkdir -p /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/lua_files/* /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/cartographer/configuration_files/* /usr/local/share/cartographer/lua_files/

test-cpp:
	cd viam-cartographer && ./scripts/test_cartographer.sh

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
