BUILD_CHANNEL?=local

bufinstall:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet

bufsetup:
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@v1.8.0
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp
	pkg-config openssl || export PKG_CONFIG_PATH=$$PKG_CONFIG_PATH:`find \`which brew > /dev/null && brew --prefix\` -name openssl.pc | head -n1 | xargs dirname`

buf: bufsetup
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.grpc.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.gen.yaml buf.build/googleapis/googleapis

clean:
	rm -rf grpc
	rm -rf viam-cartographer/build
	rm -rf viam-cartographer/cartographer/build

format-setup:
	sudo apt-get install -y clang-format

format:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*api*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

setup:
ifeq ("Darwin", "$(shell uname -s)")
	cd viam-cartographer/scripts && ./setup_cartographer_macos.sh
else
	cd viam-cartographer/scripts && ./setup_cartographer_linux.sh
endif
	

build:
ifneq ($(wildcard viam-cartographer/build/.),)
	cd viam-cartographer && ./scripts/build_viam_cartographer.sh 
else 
	cd viam-cartographer && ./scripts/build_cartographer.sh && ./scripts/build_viam_cartographer.sh
endif

install-lua-files:
	sudo mkdir -p /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/lua_files/* /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/cartographer/configuration_files/* /usr/local/share/cartographer/lua_files/

test:
	cd viam-cartographer && ./scripts/test_cartographer.sh

all: bufinstall buf setup build install-lua-files test

appimage: build
	cd packaging/appimages && BUILD_CHANNEL=${BUILD_CHANNEL} appimage-builder --recipe carto_grpc_server-`uname -m`.yml
	cd packaging/appimages && ./package_release_carto.sh
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage


include *.make
