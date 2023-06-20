cmake_minimum_required(VERSION 3.2)

project(viam-cartographer)
set (CMAKE_CXX_STANDARD 17)

if(DEFINED ENV{API_SOURCE_DIR})
  set(API_SOURCE_DIR $ENV{API_SOURCE_DIR})
else()
  set(API_SOURCE_DIR ${PROJECT_SOURCE_DIR}/..)
endif()
set(GRPC_SRC_DIR ${API_SOURCE_DIR}/grpc/cpp/gen)

if(NOT EXISTS ${GRPC_SRC_DIR})
message(FATAL_ERROR
  "RDK C++ gRPC gen files not found. Please specify the absolute path to your API_SOURCE_DIR e.g. 'export API_SOURCE_DIR=/home/user/api'"
)
endif()

message("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
message("API SOURCE DIR: " ${API_SOURCE_DIR})
message("GRPC SOURCE DIR: " ${GRPC_SRC_DIR})


# dependencies
add_subdirectory(cartographer)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPCPP REQUIRED grpc++)

find_package(absl REQUIRED)
add_definitions(-DABSL_LEGACY_THREAD_ANNOTATIONS)

find_package(PCL REQUIRED)


# build sources list
file(GLOB_RECURSE ALL_VIAM_HDRS "src/*.h")
file(GLOB_RECURSE ALL_VIAM_SRCS "src/*.cc")
file(GLOB_RECURSE ALL_VIAM_C_SRCS "src/*.c")
file(GLOB_RECURSE ALL_VIAM_TEST_SRCS "src/*_test.cc")
list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_VIAM_TEST_SRCS})

# remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_VIAM_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_DOTFILES})
endif()

# combine sources
list(APPEND ALL_LIBRARY_HDRS ${ALL_VIAM_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_VIAM_SRCS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_VIAM_C_SRCS})

# add grpc/API sources
set(GRPC_SRCS
  ${GRPC_SRC_DIR}/service/slam/v1/slam.grpc.pb.cc
  ${GRPC_SRC_DIR}/service/slam/v1/slam.pb.cc
  ${GRPC_SRC_DIR}/common/v1/common.grpc.pb.cc
  ${GRPC_SRC_DIR}/common/v1/common.pb.cc
  ${GRPC_SRC_DIR}/google/api/annotations.pb.cc
  ${GRPC_SRC_DIR}/google/api/httpbody.pb.cc
  ${GRPC_SRC_DIR}/google/api/http.pb.cc
)

# the viam-cartographer library
add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS} ${GRPC_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${GRPC_SRC_DIR}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  cartographer
  ${GRPCPP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# should be included from protobuf, but isn't in newer versions, so workaround here
if (TARGET absl::log_internal_check_op)
  target_link_libraries(${PROJECT_NAME} PUBLIC absl::log_internal_check_op)
endif()

# executables
add_executable(carto_grpc_server src/main.cc)
target_link_libraries(carto_grpc_server PUBLIC viam-cartographer)

add_executable(unit_tests ${ALL_VIAM_TEST_SRCS})
target_link_libraries(unit_tests PUBLIC viam-cartographer)
