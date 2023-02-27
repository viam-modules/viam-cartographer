cmake_minimum_required(VERSION 3.2)

project(viam-cartographer)

set(VIAM_CARTOGRAPHER_MAJOR_VERSION 1)
set(VIAM_CARTOGRAPHER_MINOR_VERSION 0)
set(VIAM_CARTOGRAPHER_PATCH_VERSION 0)
set(VIAM_CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(VIAM_CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")

find_package(absl REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Lua 5.2 REQUIRED)

# include files/library from cartographer submodule
include_directories(${PROJECT_SOURCE_DIR}/cartographer)
include_directories(${PROJECT_SOURCE_DIR}/cartographer/build)
link_libraries(${PROJECT_SOURCE_DIR}/cartographer/build/libcartographer.a)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenSSL REQUIRED)

if(WIN32)
  # On Windows, Protobuf is incorrectly found by the bundled CMake module, so prefer native CMake config.
  set(protobuf_MODULE_COMPATIBLE TRUE CACHE INTERNAL "")
  find_package(Protobuf 3.0.0 CONFIG)
else()
  find_package(Protobuf 3.4.0 REQUIRED)
endif()

include(FindPkgConfig)
if (NOT WIN32)
  PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
else()
  find_library(CAIRO_LIBRARIES cairo)
endif()

file(GLOB_RECURSE ALL_VIAM_HDRS "src/*.h")
file(GLOB_RECURSE ALL_VIAM_SRCS "src/*.cc")
file(GLOB_RECURSE ALL_VIAM_TEST_SRCS "src/*_test.cc")
list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_VIAM_TEST_SRCS})


if(DEFINED ENV{API_SOURCE_DIR})
  set(API_SOURCE_DIR $ENV{API_SOURCE_DIR})
else()
  set(API_SOURCE_DIR ${PROJECT_SOURCE_DIR}/..)
endif()
set(SRCDIR ${API_SOURCE_DIR}/grpc/cpp/gen)

message("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
message("API SOURCE DIR: " ${API_SOURCE_DIR})
message("GRPC SOURCE DIR: " ${SRCDIR})

if(EXISTS ${SRCDIR})
  find_package(PkgConfig REQUIRED)

  pkg_check_modules(GRPC REQUIRED grpc)
  pkg_check_modules(GRPCPP REQUIRED grpc++)

  set(SOURCES ${SRCDIR}/service/slam/v1/slam.grpc.pb.cc
              ${SRCDIR}/service/slam/v1/slam.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/common/v1/common.grpc.pb.cc
              ${SRCDIR}/common/v1/common.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/annotations.pb.cc
              ${SRCDIR}/google/api/httpbody.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/http.pb.cc)


  # Remove dotfiles/-folders that could potentially pollute the build.
  file(GLOB_RECURSE ALL_DOTFILES ".*/*")
  if (ALL_DOTFILES)
    list(REMOVE_ITEM ALL_VIAM_HDRS ${ALL_DOTFILES})
    list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_DOTFILES})
  endif()

  list(APPEND ALL_LIBRARY_HDRS ${ALL_VIAM_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_VIAM_SRCS})

  add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS} ${SOURCES})
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${SRCDIR})

  configure_file(
    ${PROJECT_SOURCE_DIR}/cartographer/cartographer/common/config.h.cmake
    ${PROJECT_BINARY_DIR}/common/config.h)

  message("PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR: ${PROJECT_SOURCE_DIR}, ${PROJECT_BINARY_DIR}")

  google_binary(carto_grpc_server
    SRCS
      src/main.cc
  )

  google_binary(unit_tests
    SRCS
      ${ALL_VIAM_TEST_SRCS}
  )

  target_link_libraries(carto_grpc_server PUBLIC
    ${GRPC_LIBRARIES}
    ${GRPCPP_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${JPEG_LIBRARIES})
  target_include_directories(carto_grpc_server SYSTEM PUBLIC
    ${SRCDIR}
    ${GRPC_INCLUDE_DIRS}
    ${GRPCPP_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIRS})
  target_compile_options(carto_grpc_server PUBLIC
    ${GRPC_CFLAGS_OTHER}
    ${GRPCPP_CFLAGS_OTHER}
    ${PROTOBUF_CFLAGS_OTHER}
    ${JPEG_CFLAGS_OTHER})

  target_link_libraries(unit_tests PUBLIC
    ${GRPC_LIBRARIES}
    ${GRPCPP_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${JPEG_LIBRARIES})
  target_include_directories(unit_tests SYSTEM PUBLIC
    ${SRCDIR}
    ${GRPC_INCLUDE_DIRS}
    ${GRPCPP_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIRS})
  target_compile_options(unit_tests PUBLIC
    ${GRPC_CFLAGS_OTHER}
    ${GRPCPP_CFLAGS_OTHER}
    ${PROTOBUF_CFLAGS_OTHER}
    ${JPEG_CFLAGS_OTHER})

  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${CERES_INCLUDE_DIRS}")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${LUA_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

  if (WIN32)
    find_package(glog REQUIRED)
    set(GLOG_LIBRARY glog::glog)
  else()
    set(GLOG_LIBRARY glog)
  endif()

  target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
  target_link_libraries(${PROJECT_NAME} PUBLIC gflags)
  if(WIN32)
    # Needed to fix conflict with MSVC's error macro.
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DGLOG_NO_ABBREVIATED_SEVERITIES)
  endif()
  if(MSVC)
    # Needed for VS 2017 5.8
    target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE -D_USE_MATH_DEFINES)
  endif()

  if("${CAIRO_INCLUDE_DIRS}")
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
      "${CAIRO_INCLUDE_DIRS}")
  endif()

  target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

  target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} 
    absl::algorithm
    absl::base
    absl::debugging
    absl::flat_hash_map
    absl::memory
    absl::meta
    absl::numeric
    absl::str_format
    absl::strings
    absl::synchronization
    absl::time
    absl::utility 
  )
  if (NOT WIN32)
    target_link_libraries(${PROJECT_NAME} PUBLIC pthread)
  endif()

  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

  # Add the binary directory first, so that port.h is included after it has
  # been generated.
  target_include_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
      $<INSTALL_INTERFACE:include>
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT CartographerExport
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  foreach(HDR ${INSTALL_SOURCE_HDRS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    install(
      FILES ${HDR}
      DESTINATION include/${DIR}
    )
  endforeach()

  foreach(HDR ${INSTALL_GENERATED_HDRS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    install(
      FILES ${HDR}
      DESTINATION include/${DIR}
    )
  endforeach()
else()
message(
  FATAL_ERROR
    "RDK C++ gRPC gen files not found. Please specify the absolute path to your API_SOURCE_DIR e.g. 'export API_SOURCE_DIR=/home/user/api'"
)
endif()  
