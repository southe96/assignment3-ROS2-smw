cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
  src/camera_device.cpp
  src/camera_grab.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- Hikrobot MVS SDK ----
# 优先用环境变量 MVCAM_SDK_PATH，没有就用默认的 /opt/MVS
if(DEFINED ENV{MVCAM_SDK_PATH})
  set(MVS_ROOT "$ENV{MVCAM_SDK_PATH}")
else()
  set(MVS_ROOT "/opt/MVS")
endif()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
)
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS "${MVS_ROOT}/lib/64"
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot MVS SDK not found under '${MVS_ROOT}'.\n"
    "  MvCameraControl.h   : ${MVS_INCLUDE_DIR}\n"
    "  libMvCameraControl  : ${MVS_LIBRARY}\n"
    "Install MVS, or set MVCAM_SDK_PATH to the SDK root (e.g. /opt/MVS).")
endif()
get_filename_component(MVS_LIBRARY_DIR "${MVS_LIBRARY}" DIRECTORY)
message(STATUS "Found MVS SDK: ${MVS_LIBRARY}")

# SYSTEM：SDK 头文件里的警告不算到我们头上
target_include_directories(camera_node SYSTEM PRIVATE "${MVS_INCLUDE_DIR}")
target_link_libraries(camera_node "${MVS_LIBRARY}")

# 安装后的可执行文件带上 SDK 库目录，不设 LD_LIBRARY_PATH 也能找到 .so
set_target_properties(camera_node PROPERTIES
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
)

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
