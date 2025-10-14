cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Torch_DIR "/opt/libtorch/share/cmake/Torch")

# --- 1. 查找依赖包 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Torch REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

set(MVSDK_ROOT /home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK)
set(MVSDK_INCLUDE ${MVSDK_ROOT}/include)
set(MVSDK_LIB ${MVSDK_ROOT}/lib/64)

# 可选：检查路径是否存在
if(NOT EXISTS ${MVSDK_INCLUDE})
  message(FATAL_ERROR "MVS SDK include not found: ${MVSDK_INCLUDE}")
endif()

if(NOT EXISTS ${MVSDK_LIB})
  message(FATAL_ERROR "MVS SDK lib not found: ${MVSDK_LIB}")
endif()

# --- 2. 添加可执行文件（必须提前！）---
add_executable(hik_camera_node
  src/hik_camera_node.cpp
  src/hik_camera_driver.cpp
)

add_executable(
  light_strip_detector
  src/light_strip_detector.cpp
  src/armor_classifier.cpp
  src/pose_estimator.cpp
)


# 设置 C++17
target_compile_features(hik_camera_node PUBLIC cxx_std_17)

# 添加头文件路径
include_directories(
  include
  /home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK/include
)

# 链接 ROS 依赖
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

ament_target_dependencies(light_strip_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)


target_link_libraries(light_strip_detector ${TORCH_LIBRARIES})


# ✅ 最后一步：链接海康 SDK 库（必须在 add_executable 之后）
target_link_libraries(hik_camera_node
  /home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK/lib/64/libMvCameraControl.so
  /home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK/lib/64/libFormatConversion.so
)

# ✅ 设置运行时库搜索路径（RPATH）
set_target_properties(hik_camera_node PROPERTIES
  # 编译时查找路径
  BUILD_RPATH "${MVSDK_LIB}"
  
  # 安装后运行时查找路径
  # $ORIGIN 表示可执行文件所在目录
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/lib:${MVSDK_LIB}"
  
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH FALSE
)

# --- 安装 ---
install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS light_strip_detector
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()