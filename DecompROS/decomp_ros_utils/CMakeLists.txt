cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加必要的头文件目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include/decomp_basis
  ${PROJECT_SOURCE_DIR}/include/decomp_geometry
  ${PROJECT_SOURCE_DIR}/include/decomp_ros_utils
  ${PROJECT_SOURCE_DIR}/include/decomp_util
)

# 添加必要的库
find_package(Eigen3 REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# 添加测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package()