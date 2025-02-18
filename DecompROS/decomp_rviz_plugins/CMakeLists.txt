cmake_minimum_required(VERSION 3.8)
project(decomp_rviz_plugins)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 添加必要的头文件目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include/decomp_rviz_plugins

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
find_package(decomp_ros_utils REQUIRED)

# 添加源文件
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/mesh_shape.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
)

# 添加测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

# 导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# 安装图标和头文件
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package()