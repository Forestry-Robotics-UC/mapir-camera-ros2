cmake_minimum_required(VERSION 3.8)
project(mapir_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

ament_python_install_package(mapir_camera_ros2)
ament_python_install_package(mapir_camera_core)

install(PROGRAMS
  mapir_camera_ros2/scripts/camera_node
  mapir_camera_ros2/scripts/indices_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_node_cpp mapir_camera_ros2/src/camera_node.cpp)
target_include_directories(camera_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mapir_camera_ros2/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_node_cpp
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  camera_info_manager
  rcl_interfaces
)
target_link_libraries(camera_node_cpp ${OpenCV_LIBS})

install(TARGETS camera_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY mapir_camera_ros2/include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
install(FILES resource/mapir_camera_ros2
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gstreamer_pipeline mapir_camera_ros2/test/test_gstreamer_pipeline.cpp)
  target_include_directories(test_gstreamer_pipeline PUBLIC mapir_camera_ros2/include)
endif()

ament_package()
