cmake_minimum_required(VERSION 3.8)
project(mapir_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(camera_node_cpp src/camera_node.cpp)
ament_target_dependencies(camera_node_cpp
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  camera_info_manager
  rcl_interfaces
)
target_include_directories(camera_node_cpp PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_node_cpp ${OpenCV_LIBS})

install(TARGETS camera_node_cpp DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gstreamer_pipeline test/test_gstreamer_pipeline.cpp)
  target_include_directories(test_gstreamer_pipeline PUBLIC include)
endif()

ament_package()
