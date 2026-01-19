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

install(PROGRAMS
  scripts/camera_node
  scripts/indices_node
  scripts/reflectance_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_node_cpp src/camera_node.cpp)
target_include_directories(camera_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
install(DIRECTORY include/
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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gstreamer_pipeline test/test_gstreamer_pipeline.cpp)
  target_include_directories(test_gstreamer_pipeline PUBLIC include)

  if(NOT DEFINED AMENT_CMAKE_PYTEST_WITH_COVERAGE)
    set(AMENT_CMAKE_PYTEST_WITH_COVERAGE OFF CACHE BOOL "Pytest coverage handled via PYTEST_ADDOPTS")
  endif()
  # Prevent lint tools from scanning generated build artifacts.
  file(WRITE "${CMAKE_BINARY_DIR}/../AMENT_IGNORE" "")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(pytests test
    ENV
      "PYTEST_ADDOPTS=-m \"not performance\" --cov=mapir_camera_ros2.core --cov-report=term --cov-fail-under=90"
      "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs"
  )
  ament_add_pytest_test(perf_tests test/performance
    ENV
      "PYTEST_ADDOPTS=-m performance"
      "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_logs"
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini")
  set(AMENT_LINT_AUTO_FILE_EXCLUDE build install log .venv doc/_build)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
