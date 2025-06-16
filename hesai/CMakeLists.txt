cmake_minimum_required(VERSION 3.8)
project(hesai_lidar_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find system dependencies
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Create library
add_library(${PROJECT_NAME}
  src/hesai_driver.cpp
  src/packet_parser.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)

# Create executable
add_executable(hesai_lidar_node src/hesai_lidar_node.cpp)

ament_target_dependencies(hesai_lidar_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(hesai_lidar_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)

# Create debug executable (without PacketParser dependency for now)
add_executable(debug_segfault src/debug_segfault.cpp)
ament_target_dependencies(debug_segfault rclcpp)
target_link_libraries(debug_segfault ${PROJECT_NAME})

# Create demonstration executable (with persistence warnings)
add_executable(hesai_demo_node src/hesai_demo_node.cpp)
ament_target_dependencies(hesai_demo_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(hesai_demo_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  hesai_lidar_node
  hesai_demo_node
  debug_segfault
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Lint tests
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
