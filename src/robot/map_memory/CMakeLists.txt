cmake_minimum_required(VERSION 3.10)
project(map_memory)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Build the core library
add_library(map_memory_lib
  src/map_memory_core.cpp)

# Include header files for both the library and node
target_include_directories(map_memory_lib
  PUBLIC include)

# Link ROS 2 dependencies to the library
ament_target_dependencies(map_memory_lib 
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs

)

# Create the main node executable
add_executable(map_memory_node src/map_memory_node.cpp)
target_link_libraries(map_memory_node map_memory_lib)

# Link dependencies to the node
ament_target_dependencies(map_memory_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs

)

# Install targets
install(TARGETS
  map_memory_node
  map_memory_lib
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install config files (if any)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
