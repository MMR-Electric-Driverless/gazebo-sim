cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(pure_pursuit_node ${SRC_FILES})
ament_target_dependencies(pure_pursuit_node ament_cmake rclcpp visualization_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


ament_package()
