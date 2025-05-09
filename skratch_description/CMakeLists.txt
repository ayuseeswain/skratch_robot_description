cmake_minimum_required(VERSION 2.8.3)
project(skratch_description)

# Keep C/C++ standard settings (they are harmless in ROS 1)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 1 build system
find_package(catkin REQUIRED)

catkin_package()

# Install description assets
install(
  DIRECTORY config description launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)