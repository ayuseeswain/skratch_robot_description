cmake_minimum_required(VERSION 2.8.3)
project(kinova_arm_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files if the directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# Install config and .setup_assistant
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES .setup_assistant
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
