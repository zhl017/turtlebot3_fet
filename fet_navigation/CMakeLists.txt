cmake_minimum_required(VERSION 3.0.2)
project(fet_navigation)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)