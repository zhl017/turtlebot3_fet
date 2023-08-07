cmake_minimum_required(VERSION 3.0.2)
project(fet_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    diagnostic_msgs
    turtlebot3_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    diagnostic_msgs
    turtlebot3_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(fet_diagnostics src/fet_diagnostics.cpp)
add_dependencies(fet_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fet_diagnostics ${catkin_LIBRARIES})

install(TARGETS fet_diagnostics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS 
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 99-turtlebot3-cdc.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)