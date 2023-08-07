cmake_minimum_required(VERSION 3.0.2)
project(fet_example)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    geometry_msgs
    turtlebot3_msgs
    message_generation
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    sensor_msgs
    geometry_msgs
    turtlebot3_msgs
    message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/fet_goal_key
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)