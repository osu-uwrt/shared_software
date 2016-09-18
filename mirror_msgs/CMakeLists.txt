cmake_minimum_required(VERSION 2.8.3)
project(mirror_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(FILES OdomWithAccel.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
