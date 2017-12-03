cmake_minimum_required(VERSION 2.8.3)
project(shared_utils)

find_package(catkin REQUIRED COMPONENTS roscpp roslint cv_bridge image_transport sensor_msgs)

catkin_package()
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

roslint_cpp()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(extract_video src/extract_video.cpp)
target_link_libraries(extract_video ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
