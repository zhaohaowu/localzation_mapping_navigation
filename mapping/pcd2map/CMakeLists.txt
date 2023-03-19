cmake_minimum_required(VERSION 2.8.3)
project(pcd2map)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	rostime 
	sensor_msgs 
	message_filters 
)


find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_cameracalibrate
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_executable(pcd2map src/pcd2map.cpp)

target_link_libraries(pcd2map
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)


