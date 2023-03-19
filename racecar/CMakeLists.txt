cmake_minimum_required(VERSION 3.0.2)
project(racecar)


find_package(catkin REQUIRED COMPONENTS roscpp actionlib)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES racecar
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(odom2path src/odom2path.cpp)
target_link_libraries(odom2path
  ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${aruco_LIBS}
)