cmake_minimum_required(VERSION 3.0.2)
project(navigation)


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

add_executable(odom2path2 src/odom2path2.cpp)
target_link_libraries(odom2path2
  ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${aruco_LIBS}
)