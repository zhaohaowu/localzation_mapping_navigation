cmake_minimum_required(VERSION 3.0.8)
project(can_com)

add_compile_options(-std=c++14)
add_compile_options(-fPIC)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++14 -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  serial  
  vd_msgs
  laser_geometry
#  tf2_msgs
#  tf2_sensor_msgs
#  pcl_ros  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)

add_service_files(
  FILES
  CanCtrlInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES can_com
   CATKIN_DEPENDS 
   roscpp
   rospy
   message_runtime
   std_msgs 
   nav_msgs
   geometry_msgs
#   DEPENDS system_lib
)


link_directories(
  lib
  ${catkin_LIB_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(can_capture_emcb src/can_capture_emcb.cpp)
target_link_libraries(can_capture_emcb ${catkin_LIBRARIES})
target_link_libraries(can_capture_emcb ${catkin_LIBRARIES} EMCB-1.00)
target_link_libraries(can_capture_emcb ${catkin_LIBRARIES} -lpthread -lm)
add_dependencies(can_capture_emcb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(can_server_emcb src/can_server_emcb.cpp)
target_link_libraries(can_server_emcb ${catkin_LIBRARIES})
add_dependencies(can_server_emcb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
