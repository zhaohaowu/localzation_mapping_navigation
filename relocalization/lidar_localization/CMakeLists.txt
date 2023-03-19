cmake_minimum_required(VERSION 3.10.2)
project(lidar_localization)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

# add_message_files(
#   DIRECTORY msg
#   #FILES
#     #GpsRaw.msg
# )

# add_service_files(
#    FILES
#    saveScanContext.srv
#    saveMap.srv
#    optimizeMap.srv
# )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/YAML.cmake)
#include(cmake/geographic.cmake)
include(cmake/protobuf.cmake)
include(cmake/PCL.cmake)
include(cmake/g2o.cmake)
include(cmake/OpenCV.cmake)
include(cmake/ceres.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(matching_node src/apps/matching_node.cpp ${ALL_SRCS})
add_dependencies(matching_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(matching_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(image_projection_node src/apps/loam_fea/image_projection_node.cpp ${ALL_SRCS})
add_dependencies(image_projection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_projection_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(feature_extraction_node src/apps/loam_fea/feature_extraction_node.cpp ${ALL_SRCS})
add_dependencies(feature_extraction_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_extraction_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(odom2tf_node src/apps/odom2tf_node.cpp ${ALL_SRCS})
add_dependencies(odom2tf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom2tf_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(pub_odom_node src/apps/pub_odom_node.cpp ${ALL_SRCS})
add_dependencies(pub_odom_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_odom_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS 
        data_pretreat_node
        matching_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)
