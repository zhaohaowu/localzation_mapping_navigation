cmake_minimum_required(VERSION 2.8.3)
project(vd_msgs)

find_package(catkin REQUIRED COMPONENTS
  rosmsg
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    AckermannDrive.msg
    AckermannDriveStamped.msg
    ChassisCtrl.msg
    ChassisInfo.msg
    GpsInfo.msg
    GpsRaw.msg
    ImuCalib.msg
    PlantProtectCtrl.msg
    PlantProtectStatus.msg
    SystemStatus.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    message_generation
    geometry_msgs 
    std_msgs
    nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



