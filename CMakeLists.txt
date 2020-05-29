cmake_minimum_required(VERSION 2.8.3)
project(etddf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

add_message_files(
   FILES
   Measurement.msg
   MeasurementPackage.msg
   AssetEstimate.msg
   NetworkEstimate.msg
   EtddfStatistics.msg
)

add_service_files(
  FILES
  GetMeasurementPackage.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)