cmake_minimum_required(VERSION 2.8.3)
project(etddf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   Measurement.msg
   MeasurementPackage.msg
   AssetEstimate.msg
   NetworkEstimate.msg
   EtddfStatistics.msg
   Sonar.msg
   SonarSettings.msg
)

add_service_files(
  FILES
  GetMeasurementPackage.srv
  SetSonarSettings.srv
  GetSonarSettings.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)