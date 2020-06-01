cmake_minimum_required(VERSION 2.8.3)
project(etddf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  message_generation
  uuv_control_msgs
  robot_localization
)

catkin_python_setup()

add_message_files(
   FILES
   Measurement.msg
   MeasurementPackage.msg
   AssetEstimate.msg
   NetworkEstimate.msg
<<<<<<< HEAD
<<<<<<< HEAD
   EtddfStatistics.msg
=======
=======
   EtddfStatistics.msg
>>>>>>> Added Etddf Stats Messages
   Sonar.msg
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> Added node to simulate sonar
=======
   Waypoint.msg
   WaypointSet.msg
>>>>>>> Added Enemy Rov script to move the rexrov
=======
>>>>>>> Added beginning of automating the simulation
)

add_service_files(
  FILES
  GetMeasurementPackage.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)