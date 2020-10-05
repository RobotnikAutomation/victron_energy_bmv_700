cmake_minimum_required(VERSION 2.8.3)
project(victron_energy_bmv_700)

find_package(catkin REQUIRED COMPONENTS
  robotnik_msgs
  rospy
  std_srvs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES victron_energy_bmv_700
#  CATKIN_DEPENDS robotnik_msgs rospy std_srvs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

