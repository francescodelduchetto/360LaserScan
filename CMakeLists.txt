cmake_minimum_required(VERSION 2.8.3)
project(scan_360)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  map_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS scripts/360LaserScan.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
