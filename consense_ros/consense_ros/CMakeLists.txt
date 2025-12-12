cmake_minimum_required(VERSION 3.0.2)
project(consense_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/anomaly_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_message_files(
  FILES
  ConSenseStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS ... message_runtime
)