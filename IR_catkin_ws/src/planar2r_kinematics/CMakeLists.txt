cmake_minimum_required(VERSION 2.8.3)
project(planar2r_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
	message_generation
	tf
)

catkin_package(
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(../../../IR_library/include)
LINK_DIRECTORIES(../../../IR_library/lib)

add_executable(planar2r_kinematics_forwardKinematics src/forwardKinematics.cpp)
target_link_libraries(planar2r_kinematics_forwardKinematics ${catkin_LIBRARIES} IRLibrary)

add_executable(planar2r_kinematics_geometry_shapes src/geometry_shapes.cpp)
target_link_libraries(planar2r_kinematics_geometry_shapes ${catkin_LIBRARIES})

####################
## Add genConfig build instructions ##
####################
