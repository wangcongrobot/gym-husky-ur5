cmake_minimum_required(VERSION 2.8.3)
project(husky_train)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  roscpp
  rospy
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
)
find_package(Eigen REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   EePose.srv
   EeRpy.srv
   EeTraj.srv
   JointTraj.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs trajectory_msgs
 )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES husky_train
  CATKIN_DEPENDS 
    message_runtime
    rospy 
    moveit_core  
    moveit_ros_planning_interface 
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include motion_planner
 ${catkin_INCLUDE_DIRS}
   include ${Eigen_INCLUDE_DIRS}
)

# Library files directory
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library : custom DH kinematics
add_library(kinematics_parser src/kinematics_parser.cpp)
target_link_libraries(kinematics_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ library : motion_planner
add_library(motion_planner src/motion_planner.cpp)
target_link_libraries(motion_planner kinematics_parser ${catkin_LIBRARIES})

add_executable(motion_planner_node src/motion_planner_node.cpp)
target_link_libraries(motion_planner_node 
                      motion_planner
                      ${catkin_LIBRARIES})

add_executable(motion_planner_node_test src/motion_planner_node_test.cpp)
target_link_libraries(motion_planner_node_test
                      motion_planner
                      ${catkin_LIBRARIES})

add_executable(kinematics_parser_test src/kinematics_parser_test.cpp)
target_link_libraries(kinematics_parser_test
                      kinematics_parser
                      motion_planner
                      ${catkin_LIBRARIES})