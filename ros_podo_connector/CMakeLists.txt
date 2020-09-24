cmake_minimum_required(VERSION 2.8.3)
project(ros_podo_connector)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  nav_msgs
  actionlib
  actionlib_msgs
)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Boost REQUIRED COMPONENTS system)




## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   JointData.msg
   WbikData.msg
   JointDataSet.msg
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   RosPODOmotion.action
   RosPODO_Base.action
   RosPODO_Arm.action
   RosPODO_Gripper.action
   RosPODO_Traj.action
   RosPODO_Trajectory.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 )



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#node for TX/RX with PODO software
add_executable(ros_podo_connector src/ros_podo_connector.cpp)
add_dependencies(ros_podo_connector drc_podo_connector_generate_messages_cpp)
target_link_libraries(ros_podo_connector ${catkin_LIBRARIES})

#end effector Test
add_executable(endPose_client src/endPose_client.cpp)
add_dependencies(endPose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(endPose_client ${catkin_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp std_msgs
 DEPENDS Boost
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)
