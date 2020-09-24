/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_trajectory");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  static const std::string PLANNING_GROUP = "L_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint"); //STANDARD FRAME
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  //Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");

  std::copy(move_group.getJointNames().begin(),move_group.getJointNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Planning to a Pose goal
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = move_group.getCurrentPose().pose.position.x ;
  target_pose1.position.y = move_group.getCurrentPose().pose.position.y +0.1;
  target_pose1.position.z = move_group.getCurrentPose().pose.position.z ;
  target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z;
//  target_pose1.orientation.w = 0.0;
//  target_pose1.position.x = 0.3;
//  target_pose1.position.y = 0.25;
//  target_pose1.position.z = 0.75;

//  target_pose1 = move_group.getPoseTarget(move_group.getEndEffectorLink());

  move_group.setPoseTarget(target_pose1);
  visual_tools.publishAxisLabeled(target_pose1, "pose1");


  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Now, we call the planner to compute the plan and visualize it. // Note that we are just planning,
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
  ROS_INFO_NAMED("moveo", "Visualizing plan 1 (pose goal) %s", success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "" : "FAILED");

  // Visualizing plans
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); //TRAJECTORY*******************************************************//

  if(success)
      move_group.move();


  //TEST
  moveit_msgs::RobotTrajectory msg;
  msg = my_plan.trajectory_;
  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points;
  std::vector<int>::size_type size1 = msg.joint_trajectory.points.size();

  double time_begin =ros::Time::now().toSec();

  if(success)
  {
      std::cout << std::endl << "TEST" << std::endl;
      std::cout << "size : " << msg.joint_trajectory.points.size() << std::endl;
      std::cout << "Joint name = " << msg.joint_trajectory.joint_names[0] << std::endl;
      for(int i=0; i< size1; i++){
          std::cout << "Trajectory point["<<i<<"].Position[0] = " << msg.joint_trajectory.points[i].positions[0] << std::endl;
          if ( i > 0)
            std::cout << "Duration point["<<i<<"].duration = " << msg.joint_trajectory.points[i].time_from_start.toSec() - msg.joint_trajectory.points[i-1].time_from_start.toSec() << std::endl;

      }
      std::cout << "Trajectory points = " << msg.joint_trajectory.points.size() << std::endl;


  }



  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  ros::shutdown();
  return 0;
}
