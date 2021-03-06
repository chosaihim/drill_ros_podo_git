#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "testClient");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_base.waitForServer(); //will wait for infinite time
  ac_arm.waitForServer(); //will wait for infinite time
  ac_gripper.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  
  // send a goal to the action
  ros_podo_connector::RosPODO_BaseGoal goal_base;
  ros_podo_connector::RosPODO_BaseGoal goal_base_return;
  ros_podo_connector::RosPODO_ArmGoal goal_arm;
  ros_podo_connector::RosPODO_GripperGoal goal_gripper;
  
  
  /* ============== Go2Display Action  ==============  */
  
  goal_base.wheelmove_cmd = 1;
  goal_base.MoveX = -3.0;
  goal_base.MoveY = 0.0;
  goal_base.ThetaDeg = 0;
  
  ac_base.sendGoal(goal_base);
  
  goal_arm.jointmove_cmd = MODE_MOVE_JOINT;
  goal_arm.joint_ref[rosWST].OnOffControl = CONTROL_ON;
  goal_arm.joint_ref[rosWST].reference = -180.0;
  goal_arm.joint_ref[rosWST].GoalmsTime = 5000;
  
  goal_arm.joint_ref[rosRWY2].OnOffControl = CONTROL_ON;
  goal_arm.joint_ref[rosRWY2].reference = -90.0;
  goal_arm.joint_ref[rosRWY2].GoalmsTime = 5000;
  
  ac_arm.sendGoal(goal_arm);
  
  ros::Duration(6).sleep();

		  
  /* ============== Grasp Action ==============  */
  
  
  goal_arm.joint_ref[rosRWY2].OnOffControl = CONTROL_OFF;
  
  goal_gripper.grippermove_cmd = GRIPPER_OPEN;
  goal_gripper.mode = GRIPPER_RIGHT;
  ac_gripper.sendGoal(goal_gripper);
  

  goal_arm.jointmove_cmd = 3;
  goal_arm.wbik_ref[RIGHT_HAND].OnOff_position = CONTROL_ON;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[0] = 0.6;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[1] = -0.3;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[2] = 0.2;
  goal_arm.wbik_ref[RIGHT_HAND].GoalmsTime = 3000;
  
  goal_arm.wbik_ref[RIGHT_ELBOW].OnOff_position = CONTROL_ON;
  goal_arm.wbik_ref[RIGHT_ELBOW].goal_angle = -30.0;
  goal_arm.wbik_ref[RIGHT_ELBOW].GoalmsTime = 3000;
  
  ac_arm.sendGoal(goal_arm);
  ros::Duration(4).sleep();
  
  goal_gripper.grippermove_cmd = GRIPPER_CLOSE;
  goal_gripper.mode = GRIPPER_RIGHT;
  ac_gripper.sendGoal(goal_gripper);
   ros::Duration(2).sleep();
  
  /* ============== Go2Home Action  ==============  */
  
  goal_base.wheelmove_cmd = 1;
  goal_base.MoveX = 3.0;
  goal_base.MoveY = 0.0;
  goal_base.ThetaDeg = 0;
  ac_base.sendGoal(goal_base);
  
  ros::Duration(2).sleep();
  
  goal_arm.jointmove_cmd = MODE_MOVE_JOINT;
  goal_arm.joint_ref[rosWST].OnOffControl = CONTROL_ON;
  goal_arm.joint_ref[rosWST].reference = 0.0;
  goal_arm.joint_ref[rosWST].GoalmsTime = 5000;
  
  
  ac_arm.sendGoal(goal_arm);
  
  ros::Duration(6).sleep();
  
  goal_gripper.grippermove_cmd = GRIPPER_OPEN;
  goal_gripper.mode = GRIPPER_RIGHT;
  ac_gripper.sendGoal(goal_gripper);

    
  //wait for the action to return
  bool finished_before_timeout = ac_base.waitForResult(ros::Duration(100.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_base.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
