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
  ros::init(argc, argv, "endPose");
  
  //std::string param;
  ros::NodeHandle nh("~");
  float x = 0., y = 0., z = 0.;
  float baseX = 0., baseY = 0.;

  nh.param("x", x, 0.5);
  nh.param("y", y, -0.246403);
  nh.param("z", z, 0.65);
  nh.param("baseX", baseX, 0.20);
  nh.param("baseY", baseY, 0.0);

  nh.deleteParam("x");  nh.deleteParam("y");  nh.deleteParam("z");
  nh.deleteParam("baseX");  nh.deleteParam("baseY");

  std::cout << "hand(x,y,z) = (" << x << ", " << y << ", " << z << ") " << std::endl;
  std::cout << "base(x,y) = (" << baseX << ", " << baseY << ") " << std::endl;

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
  ros_podo_connector::RosPODO_ArmGoal goal_arm;
  ros_podo_connector::RosPODO_GripperGoal goal_gripper;
  

  // ============== Base Data Action  ============== //
  goal_base.wheelmove_cmd = 1;
  goal_base.MoveX = baseX;//0.20;
  goal_base.MoveY = baseY;//0.0;
  goal_base.ThetaDeg = 0;
  ac_base.sendGoal(goal_base);


  //wait for the action to return
  bool finished_before_timeout_base = ac_base.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout_base){
    actionlib::SimpleClientGoalState state = ac_base.getState();
    ROS_INFO("BASE Action finished: %s",state.toString().c_str());
  }
  else{
    ROS_INFO("BASE Action did notfinish before the time out.");
  }
  
  // ============== Arm Data Action WBIK ==============  //
  //goal_arm.jointmove_cmd = 3;
  goal_arm.jointmove_cmd = 7;
  goal_arm.wbik_ref[RIGHT_HAND].OnOff_position = CONTROL_ON;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[0] = x;// 0.5;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[1] = y;//-0.246403;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[2] = z;// 0.65;
  goal_arm.wbik_ref[RIGHT_HAND].GoalmsTime = 2000;

  ac_arm.sendGoal(goal_arm);
//  ros::Duration(3).sleep();

  //wait for the action to return
  bool finished_before_timeout_arm = ac_arm.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout_arm)
  {
    actionlib::SimpleClientGoalState state = ac_arm.getState();
    ROS_INFO("ARM Action finished: %s",state.toString().c_str());
  }
  else{
    ROS_INFO("ARM Action did not finish before the time out.");
  }


  //exit
  return 0;
}
