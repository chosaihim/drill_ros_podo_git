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
  ros::init(argc, argv, "gripper_test");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  //ac_base.waitForServer(); //will wait for infinite time
  //ac_arm.waitForServer(); //will wait for infinite time
  ac_gripper.waitForServer(); //will wait for infinite time
  
  
  ROS_INFO("Wait for it...");
  // send a goal to the action
  ros_podo_connector::RosPODO_BaseGoal goal_base;
  ros_podo_connector::RosPODO_ArmGoal goal_arm;
  ros_podo_connector::RosPODO_GripperGoal goal_gripper;

  // action result
  ros_podo_connector::RosPODO_BaseResult result_base;
  ros_podo_connector::RosPODO_ArmResult result_arm;
  ros_podo_connector::RosPODO_GripperResult result_gripper;
  


  ROS_INFO("Action server started, sending goal.");
  
  int cnt =0;
  while(cnt<5)
  {
	cnt++;
    /* ============== Gripper Data Action  ==============  */
    //OPEN============
    goal_gripper.grippermove_cmd = GRIPPER_OPEN;
    
    std::cout << "cmd: " << goal_gripper.grippermove_cmd << std::endl;	
    
    goal_gripper.mode = GRIPPER_RIGHT;

    ac_gripper.sendGoal(goal_gripper);
    ROS_INFO("Gripper OPEN");

    //wait for the action to return
    bool finished_before_timeout = ac_gripper.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_gripper.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
      

    ros::Duration(2).sleep();


    //CLOSE===========
    goal_gripper.grippermove_cmd = GRIPPER_CLOSE;
    goal_gripper.mode = GRIPPER_RIGHT;
    std::cout << "cmd: " << goal_gripper.grippermove_cmd << std::endl;	

    ac_gripper.sendGoal(goal_gripper);
    ROS_INFO("Gripper CLOSE");

    //wait for the action to return
    //bool finished_before_timeout = ac_gripper.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_gripper.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
      
      
      
   ros::Duration(2).sleep();
   
  }



  //exit
  return 0;
}
