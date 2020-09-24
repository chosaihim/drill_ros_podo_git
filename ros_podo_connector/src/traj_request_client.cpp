#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
#include <ros_podo_connector/RosPODO_TrajAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"

enum TRAJECTORY_CMD
{
    MOVE_ABSOLUTE = 0,
    MOVE_RELATIVE
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "traj_request_client");


    ros::NodeHandle nh("~");

    //Set variables and initial values
    std::string move_mode = "RELATIVE", plan_group = "L_arm";
    double x_goal = 0., y_goal=0., z_goal=0., w_goal=0., wx_goal = 0., wy_goal = 0.,  wz_goal = 0.;

    nh.getParam("mode", move_mode); nh.getParam("plangroup", plan_group);
    nh.getParam("x", x_goal); nh.getParam("y", y_goal); nh.getParam("z", z_goal);
    nh.getParam("w", w_goal); nh.getParam("wx", wx_goal); nh.getParam("wy", wy_goal); nh.getParam("wz", wz_goal);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_TrajAction> ac_traj("rospodo_traj", true);

    // wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac_traj.waitForServer(); //will wait for infinite time

    // send a goal to the action
    ROS_INFO("Action server started, sending goal.");
    ros_podo_connector::RosPODO_TrajGoal      goal_traj;

    /* ============== Move Endeffector-==============  */

    //Set a goal
    if(move_mode == "ABSOLUTE") goal_traj.traj_cmd = MOVE_ABSOLUTE;
    else goal_traj.traj_cmd = MOVE_RELATIVE;

    goal_traj.x = x_goal;               //0.;
    goal_traj.y = y_goal;               //0.;
    goal_traj.z = z_goal;               //0.;
    goal_traj.ori_w = w_goal;           //0.;
    goal_traj.ori_x = wx_goal;          //0.;
    goal_traj.ori_y = wy_goal;          //0.;
    goal_traj.ori_z = wz_goal;          //0.;
    goal_traj.planGroup = plan_group;   //"L_arm";

    //Print Goal on the terminal
    ROS_INFO("Mode: %s", move_mode.c_str());
    ROS_INFO("PLAN Group: %s", plan_group.c_str());
    ROS_INFO("x, y, z: %lf, %lf, %lf", x_goal, y_goal, z_goal);
    ROS_INFO("w, wx, wy, wz: %lf, %lf, %lf, %lf", w_goal, wx_goal, wy_goal, wz_goal);

    ac_traj.sendGoal(goal_traj);
    ros::Duration(6).sleep();

    //wait for the action to return
    bool finished_before_timeout = ac_traj.waitForResult(ros::Duration(100.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_traj.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");


    //exit
    nh.deleteParam("mode"); nh.deleteParam("plangroup");
    nh.deleteParam("x"); nh.deleteParam("y"); nh.deleteParam("z");
    nh.deleteParam("w"); nh.deleteParam("wx"); nh.deleteParam("wy"); nh.deleteParam("wz");

    return 0;
}
