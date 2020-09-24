#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
#include <ros_podo_connector/RosPODO_TrajectoryAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"
/*for MoveIt! interface*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
/*for ROS additional msgs */
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

enum TRAJECTORY_CMD
{
    MOVE_ABSOLUTE = 0,
    MOVE_RELATIVE
};

int max_path = 100;

const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};


int main (int argc, char **argv)
{
    //Node initialization
    ros::init(argc, argv, "traj_send_client");
    ros::AsyncSpinner spinner(1);
    spinner.start();



    /* ============== Getting Goal from Parameters===============  */
    // handler for parameters
    ros::NodeHandle nh("~");
    //Set variables and initial values(default values)
    std::string move_mode = "RELATIVE", plan_group = "L_arm";
    double x_goal = 0., y_goal=0., z_goal=0., w_goal=0., wx_goal = 0., wy_goal = 0.,  wz_goal = 0.;

    //get parameters
    nh.getParam("mode", move_mode); nh.getParam("plangroup", plan_group);
    nh.getParam("x", x_goal); nh.getParam("y", y_goal); nh.getParam("z", z_goal);
    nh.getParam("w", w_goal); nh.getParam("wx", wx_goal); nh.getParam("wy", wy_goal); nh.getParam("wz", wz_goal);



    /* ============== Action Server Setting ===============  */
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_TrajectoryAction> ac_trajectory("rospodo_trajectory", true);

    // wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac_trajectory.waitForServer(); //will wait for infinite time

    // send a goal to the action
    ROS_INFO("Action server started, sending goal.");
    ros_podo_connector::RosPODO_TrajectoryGoal      goal_trajectory;



    /* ============== MoveGroup(MoveIT!) Setting ===============  */
    const std::string PLANNING_GROUP = plan_group;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint"); //STANDARD FRAME
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Text Visualization:
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.5;
    visual_tools.publishText(text_pose, "Attempting", rvt::WHITE, rvt::XLARGE);

    std::copy(move_group.getJointNames().begin(),move_group.getJointNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));


    /* ============== Setting Goal ===============  */
    // Planning to a Pose goal
    int move_mode_int = 1;
    geometry_msgs::Pose target_pose1;
    if(move_mode == "ABSOLUTE") move_mode_int = MOVE_ABSOLUTE;
    else move_mode_int = MOVE_RELATIVE;

    switch(move_mode_int)
    {
        case MOVE_ABSOLUTE:
        {
            target_pose1.position.x = x_goal;
            target_pose1.position.y = y_goal;
            target_pose1.position.z = z_goal;
            target_pose1.orientation.w = w_goal;
            target_pose1.orientation.x = wx_goal;
            target_pose1.orientation.y = wy_goal;
            target_pose1.orientation.z = wz_goal;
            break;
        }
        case MOVE_RELATIVE:
        {
            target_pose1.position.x = move_group.getCurrentPose().pose.position.x + x_goal;
            target_pose1.position.y = move_group.getCurrentPose().pose.position.y + y_goal;
            target_pose1.position.z = move_group.getCurrentPose().pose.position.z + z_goal;
            target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w + w_goal;
            target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x + wx_goal;
            target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y + wy_goal;
            target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z + wz_goal;
            break;
        }
    }


    /* ============== MoveIT! Planning ===============  */
    move_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

    visual_tools.publishAxisLabeled(target_pose1, "Goal Pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    //Trajectory Planning
    moveit_msgs::RobotTrajectory msg = my_plan.trajectory_;
    //std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points;
    std::vector<int>::size_type traj_size = msg.joint_trajectory.points.size();
    std::cout << "trajectory Size: " << msg.joint_trajectory.points.size() << std::endl;


    if(success && traj_size < max_path)
    {
        std::cout <<"Motion Plan Found" << std::endl;
        move_group.move();
        goal_trajectory.num_points = traj_size;

        for(int p=0; p<traj_size; p++){    //p: points of Trajectory
            for(int robot_j=0; robot_j< NUM_JOINTS; robot_j++)//NUM_JOINTS; robot_j++)    //robot_j: roboto joints
            {
                for(int traj_j=0; traj_j< msg.joint_trajectory.joint_names.size(); traj_j++)     // traj_j: trajectory joints
                {
                    //only if joint name matches
                    if(JointBufferNameList[robot_j] == msg.joint_trajectory.joint_names[traj_j])
                    {
                        goal_trajectory.via_point[p].joint[robot_j].OnOffControl = CONTROL_ON;
                        goal_trajectory.via_point[p].joint[robot_j].reference = msg.joint_trajectory.points[p].positions[traj_j];
                        if(p > 0) goal_trajectory.via_point[p].joint[robot_j].GoalmsTime = (msg.joint_trajectory.points[p].time_from_start.toSec() - msg.joint_trajectory.points[p-1].time_from_start.toSec());
                        else goal_trajectory.via_point[p].joint[robot_j].GoalmsTime = 0.0;
                        break;
                    }
                    else{
                        goal_trajectory.via_point[p].joint[robot_j].OnOffControl = CONTROL_OFF;
                        goal_trajectory.via_point[p].joint[robot_j].reference = 0.;
                        goal_trajectory.via_point[p].joint[robot_j].GoalmsTime = 0.;
                    }
                }
            }
        }



        /* ============== Send Action Request ===============  */

        // send a goal to the server
        ac_trajectory.sendGoal(goal_trajectory);

        //wait for the action to return
        bool finished_before_timeout = ac_trajectory.waitForResult(ros::Duration(100.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_trajectory.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");

    }

    //exit
    nh.deleteParam("mode"); nh.deleteParam("plangroup");
    nh.deleteParam("x"); nh.deleteParam("y"); nh.deleteParam("z");
    nh.deleteParam("w"); nh.deleteParam("wx"); nh.deleteParam("wy"); nh.deleteParam("wz");



	//****************Gripper TEST******************
	/*=============== Gripper action things ===========*/
	actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);
	ac_gripper.waitForServer(); //will wait for infinite time
	ros_podo_connector::RosPODO_GripperGoal goal_gripper;

	/* ============== Grasp Action ==============  */
	goal_gripper.grippermove_cmd = GRIPPER_OPEN;
	goal_gripper.mode = GRIPPER_RIGHT;
	std::cout << "Sending gripper goal" << std::endl;
	ac_gripper.sendGoal(goal_gripper);
	  	  
	//wait for the action to return
	bool finished_before_timeout_gripper = ac_gripper.waitForResult(ros::Duration(100.0));
	if (finished_before_timeout_gripper){
		actionlib::SimpleClientGoalState state = ac_gripper.getState();
		ROS_INFO("Gripper closed: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Gripper open did not finish before the time out.");
	//*************Gripper TEST END***************

    return 0;
}
