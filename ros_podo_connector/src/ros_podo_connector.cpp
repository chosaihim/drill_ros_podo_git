/* =============================================================
 *
 * This ROS node is Action Server handler for Mobile-Hubo Platform.
 * Receives ROS Action msgs {Base, Arm, Gripper} and TX/RX to PODO Software Framework.
 * In order for correct operation, please ensure PODO Software version is 'MobileHubo_ROSmotion' inside Hubo Motion PC.
 * Refer to ROSPodo Motion Manual at www.kirobotics.com
 * 
 *
 * Output : /rospodo_base/feedback
 *          /rospodo_base/result
 * 			/rospodo_base/status
 * 			/rospodo_arm/feedback
 *        : /rospodo_arm/result
 * 			/rospodo_arm/status
 * 			/rospodo_gripper/feedback
 *        : /rospodo_gripper/result
 * 			/rospodo_gripper/status
 * 
 * Input  : /rospodo_base/goal
 * 			/rospodo_arm/goal
			/rospodo_gripper/goal
 
 * E-mail : ml634@kaist.ac.kr     (Moonyoung Lee)
 * E-mail : blike4@kaist.ac.kr    (Heo Yujin)
 * E-mail : chosaihim@kaist.ac.kr (Cho Saihim)
 *
 * Versions :
 * v2.0.2019.09
 * =============================================================
 */
 
/* =============Header file include ============= */
/*for ROS Action msg */
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"


/*Pre-defined Action msg for PODO motion */
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
#include <ros_podo_connector/RosPODO_TrajAction.h>
#include <ros_podo_connector/RosPODO_TrajectoryAction.h>

/*for TCP/IP socket client */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/*for ROS additional msgs */
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

/*for MoveIt! interface*/
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

/*for multi-threaded locks */
//#include <mutex>
//std::mutex variable_lock;


/* =============TCP/IP  global variables initialize ============= */
#define PODO_ADDR       "127.0.0.1"
#define PODO_PORT       5000
#define PODO_TRAJ_PORT  7000

char ip[20];
int sock = 0;
int sock_traj = 0;
struct sockaddr_in  server;
struct sockaddr_in  server_traj;

pthread_t LANTHREAD_t;
int threadWorking = true;
int connectionStatus = false;
int ON_publish = true;

//custom header RX/TX buffer
LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;

int     RXDataSize;
int     TXDataSize;
void*   RXBuffer;
void*   RXxBuffer;
void*   TXBuffer;
bool    IsdataRead = false;

FILE *fp;
/* ====================================================*/


const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;


int     CreateSocket(const char *addr, int port);
int     CreateSocket_new(const char *addr, int port);
int     Connect2Server();
int     Connect2ServerTraj();
void*   LANThread(void*);


//subscriber
ros::Subscriber joint_states_sub;
const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};

//for topic subscriber
ros::Subscriber inter_joint_states_sub;
sensor_msgs::JointState inter_joint_states;

//Encoder Feedback publisher
ros::Publisher encoder_feedback_pub;
sensor_msgs::JointState encoder_joint_states;



/* CB for inter_joint_state(interpolated joint_states) */
void inter_joint_states_callback(const sensor_msgs::JointState& inter_joint_state_msg){
    for(int i=0; i< NUM_JOINTS; i++){
        TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = CONTROL_ON;
        TXData.ros2podo_data.Arm_action.joint[i].reference = inter_joint_state_msg.position[i];
    }
    TXData.ros2podo_data.CMD_JOINT = MODE_JOINT_PUBLISH;
    write(sock, &TXData, TXDataSize);
}

/*clear TXBuffer to default values*/
void clearTXBuffer()
{
    //reset command
    TXData.ros2podo_data.CMD_JOINT   = static_cast<JOINTMOVE_CMD>(0);
    TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(0);
    TXData.ros2podo_data.CMD_WHEEL   = static_cast<WHEELMOVE_CMD>(0);


    //reset action data
    TXData.ros2podo_data.Arm_action.result_flag     = 0;
    TXData.ros2podo_data.Base_action.result_flag    = 0;
    TXData.ros2podo_data.Gripper_action.result_flag = 0;

    for(int i = 0; i < NUM_JOINTS; i++)
    {
        //TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = 0;
    }

    for(int i = 0; i < NUM_PARTS;  i++)
    {
        //TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_movepos = 0;
        //TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_moveori = 0;
    }
}

/*clear RXBuffer to default values*/
void clearRXBuffer()
{
    //std::lock_guard<std::mutex> lock(variable_lock);
    //reset done flag
    RXData.podo2ros_data.Arm_feedback.result_flag  = 0;
    RXData.podo2ros_data.Base_feedback.result_flag = 0;
}


/*========================== Start of Base Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_BaseAction
{
protected:

    ros::NodeHandle nh_base;
    actionlib::SimpleActionServer<ros_podo_connector::RosPODO_BaseAction> asBase_;
    std::string action_name_;

    bool baseMotionSuccess = false;
    bool motionStarted = false;

    // create messages that are used to published feedback&result
    ros_podo_connector::RosPODO_BaseFeedback feedback_;
    ros_podo_connector::RosPODO_BaseResult result_;


public:

    RosPODO_BaseAction(std::string name) :
        asBase_(nh_base, name, boost::bind(&RosPODO_BaseAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asBase_.start();
    }

    ~RosPODO_BaseAction(void)
    {

    }

  
  

/*Call Back function when goal is received from Action client*/
  void executeCB(const ros_podo_connector::RosPODO_BaseGoalConstPtr &goal)
  {

      // loop this thread to check status of goal
      ros::Rate r(200);
      baseMotionSuccess = false;

      //=====execute action for robot motion========
      TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(goal->wheelmove_cmd);
      TXData.ros2podo_data.Base_action.wheel.MoveX = goal->MoveX;
      TXData.ros2podo_data.Base_action.wheel.MoveY = goal->MoveY;
      TXData.ros2podo_data.Base_action.wheel.ThetaDeg = goal->ThetaDeg;
      
      
      TXData.ros2podo_data.Base_action.wheel.VelX = goal->VelX;
      TXData.ros2podo_data.Base_action.wheel.VelY = goal->VelY;
      TXData.ros2podo_data.Base_action.wheel.VelTheta = goal->VelTheta;
      //ROS_INFO("%f\t%f\t%f\n",goal->VelX,goal->VelY,goal->VelTheta);
      write(sock, &TXData, TXDataSize);

      ROS_INFO("CMD Grip: %i, Base: %i, Arm: %i, \n",TXData.ros2podo_data.CMD_GRIPPER,  TXData.ros2podo_data.CMD_WHEEL, TXData.ros2podo_data.CMD_JOINT);
    
    
	  //multi request base move (velocity-based)
      if(TXData.ros2podo_data.CMD_WHEEL == WHEEL_MOVE_VELOCITY) 
      {
		  ; //dont wait for result
	  }
	  
	  //single request base move (position-based) 
	  else 
	  {
		  //while loop to check until goal is finished
		  while(baseMotionSuccess == false)
		  {
			  r.sleep();
		  }
	  }

      asBase_.setSucceeded(result_);
  }

  
  /* update feedback action topic*/
  void publishFeedback()
  {
      static int loopCounter = 0;

      if(loopCounter >= 20) //each counter 5msec ==> 10Hz
      {
          feedback_.MoveX = RXData.podo2ros_data.Base_feedback.wheel.MoveX;
          feedback_.MoveY = RXData.podo2ros_data.Base_feedback.wheel.MoveY;
          feedback_.ThetaDeg = RXData.podo2ros_data.Base_feedback.wheel.ThetaDeg;
          asBase_.publishFeedback(feedback_);
          loopCounter = 0;
      }

      loopCounter++;

  }
  
  /* clear flags upon finishing action */
  void clearTXFlag()
  {
      //reset command
      TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(0);
      //reset data
      TXData.ros2podo_data.Base_action.wheel.MoveX = 0;
      TXData.ros2podo_data.Base_action.wheel.MoveY = 0;
      TXData.ros2podo_data.Base_action.wheel.ThetaDeg = 0;
      TXData.ros2podo_data.Base_action.wheel.VelX = 0;
      TXData.ros2podo_data.Base_action.wheel.VelY = 0;
      TXData.ros2podo_data.Base_action.wheel.VelTheta = 0;
      TXData.ros2podo_data.Base_action.result_flag = 0;

  }
  
  /* update result action topic*/
  void publishResult()
  {
	  
	  static int motionStartedTick = 0;
	  //wait for RX flag update 1 sec
	  if(motionStartedTick > 200)  {motionStarted = true;}
	  motionStartedTick++;
		  
	  if(motionStarted == true && RXData.podo2ros_data.Base_feedback.result_flag == true )
	  {
		  //write result
		  result_.MoveX = feedback_.MoveX;
		  result_.MoveY = feedback_.MoveY;
		  result_.ThetaDeg = feedback_.ThetaDeg;
		  result_.result_flag = 1;
		  ROS_INFO("Finished base action: %i\n", result_.result_flag );
		  ROS_INFO("base result inside: %i\n", RXData.podo2ros_data.Base_feedback.result_flag);
		  //reset flags
		  clearTXFlag();
		  baseMotionSuccess = true;
		  motionStarted = false;
		  motionStartedTick = 0;
	  }
	  

      
  }

  //1 if alive, 0 else
  int returnServerStatus()
  {
      if(asBase_.isActive()) { return 1; }
      else { return 0; }
  }

};
/*========================== End of Base Action Server ==================================*/


/*========================== Start of Arm Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_ArmAction
{
protected:

    ros::NodeHandle nh_arm;
    actionlib::SimpleActionServer<ros_podo_connector::RosPODO_ArmAction> asArm_;
    std::string action_name_;
    bool armMotionSuccess = false;
    bool motionStarted = false;

    // create messages that are used to published feedback&result
    ros_podo_connector::RosPODO_ArmFeedback feedback_;
    ros_podo_connector::RosPODO_ArmResult result_;

public:

    RosPODO_ArmAction(std::string name) :
        asArm_(nh_arm, name, boost::bind(&RosPODO_ArmAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asArm_.start();
    }

    ~RosPODO_ArmAction(void)
    {

    }

    /*Call Back function when goal is received from Action client*/
    void executeCB(const ros_podo_connector::RosPODO_ArmGoalConstPtr &goal)
    {

        // helper variables
        ros::Rate r(200);
        armMotionSuccess = false;

        //=====execute action for robot motion========
        TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(goal->jointmove_cmd);

        //Map the arm move .action to TXData for cases {publish joint, joint, wbik, estop, save}
        switch(static_cast<JOINTMOVE_CMD>(goal->jointmove_cmd))
        {
        case MODE_JOINT_PUBLISH:
        {
            //ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            for(int i = 0; i < NUM_JOINTS; i++)
            {
                TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = goal->joint_ref[i].OnOffControl;
                TXData.ros2podo_data.Arm_action.joint[i].reference = goal->joint_ref[i].reference;
            }
            armMotionSuccess = true;
            static int cnt = 0;
            std::cout << "Action CB cnt: " << cnt++ << std::endl;
            break;
        }

        case MODE_MOVE_JOINT:
        {
            //ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            for(int i = 0; i < NUM_JOINTS; i++)
            {
                TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = goal->joint_ref[i].OnOffControl;
                TXData.ros2podo_data.Arm_action.joint[i].reference = goal->joint_ref[i].reference;
                TXData.ros2podo_data.Arm_action.joint[i].GoalmsTime = goal->joint_ref[i].GoalmsTime;
            }
            break;
        }

        case MODE_SET_WBIK:
        {
            //ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            for(int i = 0; i < NUM_PARTS;  i++)
            {
                TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_movepos = goal->wbik_ref[i].OnOff_position;
                TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_moveori = goal->wbik_ref[i].OnOff_orientation;
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[0] = goal->wbik_ref[i].goal_position[0];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[1] = goal->wbik_ref[i].goal_position[1];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[2] = goal->wbik_ref[i].goal_position[2];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[0] = goal->wbik_ref[i].goal_orientation[0];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[1] = goal->wbik_ref[i].goal_orientation[1];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[2] = goal->wbik_ref[i].goal_orientation[2];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[3] = goal->wbik_ref[i].goal_orientation[3];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_angle = goal->wbik_ref[i].goal_angle;
                TXData.ros2podo_data.Arm_action.wbik[i].GoalmsTime = goal->wbik_ref[i].GoalmsTime;
            }            
            break;
        }

        case MODE_E_STOP:
        {
            //ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            break;
        }

        case MODE_SAVE:
        {
            //ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            break;
        }
        case 7:
        {
			//ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
            for(int i = 0; i < NUM_PARTS;  i++)
            {
                TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_movepos = goal->wbik_ref[i].OnOff_position;
                TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_moveori = goal->wbik_ref[i].OnOff_orientation;
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[0] = goal->wbik_ref[i].goal_position[0];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[1] = goal->wbik_ref[i].goal_position[1];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[2] = goal->wbik_ref[i].goal_position[2];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[0] = goal->wbik_ref[i].goal_orientation[0];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[1] = goal->wbik_ref[i].goal_orientation[1];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[2] = goal->wbik_ref[i].goal_orientation[2];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[3] = goal->wbik_ref[i].goal_orientation[3];
                TXData.ros2podo_data.Arm_action.wbik[i].Goal_angle = goal->wbik_ref[i].goal_angle;
                TXData.ros2podo_data.Arm_action.wbik[i].GoalmsTime = goal->wbik_ref[i].GoalmsTime;
            }            
            break;
		}
        
        
        }


        //write TXdata
        write(sock, &TXData, TXDataSize);
        ROS_INFO("CMD Grip: %i, Base: %i, Arm: %i, \n",TXData.ros2podo_data.CMD_GRIPPER,  TXData.ros2podo_data.CMD_WHEEL, TXData.ros2podo_data.CMD_JOINT);

        //while loop to check until goal is finished
        while(armMotionSuccess == false)
        {
            r.sleep();
        }
        //ROS_INFO("arm action done: %i\n", armMotionSuccess);
        asArm_.setSucceeded(result_);
    }
       
    /* clear flags upon finishing action */
    void clearTXFlag()
    {

        //reset command
        TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);

        //reset action data
        TXData.ros2podo_data.Arm_action.result_flag = 0;

        for(int i = 0; i < NUM_JOINTS; i++)
        {
            TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = 0;
        }

        for(int i = 0; i < NUM_PARTS;  i++)
        {
            TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_movepos = 0;
            TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_moveori = 0;
        }
    }

    /* update feedback action topic*/
    void publishFeedback()
    {
        static int loopCounter = 0;

        if(loopCounter >= 20) //each counter 5msec ==> 10Hz
        {

            //arm joint feedback
            for(int i = 0; i < NUM_JOINTS; i++)
            {
                feedback_.joint_ref[i].OnOffControl = RXData.podo2ros_data.Arm_feedback.joint[i].ONOFF_control;
                feedback_.joint_ref[i].reference = RXData.podo2ros_data.Arm_feedback.joint[i].reference;
                feedback_.joint_ref[i].GoalmsTime = RXData.podo2ros_data.Arm_feedback.joint[i].GoalmsTime;

            }

            //arm wbik feedback
            for(int i = 0; i < NUM_PARTS;  i++)
            {
                feedback_.wbik_ref[i].OnOff_position = RXData.podo2ros_data.Arm_feedback.wbik[i].ONOFF_movepos;
                feedback_.wbik_ref[i].OnOff_orientation = RXData.podo2ros_data.Arm_feedback.wbik[i].ONOFF_moveori;
                feedback_.wbik_ref[i].goal_position[0] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[0];
                feedback_.wbik_ref[i].goal_position[1] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[1];
                feedback_.wbik_ref[i].goal_position[2] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[2];
                feedback_.wbik_ref[i].goal_orientation[0] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[0];
                feedback_.wbik_ref[i].goal_orientation[1] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[1];
                feedback_.wbik_ref[i].goal_orientation[2] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[2];
                feedback_.wbik_ref[i].goal_orientation[3] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[3];
                feedback_.wbik_ref[i].goal_angle = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_angle;
                feedback_.wbik_ref[i].GoalmsTime = RXData.podo2ros_data.Arm_feedback.wbik[i].GoalmsTime;
            }

            //publish feedback
            asArm_.publishFeedback(feedback_);
        }

        loopCounter++;
    }

    /* update result action topic*/
    void publishResult()
    {

        static int motionStartedTick = 0;

        //wait for RX flag update 2 sec
        if(motionStartedTick > 400)  {motionStarted = true;}
        motionStartedTick++;

        //received done flag from PODO
        if(motionStarted == true && RXData.podo2ros_data.Arm_feedback.result_flag)
        {
            //arm joint feedback
            for(int i = 0; i < NUM_JOINTS; i++)
            {
                result_.joint_ref[i].OnOffControl = feedback_.joint_ref[i].OnOffControl;
                result_.joint_ref[i].reference = feedback_.joint_ref[i].reference;
                result_.joint_ref[i].GoalmsTime = feedback_.joint_ref[i].GoalmsTime;
            }

            //arm wbik feedback
            for(int i = 0; i < NUM_PARTS;  i++)
            {
                result_.wbik_ref[i].OnOff_position = feedback_.wbik_ref[i].OnOff_position;
                result_.wbik_ref[i].OnOff_orientation = feedback_.wbik_ref[i].OnOff_orientation;
                result_.wbik_ref[i].goal_position[0] = feedback_.wbik_ref[i].goal_position[0];
                result_.wbik_ref[i].goal_position[1] = feedback_.wbik_ref[i].goal_position[1];
                result_.wbik_ref[i].goal_position[2] = feedback_.wbik_ref[i].goal_position[2];
                result_.wbik_ref[i].goal_orientation[0] = feedback_.wbik_ref[i].goal_orientation[0];
                result_.wbik_ref[i].goal_orientation[1] = feedback_.wbik_ref[i].goal_orientation[1];
                result_.wbik_ref[i].goal_orientation[2] = feedback_.wbik_ref[i].goal_orientation[2];
                result_.wbik_ref[i].goal_orientation[3] = feedback_.wbik_ref[i].goal_orientation[3];
                result_.wbik_ref[i].goal_angle = feedback_.wbik_ref[i].goal_angle;
                result_.wbik_ref[i].GoalmsTime = feedback_.wbik_ref[i].GoalmsTime;
            }
            
            result_.result_flag = 1;
            ROS_INFO("Finished arm action: %i\n", result_.result_flag );
            clearTXFlag();
            armMotionSuccess = true;
            motionStarted = false;
            motionStartedTick = 0;
        }
    }
  
    //1 if alive, 0 else
    int returnServerStatus()
    {
        if(asArm_.isActive()) { return 1; }
        else { return 0; }
    }

};
/*========================== End of Arm Action Server ==================================*/

/*========================== Start of Gripper Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_GripperAction
{
protected:

    ros::NodeHandle nh_gripper;
    actionlib::SimpleActionServer<ros_podo_connector::RosPODO_GripperAction> asGripper_;
    std::string action_name_;
    bool motionStarted = false;
    bool gripperMotionSuccess = false;

    // create messages that are used to published feedback&result
    ros_podo_connector::RosPODO_GripperFeedback feedback_;
    ros_podo_connector::RosPODO_GripperResult result_;


public:

    RosPODO_GripperAction(std::string name) :
        asGripper_(nh_gripper, name, boost::bind(&RosPODO_GripperAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asGripper_.start();
    }

    ~RosPODO_GripperAction(void)
    {
    }

    /*Call Back function when goal is received from Action client*/
    void executeCB(const ros_podo_connector::RosPODO_GripperGoalConstPtr &goal)
    {

        // loop this thread to check status of goal
        ros::Rate r(200);
        gripperMotionSuccess = false;

        //=====execute action for robot motion========
        TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(goal->grippermove_cmd);
        TXData.ros2podo_data.Gripper_action.side = goal->mode;

//        std::cout << "goal->gripper = " << goal->grippermove_cmd << std::endl;

        write(sock, &TXData, TXDataSize);
        ROS_INFO("CMD Grip: %i, Base: %i, Arm: %i, \n",TXData.ros2podo_data.CMD_GRIPPER,  TXData.ros2podo_data.CMD_WHEEL, TXData.ros2podo_data.CMD_JOINT);

        //while loop to check until goal is finished
        while(gripperMotionSuccess == false)
        {
            r.sleep();
        }
        //ROS_INFO("gripper action done %i\n", gripperMotionSuccess);
        asGripper_.setSucceeded(result_);
    }

    /* clear flags upon finishing action */
    void clearTXFlag()
    {
        //reset command
        TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(0);
        TXData.ros2podo_data.Gripper_action.result_flag = 0;
    }

    /* update feedback action topic*/
    void publishFeedback()
    {

    }

    /* update result action topic*/
    void publishResult()
    {
        static int motionStartedTick = 0;
        //wait for RX flag update 1 sec
        if(motionStartedTick > 200)  {motionStarted = true;}
        motionStartedTick++;

        if(motionStarted == true && RXData.podo2ros_data.Gripper_feedback.result_flag == true )
        {
            result_.result_flag = 1;
            ROS_INFO("Finished Gripper action: %i\n", result_.result_flag );
            clearTXFlag();
            gripperMotionSuccess = true;
            motionStarted = false;
            motionStartedTick = 0;
        }
    }

    //1 if alive, 0 else
    int returnServerStatus()
    {
        if(asGripper_.isActive()) { return 1; }
        else { return 0; }
    }
};
/*========================== End of Gripper Action Server ==================================*/



/*========================== Start of Traj Action Server ==================================*/
/*class RosPODO_TrajAction
{
protected:

    ros::NodeHandle nh_traj;
    actionlib::SimpleActionServer<ros_podo_connector::RosPODO_TrajAction> asTraj_;
    std::string action_name_;

    bool TrajMotionSuccess = false;
    bool motionStarted = false;

    // create messages for publishing feedback&result
    ros_podo_connector::RosPODO_TrajFeedback feedback_;
    ros_podo_connector::RosPODO_TrajResult result_;

    int max_path = 100;

public:

    RosPODO_TrajAction(std::string name) :
        asTraj_(nh_traj, name, boost::bind(&RosPODO_TrajAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asTraj_.start();
    }

    ~RosPODO_TrajAction(void)
    {

    }

    enum TRAJECTORY_CMD
    {
        MOVE_ABSOLUTE = 0,
        MOVE_RELATIVE
    };

    void executeCB(const ros_podo_connector::RosPODO_TrajGoalConstPtr &goal)
    {


        std::cout << "traj Action Recieved." << std::endl;

        // MOVEIT INTERFACE //
        // Setup
        // static const std::string PLANNING_GROUP = "L_arm";
        const std::string PLANNING_GROUP = goal->planGroup;
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        // Raw pointers are frequently used to refer to the planning group for improved performance.
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

        // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations

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
        switch(goal->traj_cmd)
        {
            case MOVE_ABSOLUTE:
            {
                std::cout << std::endl << "Mode: MOVE ABSOLUTE" << std::endl;
                target_pose1.position.x = goal->x;
                target_pose1.position.y = goal->y;
                target_pose1.position.z = goal->z;
                target_pose1.orientation.w = goal->ori_w;
                target_pose1.orientation.x = goal->ori_x;
                target_pose1.orientation.y = goal->ori_y;
                target_pose1.orientation.z = goal->ori_z;
                break;
            }
            case MOVE_RELATIVE:
            {
                std::cout << std::endl << "Mode: MOVE RELATIVE" << std::endl;
                target_pose1.position.x = move_group.getCurrentPose().pose.position.x + goal->x;
                target_pose1.position.y = move_group.getCurrentPose().pose.position.y + goal->y;
                target_pose1.position.z = move_group.getCurrentPose().pose.position.z + goal->z;
                target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w + goal->ori_w;
                target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x + goal->ori_x;
                target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y + goal->ori_y;
                target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z + goal->ori_z;

                break;
            }
        }

        std::cout << "PLANNING GROUP: " << goal->planGroup << std::endl;
        std::cout << "Goal Position x: " << target_pose1.position.x ;
        std::cout << ", y: " << target_pose1.position.y ;
        std::cout << ", z: " << target_pose1.position.z << std::endl;
        std::cout << "Goal orientation ori_w: " << target_pose1.orientation.w ;
        std::cout << ", ori_x: " << target_pose1.orientation.x ;
        std::cout << ", ori_y: " << target_pose1.orientation.y ;
        std::cout << ", ori_z: " << target_pose1.orientation.z << std::endl;

        move_group.setPoseTarget(target_pose1);
        move_group.setPlannerId("RRT");

        // Now, we call the planner to compute the plan and visualize it. // Note that we are just planning,
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
        ROS_INFO_NAMED("moveo", "Visualizing plan 1 (pose goal) %s", success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "" : "FAILED");

        // Visualizing plans
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
        visual_tools.publishAxisLabeled(target_pose1, "Goal Pose");
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); //TRAJECTORY*******************************************************

        visual_tools.trigger();

        //Trajectory Planning
        moveit_msgs::RobotTrajectory msg = my_plan.trajectory_;
        std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points;
        std::vector<int>::size_type size1 = msg.joint_trajectory.points.size();

        if(success && size1 < max_path)
        {

            move_group.move();
            //Debug
            std::cout << std::endl;
            std::cout << "Trajectory points: " << msg.joint_trajectory.points.size() << std::endl;
            std::cout << "Num of joints in planning group: " << msg.joint_trajectory.joint_names.size() << std::endl;

            ROS_JOINTREF Traj_action[msg.joint_trajectory.points.size()];

            for(int p=0; p<size1; p++){    //p: points of Trajectory
                for(int robot_j=0; robot_j< NUM_JOINTS; robot_j++)    //robot_j: roboto joints
                {
                    for(int traj_j=0; traj_j< msg.joint_trajectory.joint_names.size(); traj_j++)     // traj_j: trajectory joints
                    {
                        //only if joint name matches
                        if(JointBufferNameList[robot_j] == msg.joint_trajectory.joint_names[traj_j])
                        {
                            Traj_action[p].joint[robot_j].ONOFF_control = CONTROL_ON;
                            Traj_action[p].joint[robot_j].reference = msg.joint_trajectory.points[p].positions[traj_j];
                            if(p > 0)
                                Traj_action[p].joint[robot_j].GoalmsTime = (msg.joint_trajectory.points[p].time_from_start.toSec() - msg.joint_trajectory.points[p-1].time_from_start.toSec());
                            else
                                Traj_action[p].joint[robot_j].GoalmsTime = 0.;
                            break;
                        }
                        else {
                            Traj_action[p].joint[robot_j].ONOFF_control = CONTROL_OFF;
                            Traj_action[p].joint[robot_j].reference = 0.;
                            Traj_action[p].joint[robot_j].GoalmsTime = 0.;
                        }
                    }
                }
            }
            //WRITE TRAJECTORY TO PODO (Port 7000)
            write(sock_traj, &Traj_action, sizeof(Traj_action));

            //WRITE TO PODO (Port 5500; Send MODE CMD)
            TXData.ros2podo_data.CMD_JOINT = MODE_TRAJECTORY;
            write(sock, &TXData, TXDataSize);
            asTraj_.setSucceeded(result_);
        }
        else {
            visual_tools.publishText(text_pose, "PLAN failed", rvt::WHITE, rvt::XLARGE);

            if(size1 > max_path)
                std::cout << "Path size bigger than maximum size" << std::endl;
            else
                std::cout << "Plan Failed" << std::endl;

             //terminal status
            asTraj_.setAborted(result_);
        }

        //debugging
        asTraj_.setSucceeded(result_);
    }
};*/
/*========================== End of Traj Action Server ==================================*/

/*========================== Start of Trajectory Action Server ==================================*/
class RosPODO_TrajectoryAction
{
protected:

    ros::NodeHandle nh_trajectory;
    actionlib::SimpleActionServer<ros_podo_connector::RosPODO_TrajectoryAction> asTrajectory_;
    std::string action_name_;

    // create messages for publishing feedback&result
    ros_podo_connector::RosPODO_TrajectoryFeedback feedback_;
    ros_podo_connector::RosPODO_TrajectoryResult result_;

    //Result flag from PODO
    bool trajectorySuccess = false;
    bool motionStarted = false;

public:

    RosPODO_TrajectoryAction(std::string name) :
        asTrajectory_(nh_trajectory, name, boost::bind(&RosPODO_TrajectoryAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asTrajectory_.start();
    }

    ~RosPODO_TrajectoryAction(void)
    {

    }

    enum TRAJECTORY_CMD
    {
        MOVE_ABSOLUTE = 0,
        MOVE_RELATIVE
    };

    void executeCB(const ros_podo_connector::RosPODO_TrajectoryGoalConstPtr &goal)
    {
//        std::cout << "trajectory Action Recieved." << std::endl;

        ROS_JOINTREF Traj_action[goal->num_points];

        ros::Rate r(200);
        trajectorySuccess = false;

        for(int p=0; p<goal->num_points; p++){    //p: points of Trajectory
            for(int robot_j=0; robot_j< NUM_JOINTS; robot_j++)    //robot_j: robot joints
            {
                Traj_action[p].joint[robot_j].reference     = goal->via_point[p].joint[robot_j].reference;
                Traj_action[p].joint[robot_j].GoalmsTime    = goal->via_point[p].joint[robot_j].GoalmsTime;
                Traj_action[p].joint[robot_j].ONOFF_control = goal->via_point[p].joint[robot_j].OnOffControl;
            }

            if(p==0)
                Traj_action[p].StartFlag = true;
            else
                Traj_action[p].StartFlag = false;

            if(p==goal->num_points-1)
                Traj_action[p].DoneFlag = true;
            else
                Traj_action[p].DoneFlag = false;
        }


        //WRITE TRAJECTORY TO PODO (Port 7000)
        write(sock_traj, &Traj_action, sizeof(Traj_action));
        //ROS_ERROR("size: %f", sizeof(Traj_action));

        //WRITE TO PODO (Port 5500; Send MODE CMD)
        TXData.ros2podo_data.CMD_JOINT = MODE_TRAJECTORY;
        write(sock, &TXData, TXDataSize);

        //while loop to check until goal is finished
        while(trajectorySuccess == false)
        {
            r.sleep();
        }

        asTrajectory_.setSucceeded(result_);
    }
    
    /* clear flags upon finishing action */
    void clearTXFlag()
    {
        //reset command
        TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);

        //reset action data
        TXData.ros2podo_data.Arm_action.result_flag = 0;
    }


    /* update result action topic*/
    void publishResult()
    {
        static int motionStartedTick = 0;
        //wait for RX flag update 1 sec
        if(motionStartedTick > 200)  {motionStarted = true;}
        motionStartedTick++;

        if(motionStarted == true && RXData.podo2ros_data.Arm_feedback.result_flag == true)
        {
            result_.result_flag = 1;
            ROS_INFO("Finished Trajectory: %i\n", result_.result_flag );
            motionStarted = false;
            trajectorySuccess = true;
            
            //reset command
            TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);

            //reset action data
            TXData.ros2podo_data.Arm_action.result_flag = 0;
			
            motionStartedTick = 0;
        }
    }

    //1 if alive, 0 else
    int returnServerStatus()
    {
        if(asTrajectory_.isActive()) { return 1; }
        else { return 0; }
    }
};
/*========================== End of Trajectory Action Server ==================================*/



/*Print helpful summary about this code */
void printInitialInfo()
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Starting ROS2PODO Motion Action Server" << std::endl << std::endl;
    std::cout << "   Developer: Heo Yujin" << std::endl;
    std::cout << "   Developer: Cho Saihim" << std::endl;
    std::cout << "   E-mail   : blike4@kaist.ac.kr" << std::endl;
    std::cout << "   E-mail   : saihimcho@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;
}

/*========================== LAN Communication Functions =============================*/
/*Load IP txt file for TCP/IP TX/RX */
/*Create Socket and initialize TX/RX size */
int initializeSocket()
{
    FILE *fpNet = NULL;
    fpNet = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }
    

    if(CreateSocket(ip, PODO_PORT)){
        ROS_INFO("Created Socket..");

        RXDataSize = sizeof(LAN_PODO2ROS);
        TXDataSize = sizeof(LAN_ROS2PODO);
        RXBuffer = (void*)malloc(RXDataSize);
        RXxBuffer = (void*)malloc(sizeof(int));
        TXBuffer = (void*)malloc(TXDataSize);
        
        /*
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0){
            ROS_ERROR("Create Thread Error..");
            return 0;
        }*/
    }
    else{
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node..");
        return 0;
    }
    
}


int CreateSocket(const char *addr, int port){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }
    server.sin_addr.s_addr = inet_addr(addr);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int CreateSocket_new(const char *addr, int port){
    sock_traj = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_traj == -1){
        return false;
    }
    server_traj.sin_addr.s_addr = inet_addr(addr);
    server_traj.sin_family = AF_INET;
    server_traj.sin_port = htons(port);

    printf("CreateSocket_new\n");
    int optval = 1;
    setsockopt(sock_traj, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock_traj, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int Connect2Server()
{
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        std::cout << " Connection Failed" << std::endl;
        return false;
    }
    std::cout << "Client connect to server!! (PODO_CONNECTOR)" << std::endl;
    return true;
}

int Connect2ServerTraj()
{
    if(connect(sock_traj, (struct sockaddr*)&server_traj, sizeof(server_traj)) < 0)
    {
        std::cout << " Connection Failed TARJ" << std::endl;
        return false;
    }
    std::cout << "Client connect to server_traj!! (PODO_CONNECTOR_TRAJ)" << std::endl;
    return true;
}

void LANthread_update()
{

    static unsigned int tcp_status = 0x00;
    static unsigned int tcp_statustraj = 0x00;
    static int tcp_size = 0;
    static int tcp_size_traj = 0;
    static int connectCnt = 0;
    static int connectCntTraj = 0;

    if(threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(sock == 0){
                CreateSocket(ip, PODO_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    //std::cout << "Connect to Server Failed.." << std::endl;
                    connectCnt++;
            }
            //usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            tcp_size = read(sock, RXBuffer, RXDataSize);
            if(tcp_size == RXDataSize){
                memcpy(&RXData, RXBuffer, RXDataSize);
            }

            if(tcp_size == 0){
                tcp_status = 0x00;
                connectionStatus = false;
                close(sock);
                sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
        if(tcp_statustraj == 0x00){
            // If client was not connected
            if(sock_traj == 0){
                CreateSocket_new(ip, PODO_TRAJ_PORT);
            }
            if(Connect2ServerTraj()){
                tcp_statustraj = 0x01;
                connectCntTraj = 0;
            }else{
                if(connectCntTraj%10 == 0)
//                    std::cout << "Connect to Server Failed Traj.." << std::endl;
                    connectCntTraj++;
            }
            //usleep(1000*1000);
        }
        if(tcp_statustraj == 0x01){
            // If client was connected
            tcp_size_traj = read(sock_traj, RXxBuffer, sizeof(int));
            if(tcp_size_traj == 0){
                tcp_statustraj = 0x00;
                close(sock_traj);
                sock_traj = 0;
                std::cout << "Socket Disconnected Traj.." << std::endl;
            }
        }
    }

}
/*========================== End of LAN functions ==================================*/


/*========================== main while loop ==================================*/
int main(int argc, char **argv)
{
    printInitialInfo();
    //Initialize ROS node
    ros::init(argc, argv, "ros_podo_connector");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);


    inter_joint_states_sub = n.subscribe("inter_joint_states", 100, inter_joint_states_callback);
    encoder_feedback_pub = n.advertise<sensor_msgs::JointState>("encoder_joint_states",1);

    //initialize
    encoder_joint_states.name.resize(NUM_JOINTS+2); //+2:hands
    encoder_joint_states.position.resize(NUM_JOINTS+2);
    for(int i; i < NUM_JOINTS; i++) // joint except hands
        encoder_joint_states.name[i] = JointBufferNameList[i];
    //Hand
    encoder_joint_states.name[NUM_JOINTS] = "RHAND";
    encoder_joint_states.name[NUM_JOINTS+1] = "LHAND";

    // Create Socket
    initializeSocket();

    //Initialize ROS Action Server for TX to PODO
    RosPODO_BaseAction        rospodo_base("rospodo_base");
    RosPODO_ArmAction         rospodo_arm("rospodo_arm");
    RosPODO_GripperAction     rospodo_gripper("rospodo_gripper");
    //RosPODO_TrajAction        rospodo_Traj("rospodo_traj");
    RosPODO_TrajectoryAction  rospodo_Trajectory("rospodo_trajectory");
    ROS_INFO("Starting ROS2PODO Action Servers");




    /* === main while loop to RX feedback calls at regular periods === */
    while(ros::ok())
    {
        //update RX values from PODO
        LANthread_update();

        //Encoder Feedback Topic
        for(int i=0; i < NUM_JOINTS; i++)
            encoder_joint_states.position[i] = RXData.podo2ros_data.Arm_feedback.joint[i].reference;

        encoder_joint_states.position[NUM_JOINTS+0] = RXData.JointEncoder[26];
        encoder_joint_states.position[NUM_JOINTS+1] = RXData.JointEncoder[28];

        encoder_feedback_pub.publish(encoder_joint_states);


        //check if action server is active
        if(rospodo_base.returnServerStatus())
        {
            rospodo_base.publishFeedback();
            rospodo_base.publishResult();
        }

        if(rospodo_arm.returnServerStatus())
        {
            rospodo_arm.publishFeedback();
            rospodo_arm.publishResult();
        }

        if(rospodo_gripper.returnServerStatus())
        {
            rospodo_gripper.publishResult();
        }

        if(rospodo_Trajectory.returnServerStatus())
        {
            rospodo_Trajectory.publishResult();
        }

        ros::spinOnce();

        //loop at desired rate
        loop_rate.sleep();
    }

    return 0;
}
/*========================== End of Code ==================================*/
