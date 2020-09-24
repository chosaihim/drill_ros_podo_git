
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
 
 * 
 * E-mail : blike4@kaist.ac.kr    (Heo Yujin)
 * E-mail : chosaihim@kaist.ac.kr (Cho Saihim)
 *
 * Versions :
 * v1.0.2019.05
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

/*for multi-threaded locks */
//#include <mutex>
//std::mutex variable_lock;


/* =============TCP/IP  global variables initialize ============= */
#define PODO_ADDR       "127.0.0.1"
#define PODO_PORT       5000

char ip[20];
int sock = 0;
struct sockaddr_in  server;

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
void*   TXBuffer;
bool    IsdataRead = false;

FILE *fp;
/* ====================================================*/


const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void*);


//subscriber
ros::Subscriber joint_states_sub;
const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};



/*clear TXBuffer to default values*/
void clearTXBuffer()
{
    //std::lock_guard<std::mutex> lock(variable_lock);
    //reset command
    TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);
    TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(0);
    TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(0);


    //reset action data
    //TXData.ros2podo_data.Arm_action.joint[NUM_JOINTS]
    //TXData.ros2podo_data.Arm_action.wbik[NUM_PARTS]
    TXData.ros2podo_data.Arm_action.result_flag = 0;

    //TXData.ros2podo_data.Base_action.wheel.MoveX = 0;
    //TXData.ros2podo_data.Base_action.wheel.MoveY = 0;
    //TXData.ros2podo_data.Base_action.wheel.ThetaDeg = 0;
    TXData.ros2podo_data.Base_action.result_flag = 0;

    //TXData.ros2podo_data.Gripper_action.mode = 0;
    //TXData.ros2podo_data. = static_cast<GRIPPER_PARAMETER>(0);
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
    RXData.podo2ros_data.Arm_feedback.result_flag = 0;
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

      // publish info to the console for the user
      //ROS_INFO("%s: 1 Received Base Motion with Command: %i\n", action_name_.c_str(), goal->wheelmove_cmd);

      //=====execute action for robot motion========

      //write TX to podo

      TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(goal->wheelmove_cmd);
      TXData.ros2podo_data.Base_action.wheel.MoveX = goal->MoveX;
      TXData.ros2podo_data.Base_action.wheel.MoveY = goal->MoveY;
      TXData.ros2podo_data.Base_action.wheel.ThetaDeg = goal->ThetaDeg;
      write(sock, &TXData, TXDataSize);

      ROS_INFO("CMD Grip: %i, Base: %i, Arm: %i, \n",TXData.ros2podo_data.CMD_GRIPPER,  TXData.ros2podo_data.CMD_WHEEL, TXData.ros2podo_data.CMD_JOINT);


      //while loop to check until goal is finished
      while(baseMotionSuccess == false)
      {
          r.sleep();

      }


      //ROS_INFO("base action done: %i\n", baseMotionSuccess);
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

        //debugging
        static int cnt_fromClient = 0;
        cnt_fromClient++;
        if(cnt_fromClient > 2000)
        std::cout << "count from Client: " << cnt_fromClient << std::endl;

        asArm_.setSucceeded(result_);

//        static int cnt_CB = 0;
//        std::cout << "count for callback: " << cnt_CB++ << std::endl;

        /*
        ros::Time tzero(0);
        ros::Time beginTime = ros::Time::now();
        double beginTime_d = ros::Time::now().toSec();
        double nowNow;


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
        //clearTXBuffer();

        */


//        nowNow = ros::Time::now().toSec();
//        std::cout << "CB took " << (nowNow - beginTime_d) *1000 << " msecs." << std::endl;

    }
  

     
    /* clear flags upon finishing action */
    void clearTXFlag()
    {

        //reset command
        TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);

        //reset action data
        //TXData.ros2podo_data.Arm_action.joint[NUM_JOINTS]
        //TXData.ros2podo_data.Arm_action.wbik[NUM_PARTS]
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

            //ADD JOINT PUBLISH


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
        // publish info to the console for the user
        //ROS_INFO("%s: 1 Received Gripper Motion with Command: %i\n", action_name_.c_str(), goal->grippermove_cmd);

        //=====execute action for robot motion========

        //write TX to podo

        TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(goal->grippermove_cmd);
        TXData.ros2podo_data.Gripper_action.mode = goal->mode;

        write(sock, &TXData, TXDataSize);
        ROS_INFO("CMD Grip: %i, Base: %i, Arm: %i, \n",TXData.ros2podo_data.CMD_GRIPPER,  TXData.ros2podo_data.CMD_WHEEL, TXData.ros2podo_data.CMD_JOINT);


        //while loop to check until goal is finished
        while(gripperMotionSuccess == false)
        {
            r.sleep();

        }

        //ROS_INFO("gripper action done %i\n", gripperMotionSuccess);
        asGripper_.setSucceeded(result_);
        //clearTXBuffer();

    }

    /* clear flags upon finishing action */
    void clearTXFlag()
    {

        //std::lock_guard<std::mutex> lock(variable_lock);
        //reset command
        TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(0);

        //TXData.ros2podo_data.Gripper_action.mode = 0;
        //TXData.ros2podo_data. = static_cast<GRIPPER_PARAMETER>(0);
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



/*Print helpful summary about this code */
void printInitialInfo()
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Starting ROS2PODO Motion Action Server" << std::endl << std::endl;
    std::cout << "   Developer: Heo Yujin" << std::endl;
    std::cout << "   Developer: Cho Saihim" << std::endl;
    std::cout << "   E-mail   : blike4@kaist.ac.kr" << std::endl;
    std::cout << "   E-mail   : chosaihim@kaist.ac.kr" << std::endl;
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

void LANthread_update()
{

    static unsigned int tcp_status = 0x00;
    static int tcp_size = 0;
    static int connectCnt = 0;

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
    }
}

/*========================== End of LAN functions ==================================*/


/*========================== main while loop ==================================*/
int main(int argc, char **argv)
{
    printInitialInfo();
    //Initialize ROS node
    ros::init(argc, argv, "delayTest_server");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    
    // Create Socket
    initializeSocket();

    //Initialize ROS Action Server for TX to PODO
    RosPODO_BaseAction rospodo_base("rospodo_base");
    RosPODO_ArmAction rospodo_arm("rospodo_arm");
    RosPODO_GripperAction rospodo_gripper("rospodo_gripper");
    ROS_INFO("Starting ROS2PODO Action Servers");

    ros::Time tzero(0);
    ros::Time beginTime = ros::Time::now();
    double beginTime_d = ros::Time::now().toSec();
    double nowNow;

    static int loop_cnt = 0;
   
    /* === main while loop to RX feedback calls at regular periods === */
    while(ros::ok())
    {

//        beginTime_d = ros::Time::now().toSec();

        //update RX values from PODO
        //LANthread_update();

        //check if action server is active

//        if(rospodo_base.returnServerStatus())
//        {
//            rospodo_base.publishFeedback();
//            rospodo_base.publishResult();
//        }

//        if(rospodo_arm.returnServerStatus())
//        {
//            rospodo_arm.publishFeedback();
//            rospodo_arm.publishResult();
//        }

//        if(rospodo_gripper.returnServerStatus())
//        {
//            rospodo_gripper.publishResult();
//        }

        //loop at desired rate
        ros::spinOnce();
        loop_rate.sleep();


//        nowNow = ros::Time::now().toSec();
        //std::cout << "loop took " << (nowNow - beginTime_d) *1000 << " msecs." << std::endl;
        //sstd::cout << "loop count: " << loop_cnt++ << std::endl;
    }
    return 0;
}
/*========================== End of Code ==================================*/
