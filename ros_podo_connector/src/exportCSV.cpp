
// ##############################################
// #
// #Purpose: save joint states data to CSV
// #Author: Moonyoung Lee (ml634@kaist.ac.kr)
// #Date:  04.2019
// #
// ##############################################


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "tf/transform_datatypes.h"

#include <fstream>

//robot joint info
#include "ROSLANData.h"

#define R2D			5.729577951308232e1

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;


ros::Publisher pub;
geometry_msgs::Pose tfPose;

using namespace std;
ofstream outputFile;

	  
/*global var*/
ros::Time currentTime, beginTime, currentTimeWhile, beginTimeWhile, beginTimeMain;
int FPScount = 0; int FPScountWhile = 0;

sensor_msgs::JointState joint_state_podo;
ros::Publisher pub_joint_state_podo;


void callback_jointstates(const sensor_msgs::JointState& joint_state_msg)
{
    
    //ROS_INFO("inside CB\n"); 
    
    //determine Hz
    currentTime = ros::Time::now();
	ros::Duration durationTime = currentTime - beginTime;
	FPScount++;
    if (durationTime.toSec() > 1.0) {
		ROS_INFO("Hz: %d\n", FPScount);
		FPScount = 0;
		beginTime = currentTime;
    
    }
    
    //copy joint state 
    //joint_state_podo = joint_state_msg;
	
	
	//read subscribed joint msg
    for(int i=0; i< NUM_JOINTS; i++)
    { 	
		//joints in "joint_state" are listed in different order with the joints in joint_information.h
		for(int j=0; j< joint_state_msg.name.size(); j++)
		{   
			//go through all the joint names of "joint_states"
			if(JointBufferNameList[i] == joint_state_msg.name[j])
			{   
				// store the joint information only when joint name matches
				//std::cout<< "matched" << std::endl;
    
				//Joint Reference copy to ROS joint state topic {time stamp, position, velocity}
				//joint_state_podo[i].position = joint_state_msg.position[j] *R2Df;  	
				
				//Joint Reference copy to PODO joint data {reference, time, ON/OFF}
				TXData.ros2podo_data.joint[i].reference = joint_state_msg.position[j] *R2Df;  //Joint Reference
				TXData.ros2podo_data.joint[i].ONOFF_control = CONTROL_ON;                	 //Move mode
				break; 	//break early if matched											
				
			}
		}
	}
    

    
  
  
}


int main(int argc, char **argv)
{
	
	
	//create CSV file
	outputFile.open("joint_ref.csv");
	
	//write time header to CSV file
	outputFile << "Time" << ",";
	
	//write data header to CSV file
	  for(int i=0; i< NUM_JOINTS; i++)
	  {
		  if(i == NUM_JOINTS-1)
		  {
			  outputFile << JointBufferNameList[i] << std::endl;
		  }
		  
		  else
		  {
			  outputFile << JointBufferNameList[i] << ",";
		  }
	  }
	  

    //initialize ROS node 
    ros::init(argc,argv, "exportCSV");
    ros::NodeHandle nh;
   
	//determine while loop rate 200hz or 10hz
    ros::Rate loop_rate(10); 
    
	//subscribe to 10Hz moveit joint
	ros::Subscriber sub_jointstate = nh.subscribe("/joint_states", 10, callback_jointstates);

    //publish higher 200Hz joint
    pub_joint_state_podo = nh.advertise<sensor_msgs::JointState>("/joint_states_podo", 10);
    
    //Time stamp at init 
	beginTimeMain = ros::Time::now();
	
    
  //main while loop
  while (ros::ok())
  {
  
	  //determine Hz
	  currentTimeWhile = ros::Time::now();
	  ros::Duration durationTimeWhile = currentTimeWhile - beginTimeWhile;
	  FPScountWhile++;
	  if (durationTimeWhile.toSec() > 1.0) 
	  {
		  ROS_INFO("While Hz: %d\n", FPScountWhile);
		  FPScountWhile = 0;
		  beginTimeWhile = currentTimeWhile;
	  }
	  
	  //write time stamp to CSV file
	  outputFile << ros::Time::now() - beginTimeMain << ",";
	  
	  //write data to CSV file
	  for(int i=0; i< NUM_JOINTS; i++)
	  {
		  if(i == NUM_JOINTS-1)
		  {
			  //std::cout << TXData.ros2podo_data.joint[i].reference << std::endl;
			  outputFile << TXData.ros2podo_data.joint[i].reference << std::endl;
		  }
		  
		  else
		  {
			  //std::cout << TXData.ros2podo_data.joint[i].reference << "," ;
			  outputFile << TXData.ros2podo_data.joint[i].reference << ",";
		  }

	}
	
	//pub_joint_state_podo.publish(joint_state_podo);
	  
	  
	  
	  //repeat loop at specified rate
	  ros::spinOnce();
	  loop_rate.sleep();
  }


  return 0;
}

