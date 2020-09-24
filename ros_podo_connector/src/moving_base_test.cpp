#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
/*custom defined Action header for robot motion */
#include "ros_podo_connector/key_vel.h"
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>

/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"
/* ROS package include */
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>

#include <sensor_msgs/JointState.h>

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

//variables
ros::Publisher path_pub; //path difference
const int averageWindowN = 5; //sample number for low-pass filter
const float grid_resolution = 0.1; //meter

float velocityxyzw [4][averageWindowN];
float vx_tx, vy_tx, vw_tx, vz_tx;

float sum_of_i = 0;

int received_path = 0;

void velocityFromPath(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO(" ============= received path CB =============");
    received_path = 1;
    
    /*check for non-empty vector*/
    if(msg->poses.empty())
    {
		ROS_ERROR(" empty path");
		return;
	}
    int pathSize = msg->poses.size();
    ROS_INFO("path size: %d", pathSize); 
    
    nav_msgs::Path pathDifferece; //vectory of differences in path
    geometry_msgs::PoseStamped temporaryPose; //used to fill
    
    //create pose difference array 
    for(int i = 0; i < pathSize-1; i++)
    {
		temporaryPose.pose.position.x = msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x;
		temporaryPose.pose.position.y = msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y;
		temporaryPose.pose.orientation.z = msg->poses[i+1].pose.orientation.z -msg->poses[i].pose.orientation.z;
		temporaryPose.pose.orientation.w = msg->poses[i+1].pose.orientation.w -msg->poses[i].pose.orientation.w;
		
		pathDifferece.poses.push_back(temporaryPose);
	}
    
    int pathSize2 = pathDifferece.poses.size();
    //ROS_INFO("path2 size: %d", pathDifferece.poses.size()); 
    
    path_pub.publish(pathDifferece);
    
    
    /* deltaX range [0.0~0.1]m */
    /* velocity range [0.0~1.0] */
    /* linearly scale delta to velocity output range*/
    float vx_ref, vy_ref, vz_ref, vw_ref;
    float vx_out, vy_out, vz_out, vw_out;
    
    if (pathSize > averageWindowN ) //more samples remainining
    {
		
			if(pathDifferece.poses.empty())
		{
			ROS_ERROR("Difference path  empty!!!");
			return;
		}
		
		ROS_INFO("Difference path size: %d", pathSize2); 
		
	
		for (int i =0; i < averageWindowN; i++) //sum the next window of path differences
		{
			vx_ref = vx_ref + pathDifferece.poses[i].pose.position.x;
			vy_ref = vy_ref + pathDifferece.poses[i].pose.position.y;
			vw_ref = vw_ref + pathDifferece.poses[i].pose.orientation.z;
			vz_ref = vz_ref + pathDifferece.poses[i].pose.orientation.w;
		}
		
		//average and then scale to unitless velocity
    vx_ref = vx_ref / averageWindowN * (1.0-0.0)/grid_resolution;
    vy_ref = vy_ref / averageWindowN * (1.0-0.0)/grid_resolution;
    vz_ref = vz_ref / averageWindowN * (1.0-0.0)/grid_resolution;
    vw_ref = vw_ref / averageWindowN * (1.0-0.0)/grid_resolution;
    
    ROS_INFO("vx_ref: %f, vy_ref: %f\n", vx_ref, vy_ref); 
    
    //new reference at [end]
    velocityxyzw [0][averageWindowN-1] = vx_ref;
    velocityxyzw [1][averageWindowN-1] = vy_ref;
    velocityxyzw [2][averageWindowN-1] = vz_ref;
    velocityxyzw [3][averageWindowN-1] = vw_ref;
    
    
    /* to weight the LPF values by recent ones*/
    sum_of_i = 0;
    
    for(int i =1; i < averageWindowN+1; i++)
    {
		sum_of_i = sum_of_i + i;
		 
	}
        //ramp velocity using low-pass filter window
    for(int i =0; i < averageWindowN; i++)
    {
		
		vx_out = vx_out + ((i+1)/float(sum_of_i))*velocityxyzw[0][i];
		vy_out = vy_out + ((i+1)/float(sum_of_i))*velocityxyzw[1][i];
		vz_out = vz_out + ((i+1)/float(sum_of_i))*velocityxyzw[2][i];
		vw_out = vw_out + ((i+1)/float(sum_of_i))*velocityxyzw[3][i];
		
	}
	
	
	
	
	//update window
	for (int i =0; i < averageWindowN-1; i++)
	{
		velocityxyzw [0][i] = velocityxyzw [0][i+1];
		velocityxyzw [1][i] = velocityxyzw [1][i+1];
		velocityxyzw [2][i] = velocityxyzw [2][i+1];
		velocityxyzw [3][i] = velocityxyzw [3][i+1];
	}
	
	
	}
	
	else //less than averageWindowN samples remaining
	{
		
		
		if (pathSize > 2)
		{
			
			/*
			vx_out = vx_tx / 2;
			vy_out = vy_tx / 2;
			vz_out = vz_tx / 2;
			vw_out = vw_tx / 2;
			*/
			 
			vx_out = pathDifferece.poses[0].pose.position.x * pathSize; // [-0.1 ~ 0.1] * [1 ~ 5]
			vy_out = pathDifferece.poses[0].pose.position.y * pathSize;
			vz_out = pathDifferece.poses[0].pose.orientation.z * pathSize;
			vw_out = pathDifferece.poses[0].pose.orientation.w * pathSize;
			
		}
		
		else
		{
			vx_out = 0;
			vy_out = 0;
			vz_out = 0;
			vw_out = 0;
			
			
		}
	
	}
	
	/* bound max values*/
	if (vx_out <= 1.0 and vx_out >= -1.0) vx_tx = vx_out;
	else if(vx_out > 1.0 ) 		vx_tx = 1.0;
	else if(vx_out < -1.0) 	vx_tx = -1.0;
	else vx_tx = 0;
	
	if(vy_out <= 1.0 and vy_out >= -1.0) 	vy_tx = vy_out;
	else if(vy_out > 1.0) 		vy_tx = 1.0;
	else if(vy_out < -1.0) 	vy_tx = -1.0;
	else vy_tx = 0;
	
	/* handle nan or inf */
	
	ROS_INFO("\tvx: %f, vy: %f\n", vx_out, vy_out); 
	
	
	
	
    
    

    
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "moving_base_test");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
   
    ROS_INFO("Waiting for action server to start LMY");
    // wait for the action server to start
    //ac_base.waitForServer();      //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // create goal instance
    ros_podo_connector::RosPODO_BaseGoal      goal_base;


    /* ============== Initialize ==============  */
    ros::NodeHandle n;
    ros::Subscriber key_input_sub = n.subscribe("/mobile_hubo/navigation_path",100,velocityFromPath);
    
    
    path_pub = n.advertise<nav_msgs::Path>("path",100);
    

    
    ros::Rate loop_rate(10);

    

    //Subscribe topic "joint_states"
    while(ros::ok())
    {
		
        ros::spinOnce();
        
        /* start sending velocity command */
        if( received_path == 1)
        {
			goal_base.wheelmove_cmd = WHEEL_MOVE_VELOCITY;
			goal_base.VelX = vx_tx;
            goal_base.VelY = vy_tx;
			ROS_INFO("vx: %f, vy: %f\n", goal_base.VelX, goal_base.VelY); 
            
            ac_base.sendGoal(goal_base);
            
		}
		
		received_path = 0;
        

        
        loop_rate.sleep();
    }

    //exit
    return 0;
}
