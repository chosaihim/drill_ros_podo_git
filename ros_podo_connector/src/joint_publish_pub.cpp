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


#include <sensor_msgs/JointState.h>

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

//variables
int loop_cnt=0; //loop counter
int prev_cnt=0; //save loop counter for calculation.
double prev_ref[NUM_JOINTS]={0,};
double curr_ref[NUM_JOINTS]={0,};

int CB_flag = 0;

//subscriber
ros::Subscriber joint_states_sub;
const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};

//debugging
ros::Publisher inter_joint_states_pub;
sensor_msgs::JointState inter_joint_states;

void joint_states_callback(const sensor_msgs::JointState& joint_state_msg){

    CB_flag++;
    prev_cnt = loop_cnt;
    loop_cnt = 0;

    for(int i=0; i<NUM_JOINTS; i++)
        prev_ref[i] = curr_ref[i];

    //Get current reference values from joint_states topic
    for(int i=0; i< NUM_JOINTS; i++){         //Match Joint names(order of "joint_states(topic)" and "curr_ref(array)" are different, so the names are compared to get right joint reference)
        for(int j=0; j< joint_state_msg.name.size(); j++){
            if(JointBufferNameList[i] == joint_state_msg.name[j]){
                curr_ref[i] = joint_state_msg.position[j];
            }
        }
    }
}




int main (int argc, char **argv)
{
    ros::init(argc, argv, "joint_publish_pub");

    /* ============== Arm Data Action JOINT PUBLISH ==============  */

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);

    ros::NodeHandle nh("~");
    int param = 500; double freq = 200;
    nh.getParam("param", param);
    nh.getParam("freq", freq);


    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_arm.waitForServer();       //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // create goal instance
    ros_podo_connector::RosPODO_ArmGoal       goal_arm;


    ros::Subscriber joint_states_sub;
    ros::NodeHandle n;
    joint_states_sub = n.subscribe("joint_states", 100, joint_states_callback);

    //debugging:: Interpolated joint_states
    inter_joint_states_pub = n.advertise<sensor_msgs::JointState>("inter_joint_states",1);
    inter_joint_states.name.resize(NUM_JOINTS);
    inter_joint_states.position.resize(NUM_JOINTS);

    //initializing
    for(int i=0; i< NUM_JOINTS; i++)
        inter_joint_states.name[i] = JointBufferNameList[i];


    double d_ref[NUM_JOINTS];
//    ros::Rate loop_rate(200);
    ros::Rate loop_rate(freq);


    static int pub_cnt = 0;

    //Subscribe topic "joint_states"
//    while(ros::ok())
    while(pub_cnt < param)
    {
        ros::spinOnce();

        //First CB called
        if(CB_flag == 1){   //initial position
            for(int i=0; i<NUM_JOINTS; i++){
                inter_joint_states.position[i] = curr_ref[i]*R2Df;
            }
        }

        // if CB is called more than once
        if(CB_flag >1)
        {
            //interpolate
            for(int i=0; i<NUM_JOINTS; i++){
                d_ref[i] = (curr_ref[i] - prev_ref[i]) / prev_cnt;
                inter_joint_states.position[i] += d_ref[i]*R2Df;
            }

            //debugging
            inter_joint_states.position[NUM_JOINTS-1] = pub_cnt;        //debugging

            inter_joint_states_pub.publish(inter_joint_states);

            std::cout << "cnt pub = " << pub_cnt++ << std::endl;  //debugging

        }

        loop_cnt++;
        //std::cout << "Loop Counter = " << loop_cnt << std::endl;  //debugging
        loop_rate.sleep();
    }

    //exit
    return 0;
}
