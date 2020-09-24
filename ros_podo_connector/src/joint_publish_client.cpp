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

    //debugging
    // std::cout<< "callback is called" << std::endl;

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
    ros::init(argc, argv, "joint_publish_client");

    ros::NodeHandle nh("~");
    int param = 500; double freq = 200;
    nh.getParam("param", param);
    nh.getParam("freq",freq);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_base.waitForServer();      //will wait for infinite time
    ac_arm.waitForServer();       //will wait for infinite time
    ac_gripper.waitForServer();   //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // create goal instance
    ros_podo_connector::RosPODO_BaseGoal      goal_base;
    ros_podo_connector::RosPODO_ArmGoal       goal_arm;
    ros_podo_connector::RosPODO_GripperGoal   goal_gripper;



    /* ============== Arm Data Action JOINT PUBLISH ==============  */
    ros::Subscriber joint_states_sub;
    ros::NodeHandle n;
    joint_states_sub = n.subscribe("joint_states", 100, joint_states_callback);

    //debugging:: Interpolated joint_states
    inter_joint_states_pub = n.advertise<sensor_msgs::JointState>("inter_joint_states",1);
    inter_joint_states.name.resize(NUM_JOINTS);
    inter_joint_states.position.resize(NUM_JOINTS);

    //initializing
    for(int i=0; i< NUM_JOINTS; i++){
        inter_joint_states.name[i] = JointBufferNameList[i];
    }



    ros::Time tzero(0);
    ros::Time beginTime = ros::Time::now();
    double beginTime_d = ros::Time::now().toSec();
    double nowNow;

    //ros::Rate loop_rate(200);
    ros::Rate loop_rate(freq);
    double d_ref[NUM_JOINTS-3];

    //delay test
    int test_cnt=0;

    //Subscribe topic "joint_states"
    while(test_cnt < param )//(ros::ok())
    {

        std::cout << "test_cnt " << test_cnt++ << std::endl;

        ros::spinOnce();
        nowNow = ros::Time::now().toSec();

        //First CB called
        if(CB_flag == 1){
            for(int i=0; i<NUM_JOINTS-3; i++){
                goal_arm.joint_ref[i].reference = curr_ref[i]*R2Df;
                goal_arm.joint_ref[i].OnOffControl = CONTROL_ON;
            }
            goal_arm.jointmove_cmd = MODE_JOINT_PUBLISH;

            // send a goal to the action
            ac_arm.sendGoal(goal_arm);
        }

        // if CB is called more than once
        if(CB_flag >1)
        {
            //interpolate
            for(int i=0; i<NUM_JOINTS-3; i++){
                d_ref[i] = (curr_ref[i] - prev_ref[i]) / prev_cnt;
                goal_arm.joint_ref[i].reference += d_ref[i]*R2Df;
                goal_arm.joint_ref[i].OnOffControl = CONTROL_ON;

                //debugging
                inter_joint_states.position[i] = goal_arm.joint_ref[i].reference;     //debugging
            }
            goal_arm.jointmove_cmd = MODE_JOINT_PUBLISH;

            //debugging(publishing a topic for sub_test node)
            //inter_joint_states_pub.publish(inter_joint_states);     //debugging

            // send a goal to the action
            ac_arm.sendGoal(goal_arm);

            static int cnt_toServer = 0;
            //std::cout << "count to Server: " << cnt_toServer++ << std::endl;
        }

        loop_cnt++;
        //std::cout << "Loop Counter = " << loop_cnt << std::endl;  //debugging
        loop_rate.sleep();
        beginTime_d = ros::Time::now().toSec();
    }

    //exit
    return 0;
}
