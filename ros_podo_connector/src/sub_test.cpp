#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>

#include "RosCommand.h"
#include "ros/time.h"


const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

sensor_msgs::JointState joint_state;

//subscriber
ros::Subscriber joint_states_sub;
ros::Subscriber inter_joint_states_sub;
sensor_msgs::JointState inter_joint_states;

double curr_ref[NUM_JOINTS]={0,};


void joint_states_callback(const sensor_msgs::JointState& joint_state_msg){
        std::cout << "Joint state has been Subscribed" << std::endl;

        double time_secs =ros::Time::now().toSec();

        //Match Joint names
        for(int i=0; i< NUM_JOINTS; i++){ //joints in "joint_state" are listed in different order with the joints in joint_information.h
            for(int j=0; j< joint_state_msg.name.size(); j++){                           //go through all the joint names of "joint_states"
                if(JointBufferNameList[i] == joint_state_msg.name[j]){                   // store the joint information only when joint name matches
                    curr_ref[i] = joint_state_msg.position[j];
                }
            }
        }

        //file read and write
        FILE *testFile = NULL;
        testFile = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/src/test_joint_state.txt","a");
        if((testFile == NULL))
            std::cout << "Failed to open test_joint_state.txt" << std::endl;
        else{
            fprintf(testFile,"%f ",time_secs);
            for(int i=0; i<NUM_JOINTS; i++){
                fprintf(testFile,"%f, ",curr_ref[i]);
            }

            fprintf(testFile,"\n\n");
        }
        fclose(testFile);

}
void inter_joint_states_callback(const sensor_msgs::JointState& inter_joint_state_msg){

    std::cout << "Interpolated Joint state has been Subscribed" << std::endl;

    double time_secs =ros::Time::now().toSec();

    //file read and write
    FILE *testFile = NULL;
    testFile = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/src/test_interpolated.txt","a");
    if((testFile == NULL))
        std::cout << "Failed to open test_interpolated.txt" << std::endl;
    else{
        fprintf(testFile,"%f ",time_secs);
        for(int i=0; i<NUM_JOINTS; i++){
            fprintf(testFile,"%f, ",inter_joint_state_msg.position[i]);
        }

        fprintf(testFile,"\n\n");
    }
    fclose(testFile);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_test");
    ros::NodeHandle n;

    joint_states_sub = n.subscribe("joint_states", 100, joint_states_callback);
    inter_joint_states_sub = n.subscribe("inter_joint_states", 100, inter_joint_states_callback);


    ros::Time tzero(0);
    ros::Time beginTime = ros::Time::now();
    
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}




