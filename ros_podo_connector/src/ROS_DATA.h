#ifndef ROSDATA_H
#define ROSDATA_H

#include "JointInformation.h"
enum
{
    CONTROL_OFF = 0, CONTROL_ON
};

enum JOINTNUMBER
{
    rosRSP=0, rosRSR, rosRSY, rosREB, rosRWY, rosRWP, rosRWY2,
    rosLSP, rosLSR, rosLSY, rosLEB, rosLWY, rosLWP, rosLWY2,
    rosWST, rosRWH, rosLWH, rosBWH, NUM_JOINTS
};

const struct {
    int num;
} ROS_to_JOINT_NUM[NUM_JOINTS-3] = {
    RSP, RSR, RSY, REB, RWY, RWP, RWY2,
    LSP, LSR, LSY, LEB, LWY, LWP, LWY2,
    WST

};

const struct {
    int id;
    int ch;
} ROS_MC_Pairs[NUM_JOINTS] = {
    {13,0}, {14,0}, {15,0}, {15,1}, {16,0}, {16,1}, {21,0},
    {17,0}, {18,0}, {19,0}, {19,1}, {20,0}, {20,1}, {22,0},
    {12,0}, {4,1}, {10,1}, {4,0}
};


enum PARTNUMBER
{
    RIGHT_HAND = 0, RIGHT_ELBOW, LEFT_HAND, LEFT_ELBOW,
    PEL, WAIST, NUM_PARTS
};

typedef struct _JOINT_DATA_
{
    int ONOFF_control;
    double reference;
    double GoalmsTime;
} __attribute__((packed)) JOINT_DATA;

typedef struct _WBIK_DATA_
{
    int ONOFF_movepos;
    int ONOFF_moveori;
    double Goal_pos[3];
    double Goal_quat[4];
    double Goal_angle;
    double GoalmsTime;
}WBIK_DATA;

typedef struct _WHEELMOVE_DATA_
{
    double MoveX;
    double MoveY;
    double ThetaDeg;
    double VelX;
    double VelY;
    double VelTheta;
}WHEELMOVE_DATA;


typedef struct _ROS_ACTION_MANIPULATOR_
{
    JOINT_DATA joint[NUM_JOINTS];
    WBIK_DATA wbik[NUM_PARTS];

    int result_flag;
}MANIPULATOR_ACTION;


typedef struct _ROS_ACTION_BASE_
{
    WHEELMOVE_DATA wheel;

    int result_flag;
}BASE_ACTION;


typedef struct _ROS_ACTION_GRIPPER_
{
    int side;
    double desired_mm;
    int result_flag;
}GRIPPER_ACTION;


#endif // ROSDATA_H
