#ifndef ROSCOMM_H
#define ROSCOMM_H

#define ROS_SHM_NAME        "ROS_SHARED_MEMORY"

#include "ROS_COMMAND.h"
#include "ROS_DATA.h"
#include "PODO_STATE.h"
 
typedef struct _ROS2PODO_DATA_
{
    JOINTMOVE_CMD       CMD_JOINT;
    GRIPPERMOVE_CMD     CMD_GRIPPER;
    WHEELMOVE_CMD       CMD_WHEEL;

    MANIPULATOR_ACTION  Arm_action;
    BASE_ACTION         Base_action;
    GRIPPER_ACTION      Gripper_action;

    int index;
}ROS2PODO_DATA;

typedef struct _PODO2ROS_DATA_
{
    MANIPULATOR_ACTION  Arm_feedback;
    BASE_ACTION         Base_feedback;
    GRIPPER_ACTION      Gripper_feedback;

    SENSOR_DATA sensor;
    ROBOT_STATE_ARM  state_arm;
    ROBOT_STATE_BASE state_base;

    int index;
}PODO2ROS_DATA;

typedef struct _ROS_JOINTREF_
{
    JOINT_DATA joint[NUM_JOINTS];
    int DoneFlag;
    int StartFlag;
}__attribute__((packed))ROS_JOINTREF;

#endif // ROSCOMM_H
