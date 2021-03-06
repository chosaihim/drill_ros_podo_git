#ifndef ROSCOMMAND_H
#define ROSCOMMAND_H

#define ROS_SHM_NAME        "ROS_SHARED_MEMORY"


enum JOINTMOVE_CMD
{
    MODE_BREAK = 0,
    MODE_JOINT_PUBLISH,
    MODE_MOVE_JOINT,
    MODE_SET_WBIK,
    MODE_E_STOP,
    MODE_SAVE,
    MODE_TRAJECTORY
};

enum GRIPPERMOVE_CMD
{
    GRIPPER_BREAK = 0,
    GRIPPER_STOP,
    GRIPPER_OPEN,
    GRIPPER_CLOSE
};

enum WHEELMOVE_CMD
{
    WHEEL_BREAK = 0,
    WHEEL_MOVE_START,
    WHEEL_MOVE_STOP,
    WHEEL_MOVE_VELOCITY
};

enum GRIPPER_SIDE
{
    GRIPPER_BOTH = 0,
    GRIPPER_RIGHT,
    GRIPPER_LEFT
};

typedef struct _ROS_COMMAND_DATA_
{
    JOINTMOVE_CMD   CMD_JOINT;
    GRIPPERMOVE_CMD CMD_GRIPPER;
    WHEELMOVE_CMD   CMD_WHEEL;
}ROS_COMMAND;
#endif // COMMAND_H
