#!/usr/bin/env python

import rospy
import actionlib
#import actionlib_tutorials.msg
#import actionlib_tutorials.msg._JointData
#import actionlib_tutorials.msg._JointDataSet
import ros_podo_connector.msg
import ros_podo_connector.msg._JointData
import ros_podo_connector.msg._JointDataSet

from arm_move.srv._arm_move_srv import *

NUM_JOINTS = 18

def convert2action(plan, move_group):
    # print type(plan), plan.joint_names, plan.joint_names
    # for i in range(10):
    #     print i, "th time:", type(plan.points[i].time_from_start.to_sec()), plan.points[i].time_from_start.to_sec()
    #     print i, "th pos:", plan.points[i].positions
    act_client = actionlib.SimpleActionClient('rospodo_trajectory', ros_podo_connector.msg.RosPODO_TrajectoryAction)
    act_client.wait_for_server()

#    action_goal = actionlib_tutorials.msg.RosPODO_TrajectoryGoal()
    action_goal = ros_podo_connector.msg.RosPODO_TrajectoryGoal()
    print "action length:", len(action_goal.via_point)

    action_goal.num_points = len(plan.points)
    action_goal.planGroup = move_group

    for point_i in range(len(plan.points)):
        for joint_i in range(NUM_JOINTS):
            for joint_tra_i in range(len(plan.joint_names)):
                # only if joint name matches
                if joint_buffer_list[joint_i] == plan.joint_names[joint_tra_i]:
                    action_goal.via_point[point_i].joint[joint_i].OnOffControl = 1
                    action_goal.via_point[point_i].joint[joint_i].reference = plan.points[point_i].positions[joint_tra_i]
                    if point_i > 0:
                        action_goal.via_point[point_i].joint[joint_i].GoalmsTime = plan.points[point_i].time_from_start.to_sec()-plan.points[point_i-1].time_from_start.to_sec()
                    else:
                        action_goal.via_point[point_i].joint[joint_i].GoalmsTime = 0.0
                    break
                else:
                    action_goal.via_point[point_i].joint[joint_i].OnOffControl = 0
                    action_goal.via_point[point_i].joint[joint_i].reference = 0.
                    action_goal.via_point[point_i].joint[joint_i].GoalmsTime = 0.
    for i in range(5):
        print "action goal", action_goal.via_point[i].joint[0].GoalmsTime

    act_client.send_goal(action_goal)
    act_client.wait_for_result()

    return act_client.get_result()

def feasible_check_client():
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append('R_arm')
        pub_msg.goal_position.x = 0.3
        pub_msg.goal_position.y = -0.4
        pub_msg.goal_position.z = 0.7

        pub_msg.goal_orientation.x = 0.5
        pub_msg.goal_orientation.y = -0.5
        pub_msg.goal_orientation.z = -0.5
        pub_msg.goal_orientation.w = 0.5
        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        if resp1.w_flag == 1:
            print "work done"
        if resp1.feasibility == 1:
            print "Plan is found successfully with", len(resp1.r_trj.joint_trajectory.points), "steps"
            print "Send convert plan_msg to action_msg"
            retGoal = convert2action(resp1.r_trj.joint_trajectory, 'R_arm')
            # print retGoal

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e



if __name__ == '__main__':
    rospy.init_node('traj_send_client', anonymous=True)
    print "Requesting for feasibility check in moveIT"
    joint_buffer_list = ["RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2", "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2", "WST", "RWH", "LWH", "BWH"]
    feasible_check_client()

