#!/usr/bin/env python

 ###################################################################
 #
 # This ROS node is Action Client that enables motion on Mobile Hubo Platform.
 # This is only an example file to serve as a tutorial for new teams developing on Mobile Hubo Platform.
 # In order for correct operation, please ensure PODO Software version is 'MobileHubo_ROSmotion' inside Hubo Motion PC.
 # Refer to ROSPodo Motion Manual at www.kirobotics.com
 # 
 #
 # Output : /rospodo_base/goal
 #          /rospodo_arm/goal
 #          /rospodo_gripper/goal
 # 
 # Input  : /rospodo_base/result
 #          /rospodo_arm/result
 #          /rospodo_gripper/result
 
 # E-mail : ml634@kaist.ac.kr     (Lee Moonyoung)
 # E-mail : blike4@kaist.ac.kr    (Heo Yujin)
 # E-mail : saihimcho@kaist.ac.kr (Cho Saihim)
 #
 # Versions :
 # v1.0.2019.05
 ###################################################################
 
 

#import standard ROS packages
import rospy
import numpy as np
import smach
import smach_ros
from smach import StateMachine
from std_msgs.msg import String
from std_msgs.msg import Int32
import time

#import ROSPODO action headers
import actionlib
import ros_podo_connector.msg 
from ros_podo_connector.msg import JointData
from ros_podo_connector.msg import WbikData
from ros_podo_connector.msg import *


#global var
base_result_flag = False
arm_result_flag = False
gripper_result_flag = False

# ================================================ result subscribe call back ================================================ #

def base_result_callback(data):
    global base_result_flag
    rospy.loginfo("Base Callback result flag: %i", data.result.result_flag)
    base_result_flag = True

def arm_result_callback(data):
    global arm_result_flag
    rospy.loginfo("Arm Callback result flag: %i", data.result.result_flag)
    arm_result_flag = True

def gripper_result_callback(data):
    global gripper_result_flag
    rospy.loginfo("Gripper Callback result flag: %i", data.result.result_flag)
    gripper_result_flag = True

# ================================================ state machine states ================================================ #
# define state IDLE
class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
 

    def execute(self, userdata):
        #simple time countdown 
        timeCountSec = 3

        while timeCountSec:
            mins, secs = divmod(timeCountSec, 60)
            timeformat = '{:02d}:{:02d}'.format(mins, secs)
            print("time until action start: " + str(timeformat))
            time.sleep(1)
            timeCountSec -= 1

        print(" === Starting Store Application Motion ===")

        return 'outcome1' #go to display
    
        

# define state GOTO_DISPLAY
class GOTO_DISPLAY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GOTO_DISPLAY')
        global ac_base, base_result_flag
        global ac_arm, arm_result_flag
        global ac_gripper, gripper_result_flag


        # Creates a base goal with parameters.
        goal_base = ros_podo_connector.msg.RosPODO_BaseGoal(wheelmove_cmd = 1, MoveX = -3.0, MoveY = 0.0, ThetaDeg = 0 )
        ac_base.send_goal(goal_base)


        # Creates an arm goal for WST movement.
        arnGoal_joint_ref = []
        for i in range(18):
            arnGoal_joint_ref.append( ros_podo_connector.msg.JointData(OnOffControl = 0) )


        # desired parameters 
        arnGoal_joint_ref[14].OnOffControl = 1
        arnGoal_joint_ref[14].reference = -180
        arnGoal_joint_ref[14].GoalmsTime = 5000

        arnGoal_joint_ref[6].OnOffControl = 1
        arnGoal_joint_ref[6].reference = -90
        arnGoal_joint_ref[6].GoalmsTime = 5000

        # send arm goal with desired parameters
        goal_arm = ros_podo_connector.msg.RosPODO_ArmGoal(jointmove_cmd = 2, joint_ref = arnGoal_joint_ref)
        ac_arm.send_goal(goal_arm)


        # send gripper goal with desired parameters
        goal_gripper = ros_podo_connector.msg.RosPODO_GripperGoal(grippermove_cmd = 2, mode = 1)
        ac_gripper.send_goal(goal_gripper)
        gripper_result_flag = False


        # check for done flags
        rospy.Subscriber("/rospodo_base/result",RosPODO_BaseActionResult, base_result_callback, queue_size=1, buff_size=10)
        rospy.Subscriber("/rospodo_arm/result",RosPODO_ArmActionResult, arm_result_callback, queue_size=1, buff_size=10)

        rate = rospy.Rate(10) # 10hz

        # wait in loop until received done flags
        while(base_result_flag is False or arm_result_flag is False):
            rate.sleep()


        if (base_result_flag is True and arm_result_flag is True):

            return 'outcome1' #next
        
        else:
            
            return 'outcome2' #abort
        


# define state REACH
class REACH(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state REACH')
        global ac_arm, arm_result_flag

        # Creates a goal to send to the action server.
        arnGoal_joint_ref = []
        for i in range(18):
            arnGoal_joint_ref.append( ros_podo_connector.msg.JointData(OnOffControl = 0) )

        arnGoal_wbik_ref = []
        for i in range(6):
            arnGoal_wbik_ref.append( ros_podo_connector.msg.WbikData(OnOff_position = 0) )

        # desired parameters 
        arnGoal_wbik_ref[0].OnOff_position = 1
        arnGoal_wbik_ref[0].goal_position[0] = 0.6
        arnGoal_wbik_ref[0].goal_position[1] = -0.3
        arnGoal_wbik_ref[0].goal_position[2] = 0.2
        arnGoal_wbik_ref[0].GoalmsTime = 2000

        arnGoal_wbik_ref[1].OnOff_position = 1
        arnGoal_wbik_ref[1].goal_angle = -30.0
        arnGoal_wbik_ref[1].GoalmsTime = 2000

        # send arm goal with desired parameters
        goal_arm = ros_podo_connector.msg.RosPODO_ArmGoal(jointmove_cmd = 3, joint_ref = arnGoal_joint_ref, wbik_ref = arnGoal_wbik_ref)

        ac_arm.send_goal(goal_arm)
        arm_result_flag = False

        # check for done flag
        rospy.Subscriber("/rospodo_arm/result",RosPODO_ArmActionResult, arm_result_callback, queue_size=1, buff_size=10)
        rate = rospy.Rate(10) # 10hz

        # wait in loop until received done flags
        while(arm_result_flag is False):
            rate.sleep()



        if (arm_result_flag is True):

            return 'outcome1' #next
        
        else:
            
            return 'outcome2' #abort

# define state GRASP
class GRASP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP')
        global ac_gripper, gripper_result_flag

        # check for done flags
        rospy.Subscriber("/rospodo_gripper/result",RosPODO_GripperActionResult, gripper_result_callback, queue_size=1, buff_size=10)

        # send gripper goal with desired parameters
        goal_gripper = ros_podo_connector.msg.RosPODO_GripperGoal(grippermove_cmd = 3, mode = 1)
        ac_gripper.send_goal(goal_gripper)
        gripper_result_flag = False


        
        rate = rospy.Rate(10) # 10hz

        # wait in loop until received done flags
        while(gripper_result_flag is False):
            rate.sleep()


        if (gripper_result_flag is True):

            return 'outcome1' #next
        
        else:
            
            return 'outcome2' #abort


# define state GOTO_HOME
class GOTO_HOME(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GOTO_HOME')
        global ac_arm, arm_result_flag
        global ac_base, base_result_flag

        # Creates a goal to send to the action server.
        arnGoal_joint_ref = []
        for i in range(18):
            arnGoal_joint_ref.append( ros_podo_connector.msg.JointData(OnOffControl = 0) )

        arnGoal_wbik_ref = []
        for i in range(6):
            arnGoal_wbik_ref.append( ros_podo_connector.msg.WbikData(OnOff_position = 0) )


        # desired parameters 
        arnGoal_joint_ref[14].OnOffControl = 1
        arnGoal_joint_ref[14].reference = 0
        arnGoal_joint_ref[14].GoalmsTime = 5000

        # send arm goal with desired parameters
        goal_arm = ros_podo_connector.msg.RosPODO_ArmGoal(jointmove_cmd = 2, joint_ref = arnGoal_joint_ref)
        ac_arm.send_goal(goal_arm)


        # Creates a base goal with parameters.
        goal_base = ros_podo_connector.msg.RosPODO_BaseGoal(wheelmove_cmd = 1, MoveX = 3.0, MoveY = 0.0, ThetaDeg = 0 )
        ac_base.send_goal(goal_base)

        base_result_flag = False
        arm_result_flag = False

        # check for done flag
        rospy.Subscriber("/rospodo_base/result",RosPODO_BaseActionResult, base_result_callback, queue_size=1, buff_size=10)
        rospy.Subscriber("/rospodo_arm/result",RosPODO_ArmActionResult, arm_result_callback, queue_size=1, buff_size=10)
        
        rate = rospy.Rate(10) # 10hz

        # wait in loop until received done flags
        while(arm_result_flag is False or base_result_flag is False):
            rate.sleep()



        if (arm_result_flag is True and base_result_flag is True):

            return 'outcome1' #next
        
        else:
            
            return 'outcome2' #abort
        

# ================================================ main loop ================================================ #


def main():
    #initialize node
    rospy.init_node('ros2podo_action_FSMclient')
    #declare global var
    global ac_base, ac_arm, ac_gripper

    # Creates the SimpleActionClient, passing the type of the action (RosPODO_BaseAction) to the constructor.
    ac_base = actionlib.SimpleActionClient('rospodo_base', ros_podo_connector.msg.RosPODO_BaseAction)
    ac_arm = actionlib.SimpleActionClient('rospodo_arm', ros_podo_connector.msg.RosPODO_ArmAction)
    ac_gripper = actionlib.SimpleActionClient('rospodo_gripper', ros_podo_connector.msg.RosPODO_GripperAction)

    # Waits until the action server has started up and started
    # listening for goals.
    ac_base.wait_for_server()
    ac_arm.wait_for_server()
    ac_gripper.wait_for_server()

    
   
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['ABORTED', 'RETURNED_SUCCESS', 'RETURNED_WITHOUT'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('IDLE', IDLE(), 
                               transitions={'outcome1':'GOTO_DISPLAY'})

        smach.StateMachine.add('GOTO_DISPLAY', GOTO_DISPLAY(), 
                               transitions={'outcome1':'REACH', 'outcome2':'ABORTED'})

        smach.StateMachine.add('REACH', REACH(), 
                               transitions={'outcome1':'GRASP', 'outcome2':'ABORTED'})

        smach.StateMachine.add('GRASP', GRASP(), 
                               transitions={'outcome1':'GOTO_HOME', 'outcome2':'ABORTED'})

        smach.StateMachine.add('GOTO_HOME', GOTO_HOME(), 
                               transitions={'outcome1':'RETURNED_SUCCESS', 'outcome2':'RETURNED_WITHOUT', 'outcome3':'ABORTED'})





    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    sis.stop()
   

if __name__ == '__main__':
    main()
