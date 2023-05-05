#!/usr/bin/env python
"""
.. module:: send_goal_to_arm.py
   :platform: Unix
   :synopsis: Motion planning for the robot arm to detect the aruco tags 
.. moduleauthor:: yeshwanth guru krishnakumar

This script is a Python program that uses the MoveIt! package and ROS to control a robotic arm. It does the following:

Initializes the ROS and MoveIt! nodes.
Instantiates a MoveGroupCommander object for the robotic arm.
Sets named targets for the arm to move to, such as "home", "left_45", "horse", etc.
Plans and executes a trajectory to reach each target.
Shuts down the MoveIt! and ROS nodes.
Initializes a new ROS node.
Publishes a boolean message on the "/decision" topic.
Sleeps for 1 second to wait for the publisher to initialize.
Sets the boolean message to True and publishes it.
Signals to ROS that the program is done.
 
"""
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from std_msgs.msg import Bool

def main():
    """
    A script to execute a sequence of predefined joint poses using MoveIt!.

    The script initializes MoveIt! and a ROS node, sets a named target pose for the robot arm, plans and
    executes the motion to reach the target pose, and repeats this process for a sequence of different target
    poses. After reaching the final target pose, the script publishes a boolean message and shuts down the nodes.
    """

    # Initialize the moveit_commander and rospy nodes
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Instantiate a MoveGroupCommander object for the arm group
    arm = MoveGroupCommander('arm')

    # Set the joint pose named "back_45"
    arm.set_named_target('back_45')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "home"
    arm.set_named_target('home')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "home_2"
    arm.set_named_target('home_2')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "right_45"
    arm.set_named_target('right_45')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "home3"
    arm.set_named_target('home3')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "down"
    arm.set_named_target('down')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "down_left"
    arm.set_named_target('down_left')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "down_right"
    arm.set_named_target('down_right')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "horse"
    arm.set_named_target('horse')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "horse_left"
    arm.set_named_target('horse_left')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "horse_right"
    arm.set_named_target('horse_right')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Set the joint pose named "left_45"
    arm.set_named_target('left_45')

    # Plan and execute the trajectory to the target pose
    arm.go()

    # Shutdown moveit_commander and rospy nodes
    moveit_commander.roscpp_shutdown()

    # Publish boolean message
    rospy.init_node('move_group_python_interface', anonymous=True)

    pub = rospy.Publisher('/decision', Bool, queue_size=10)
    rospy.sleep(1)  # wait for the publisher to initialize
    msg = Bool()
    msg.data = True
    pub.publish(msg)

    rospy.signal_shutdown("Done")

if __name__ == '__main__':
    main()

