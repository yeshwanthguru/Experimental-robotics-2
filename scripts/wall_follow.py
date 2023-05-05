#! /usr/bin/env python
"""
.. module:: wall_follow.py
   :platform: Unix
   :synopsis: wall follow service for the survilanc erobot   
.. moduleauthor:: yeshwanth guru krishnakumar

This is a Python script for a ROS node that implements a wall-following behavior for a robot using laser sensors. The script defines several functions that return Twist messages that control the linear and angular velocities of the robot. It also defines a SimpleActionClient class that sends control actions to move the robot to a target location.

The main() function initializes a ROS node, creates a publisher and subscriber for the laser scan and Twist messages, and a service to switch the wall-follower on and off. The clbk_laser() function is the callback function for the laser scan subscriber that updates the regions_ variable, which is a dictionary that contains the distances to obstacles in different directions. The take_action() function determines the state of the robot based on the values in regions_ and calls the appropriate function to generate the Twist message to control the robot. The state of the robot is changed using the change_state() function.

The find_wall() function generates a Twist message that makes the robot move forward and turn left to find a wall to follow. The turn_left() function generates a Twist message that makes the robot turn left. The follow_the_wall() function generates a Twist message that makes the robot follow the wall.
 
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


def wall_follower_switch(req):
    """Callback function to activate or deactivate the wall follower behavior.

    Args:
        req: a ROS message that contains a boolean value indicating whether the wall follower should be activated or deactivated.

    Returns:
        a ROS message that contains a boolean value indicating whether the service call succeeded or failed and an informative message.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    """Callback function to process laser scan data and update the robot's current environment.

    Args:
        msg: a ROS message that contains laser scan data.

    Returns:
        None
    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    """Function to change the robot's state.

    Args:
        state: a string representing the new state.

    Returns:
        None
    """
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state



def take_action():
    """
    Determines the robot's action based on the current sensor readings and state.
    
    Parameters:
        None
    
    Returns:
        None
    """
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(1)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    """
    Move the robot forward and rotate it counterclockwise to find a wall to follow.

    Returns:
        geometry_msgs/Twist: The Twist message to move the robot forward and rotate it.
    """
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.6
    return msg


def turn_left():
    """
    Generates a Twist message to turn the robot left.

    :return: Twist message to turn left
    :rtype: geometry_msgs/Twist
    """
    msg = Twist()
    msg.angular.z = 0.8
    return msg



def follow_the_wall():
    """
    Generates a Twist message that makes the robot follow the wall.

    :return: Twist message with linear x velocity set to 0.5 and zero angular velocity.
    :rtype: geometry_msgs.msg.Twist
    """
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0
    return msg



def main():
    """
    Initializes a ROS node and sets up publishers, subscribers, services, and a loop that continuously executes as long
    as the ROS node is running. The loop checks the `active_` variable, which is set by a ROS service
    `wall_follower_switch` that toggles the state of the wall following algorithm. If `active_` is `True`, the function
    determines the current state of the algorithm (`state_`) and publishes the appropriate command to the
    `robot_base_velocity_controller/cmd_vel` topic via the `pub_` publisher. The three states of the algorithm are
    defined in three other functions: `find_wall()`, `turn_left()`, and `follow_the_wall()`. The loop is executed at a
    rate of 20 Hz using the `rospy.Rate` class.
    """
    global pub_, active_


    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('robot_base_velocity_controller/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
