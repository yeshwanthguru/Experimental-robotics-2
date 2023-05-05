#! /usr/bin/env python
"""
.. module:: bug_m.py
   :platform: Unix
   :synopsis: Script for the bug algorithm of the survilance robot.  
.. moduleauthor:: yeshwanth guru krishnakumar

This script is an implementation of a bug algorithm for a mobile robot to navigate towards a desired position while avoiding obstacles in real-time using laser range data. It demonstrates the use of ROS (Robot Operating System) for communication between nodes, message passing, and service calls. Such algorithms can be useful for autonomous navigation of robots in various settings, including warehouses, factories, and search and rescue operations.
 
"""
import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math

pub = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'target reached']
state_ = 0
# 0 - go to point
# 1 - wall following

# callbacks


def clbk_odom(msg):
    """
    Callback function to receive and process the odometry data.

    Parameters:
    -----------
    msg: nav_msgs/Odometry
        Odometry data message containing the robot's position and orientation.

    Returns:
    --------
    None

    """
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    """
    Callback function to receive and process the laser scan data.

    Parameters:
    -----------
    msg: sensor_msgs/LaserScan
        Laser scan data message containing the range measurements.

    Returns:
    --------
    None

    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }



def change_state(state):
    """
    This function changes the current state of the robot. It takes one argument:

    state: an integer value representing the new state to be set.
    Global variables used:

    state_: integer representing the current state of the robot.
    state_desc_: a dictionary containing descriptions of each state.
    srv_client_wall_follower_: a ROS service client for wall following service.
    srv_client_go_to_point_: a ROS service client for go to point service.
    If the new state is 0, the go to point service is set to true and wall following service is set to false.
    If the new state is 1, the go to point service is set to false and wall following service is set to true.
    If the new state is 2, both go to point and wall following services are set to false, and the robot's linear and angular velocity is set to zero. Finally, the user interface service is called.
    """
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        resp = srv_client_user_interface_()


def normalize_angle(angle):
    
    '''
    This function normalizes the given angle to be within the range of -pi to +pi radians.
    If the absolute value of the angle is greater than pi, it subtracts 2*pi times the angle divided by its absolute value.
    The normalized angle is then returned.

    Input:
    - angle: float value representing an angle in radians

    Output:
    - angle: normalized angle in the range of -pi to +pi radians
    '''
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    """
    Main function that initializes the ROS node, sets up subscribers, publishers and service clients,
    and runs the control loop to control the robot's movement.
    """
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('robot_base_velocity_controller/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('robot_base_velocity_controller/cmd_vel', Twist, queue_size=1)

    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)

    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if state_ == 0:
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
            if(err_pos < 0.3):
                change_state(2)

            elif regions_['front'] < 0.5:
                change_state(1)

        elif state_ == 1:
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))

            if(err_pos < 0.3):
                change_state(2)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)

        elif state_ == 2:
            desired_position_.x = rospy.get_param('des_pos_x')
            desired_position_.y = rospy.get_param('des_pos_y')
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
            if(err_pos > 0.35):
                change_state(0)

        rate.sleep()


if __name__ == "__main__":
    main()
