#! /usr/bin/env python
"""
.. module:: nav.py
   :platform: Unix
   :synopsis: Script for the survilance robot navigation.  
.. moduleauthor:: yeshwanth guru krishnakumar
This script is for controlling a mobile robot to move towards a given point in space. The robot's position is obtained through a topic, and its orientation is obtained from the same topic. The robot moves towards the target position by rotating towards it and then moving straight ahead. The program contains three states:

Rotate towards the goal position
Move straight ahead
Goal reached
The robot will start rotating towards the goal position in state 1. In this state, the program will calculate the error between the desired orientation and the robot's orientation. It will then send a Twist message to the robot to rotate in the direction that reduces the error. The robot will continue to rotate until it reaches the yaw_precision_2_ (a small value that represents an acceptable error).

After reaching the acceptable error, the robot will transition to state 2. In this state, the program will calculate the error between the desired position and the robot's position. It will then send a Twist message to the robot to move forward in the direction that reduces the error. The robot will continue moving forward until it reaches the dist_precision_ (a small value that represents an acceptable distance).

After reaching the acceptable distance, the robot will transition to state 3. In this state, the program will send a Twist message to stop the robot. If the robot is too far from the goal position, the program will return to state 1, and the process will start again.

The program listens to a service call to start and stop the robot's movement. It also listens to a topic to receive the goal position. The program will run at a rate of 20 Hz.
"""
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False


yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.3

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# service callbacks

def target(msg):
    """
    Sets the target position for the robot.
    
    Parameters:
        - msg (Point): The target position as a Point object.
    
    Returns: None
    """
    global position
    position = msg

    
def go_to_point_switch(req):
    """
    Service callback function to activate or deactivate the robot's movement.

    Parameters:
        - req (SetBoolRequest): A boolean request indicating whether to activate or deactivate the robot's movement.

    Returns: 
        - res (SetBoolResponse): A boolean response indicating whether the operation was successful.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks


def clbk_odom(msg):
    """
    Callback function for the robot's odometry data.

    Parameters:
        msg (nav_msgs/Odometry): The Odometry message received by the robot.

    Global Variables:
        position_ (geometry_msgs/Point): The position of the robot.
        yaw_ (float): The yaw angle of the robot.

    Returns:
        None
    """
    global position_
    global yaw_

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



def change_state(state):
    """
    Changes the state of the robot.

    :param state: An integer representing the new state of the robot.
    """
    global state_
    state_ = state
    print('State changed to [%s]' % state_)



def normalize_angle(angle):
    """
    Normalizes an angle to be between -pi and pi.

    :param angle: The angle to be normalized.
    :return: The normalized angle.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle



def fix_yaw(des_pos):
    """
    Adjusts the robot's orientation to face the given position.

    :param des_pos: A Point object representing the desired position to face.
    :type des_pos: geometry_msgs.msg.Point
    """
    global yaw_, pub, yaw_precision_2_, state_

    # Calculate the desired yaw angle
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)

    # Calculate the error between the desired and actual yaw angles
    err_yaw = normalize_angle(desired_yaw - yaw_)

    # Log the error
    rospy.loginfo(err_yaw)

    # Create a Twist message to send to the robot
    twist_msg = Twist()

    # If the error is greater than the yaw precision threshold, adjust the robot's orientation
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    # Publish the Twist message
    pub.publish(twist_msg)

    # Check if the error is within the yaw precision threshold and change the state accordingly
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)



def go_straight_ahead(des_pos):
    """
    Controls the robot to go straight ahead towards a desired position.

    Args:
        des_pos (Point): The desired position to move towards.

    Returns:
        None
    """

    global yaw_, pub, yaw_precision_, state_

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d*(err_pos)

        # Ensure the linear velocity is within bounds
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        rospy.loginfo('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        rospy.loginfo('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done(des_pos):
    """
    Stops the robot and checks if it has reached the desired position. If the robot is too far from the desired position,
    it changes the state to 0 (fix yaw), otherwise it stays in state 3 (done).

    Args:
        des_pos (Point): The desired position to reach.

    Returns:
        None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    if(err_pos > 0.35):
        change_state(0)



def main():
    """
    The main function that initializes the ROS node 'go_to_point', creates publishers and subscribers, and runs the control loop.

    Subscribers:
        - '/robot_base_velocity_controller/odom' (Odometry): callback function is clbk_odom
        - '/target_topic' (Odometry): callback function is target

    Publishers:
        - '/robot_base_velocity_controller/cmd_vel' (Twist)

    Services:
        - 'go_to_point_switch' (SetBool): callback function is go_to_point_switch

    Parameters:
        - 'des_pos_x': x-coordinate of the desired position
        - 'des_pos_y': y-coordinate of the desired position

    Control Loop:
        The control loop runs at a rate of 20 Hz. It first gets the desired position from ROS parameters. If the control mode is active, it executes the current state of the state machine:
        - State 0: Fixes the yaw angle of the robot towards the desired position using the function fix_yaw()
        - State 1: Moves the robot towards the desired position while maintaining the yaw angle using the function go_straight_ahead()
        - State 2: Stops the robot when it reaches the desired position using the function done()
        If the control mode is not active, the function just continues without executing any state.

    Returns:
        None
    """

    global pub, active_, desired_position_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/robot_base_velocity_controller/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/robot_base_velocity_controller/odom', Odometry, clbk_odom)
    tar_sub_odom = rospy.Subscriber('/target_topic', Odometry, target)
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        desired_position_.x = rospy.get_param('des_pos_x')
        desired_position_.y = rospy.get_param('des_pos_y')
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done(desired_position_)
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
    reach_pub = rospy.Publisher('/reached', Bool, queue_size=1)
    
