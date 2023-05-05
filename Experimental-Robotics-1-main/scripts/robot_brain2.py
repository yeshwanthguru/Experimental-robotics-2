#!/usr/bin/env python3
"""
.. module:: robot_brain2.py
   :platform: Unix
   :synopsis: Script for the initialization of the robots required data.  
.. moduleauthor:: yeshwanth guru krishnakumar

This script defines a class ActionClientcortex that implements a SimpleActionClient to send and cancel goals to a specified service. It also defines a class sensory1 that acquires locations from a database using ArmorClient, determines the robot's current location, and chooses a target location based on the robot's position, reachable locations, and urgency.

In the working scenario, the robot uses sensory1 to determine its current location and choose a target location. Then, the ActionClientcortex is used to send a control action to move the robot to the target location. The robot moves to the target location, and the process repeats until a stopping condition is met.
 
"""
import rospy
import random
import time
import threading
import re
from std_msgs.msg import String
from actionlib import SimpleActionClient
from threading import Lock
from std_msgs.msg import Bool
from expo_assignment_1.msg import PlanAction, ControlAction
from expo_assignment_1.srv import SetPose

from armor_api.armor_client import ArmorClient
from Expo_assignment_1 import architecture_name_mapper as anm

client = ArmorClient("armor_client", "survilance_ontology")


class ActionClientcortex:
    """
    A simple wrapper for ROS action client to send and receive goals.

    Args:
        service_name (str): The name of the action service to connect to.
        action_type (Any): The type of the action message.
        done_callback (Optional[Callable[[int, Any]], None]]): The callback function to be called when the action is done.
            It takes two arguments: the status code (int) and the result (Any). Defaults to None.
        feedback_callback (Optional[Callable[[Any], None]]): The callback function to be called when there is feedback from the action.
            It takes one argument: the feedback message (Any). Defaults to None.
        mutex (Optional[Lock]): The mutex to lock when accessing class variables. Defaults to None.

    Attributes:
        _is_running (bool): A boolean value indicating whether the action is currently running.
        _is_done (bool): A boolean value indicating whether the action is done.
        _results (Any): The results of the action.
        _service_name (str): The name of the action service to connect to.
        _mutex (Lock): The mutex to lock when accessing class variables.
        _client (SimpleActionClient): The SimpleActionClient object used to communicate with the action service.
        _external_done_cb (Optional[Callable[[int, Any], None]]): The external done callback function.
        _external_feedback_cb (Optional[Callable[[Any], None]]): The external feedback callback function.
    """

    def __init__(self, service_name: str, action_type, done_callback=None, feedback_callback=None, mutex=None):
        """
        Initializes the ActionClientcortex object.

        Args:
            service_name (str): The name of the action service to connect to.
            action_type (Any): The type of the action message.
            done_callback (Optional[Callable[[int, Any]], None]]): The callback function to be called when the action is done.
                It takes two arguments: the status code (int) and the result (Any). Defaults to None.
            feedback_callback (Optional[Callable[[Any], None]]): The callback function to be called when there is feedback from the action.
                It takes one argument: the feedback message (Any). Defaults to None.
            mutex (Optional[Lock]): The mutex to lock when accessing class variables. Defaults to None.
        """
        self._is_running = False
        self._is_done = False
        self._results = None
        self._service_name = service_name
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        self._client = SimpleActionClient(service_name, action_type)
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback        
        self._client.wait_for_server()

    def send_goal(self, goal) -> None:
        """
        Sends a goal to the action service.

        Args:
            goal (Any): The goal message to send.
        """
        if not self._is_running:
            self._client.send_goal(goal,
                                   done_cb = self.done_callback_,
                                   feedback_cb = self.feedback_callback_)            
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            rospy.logwarn("Cannot send a new goal, cancel the current request first")

    def cancel_goals(self) -> None:
        """
        Cancels all goals sent to the action service.
        """
        if self._is_running:
            self._client.cancel_all_goals()
            self.reset_client_states()
        else:
            rospy.logwarn("Cannot cancel a not running service")

    def reset_client_states(self) -> None:
        """
        Resets the client states.
        """
        self._is_running = False
        self._is_done = False
        self._results = None

    def feedback_callback_(self, feedback) -> None:
        """
        The internal feedback callback function.

        Args:
            feedback (Any): The feedback message received from the action service.
        """
        self._mutex.acquire()
        try:
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            self._mutex.release()

    def done_callback_(self, status, results) -> None:
        """
        The internal done callback function.

        Args:
            status (int): The status code of the action.
            results (Any): The result message received from the action service.
        """
        self._mutex.acquire()
        try:
            self._is_running = False
            self._is_done = True
            self._results = results
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self) -> bool:
        """
        Returns a boolean value indicating whether the action is done.

        Returns:
            bool: True if the action is done, False otherwise.
        """
        return self._is_done

    def is_running(self) -> bool:
        """
        Returns a boolean value indicating whether the action is currently running.

        Returns:
            A boolean value indicating whether the action is currently running.
        """
        return self._is_running
    
    def get_results(self):
        """
        Returns the results of the action if it has completed, otherwise logs an error message.

        Returns:
            The results of the action if it has completed.
        """
        if self._is_done:
            return self._results
        else:
            rospy.logerr("Cannot get result, service not done yet")

class sensory1:
    def __init__(self):       
        self.robot = "Robot1"
        self.charging_room = "Room-E"       
        self.urgent = 'urgent'
        self.corridor = 'corridor'
        self.robot_position = 'robot_position'
        self.reachable_locations = 'reachable_locations' 

    def list_formatter(self, old_location, start, end):
        """
        Formats a list of locations by extracting the substring between the start and end characters.
        
        Args:
            old_location (list): A list of strings to be formatted.
            start (str): The starting character of the substring to be extracted.
            end (str): The ending character of the substring to be extracted.
        
        Returns:
            list: A list of formatted strings.
        """
        new_location = []
        for location in old_location:
            new_location.append(re.search(start + '(.+?)' + end, location).group(1))
        return new_location	

    def location_acquire(self, location):
        """
        Acquires information about the robot's current location, reachable locations, corridors, or urgent rooms.
        
        Args:
            location (str): A string indicating the type of location information to acquire.
        
        Returns:
            list: A list of locations meeting the specified criteria.
        """
        if location == self.corridor:
            corridors_list = self.list_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
            return corridors_list
        elif location == self.urgent:
            urgent_rooms = self.list_formatter(client.query.ind_b2_class('URGENT'), '#', '>')
            return urgent_rooms
        elif location == self.robot_position:
            current_position = self.list_formatter(client.query.objectprop_b2_ind('isIn', self.robot), '#', '>')[0]
            return current_position
        elif location == self.reachable_locations:
            reachable_destinations =  self.list_formatter(client.query.objectprop_b2_ind('canReach',self.robot), '#', '>')
            return reachable_destinations
        else:
            return []

    def get_coordinate(self, location):
        """
        Gets the coordinate of a given location.
        
        Args:
            location (str): The location whose coordinate to retrieve.
        
        Returns:
            tuple: A tuple containing the x and y coordinates of the specified location.
        """
        coordinate_dict = {
            'Room-E': (0, 0),
            'Room-C1': (1, 1),
            'Room-C2': (-1, 1),
            'Room-R1': (2, 1),
            'Room-R2': (2, 2),
            'Room-R3': (-2, 1),
            'Room-R4': (-2, 2)
        }
        return coordinate_dict[location]

    def choose_target(self):
        """
        Chooses a target location for the robot to move to based on its current position and reachable locations.
        
        Returns:
            tuple: A tuple containing the robot's current position and its target destination.
        """
        corridors_list = self.location_acquire(self.corridor)
        current_position = self.location_acquire(self.robot_position)
        rospy.loginfo(f"Current position of robot is: [{current_position}]")
        coordinate = self.get_coordinate(current_position)
        rospy.loginfo(f"Coordinate is: {coordinate}")
        possible_destinations = self.location_acquire(self.reachable_locations)       
        urgent_rooms = self.location_acquire(self.urgent)
        ReachableUrgent_room = [i for i in possible_destinations if i in urgent_rooms]
        if not ReachableUrgent_room:
            ReachableCorridor_room = [i for i in possible_destinations if i in corridors_list]
            if not ReachableCorridor_room:
                target = random.choice(possible_destinations)
            else:
                target = random.choice(ReachableCorridor_room)
        else:
            # save the first element of the list as the oldest timestamp
            oldest = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[0])
            oldest = str(self.list_formatter(oldest, '"', '"')[0])
            for i in range(len(ReachableUrgent_room)):                
                choice_last_visit = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[i])
                choice_last_visit = str(self.list_formatter(choice_last_visit, '"', '"')[0])
                if choice_last_visit <= oldest:
                    target = ReachableUrgent_room[i]
        rospy.loginfo(f"Target to be reached {target}")
        return current_position, target
    def move_location(self, target, current_position):
        """
        Moves the robot to a new location.

        Publishes the target and current position to their respective topics, updates the robot's location
         in the knowledge base, and synchronizes the changes with the buffered reasoner.

        :param target: The target location for the robot.
        :param current_position: The current position of the robot.
        :return: None
        """
        # Publish target
        target_pub = rospy.Publisher('target_topic', String, queue_size=10)
        target_pub.publish(target)

        # Publish current_position
        pos_pub = rospy.Publisher('current_position_topic', String, queue_size=10)
        pos_pub.publish(current_position)
        
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, target, current_position)
        rospy.loginfo(f"Robot in {target} and monitoring")
        last_change = client.query.dataprop_b2_ind('now', self.robot)
        last_change = str(self.list_formatter(last_change, '"', '"')[0])
        current_time = str(int(time.time()))
        client.manipulation.replace_dataprop_b2_ind('now', self.robot, 'Long', current_time, last_change)
        corridors = self.location_acquire(self.corridor)
        if target not in corridors:
            last_visit = client.query.dataprop_b2_ind('visitedAt', target)
            last_visit =  str(self.list_formatter(last_visit, '"', '"')[0])
            client.manipulation.replace_dataprop_b2_ind('visitedAt', target, 'Long', current_time, last_visit)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    def go_to_recharge(self, robot_location):
        """
        Moves the robot to the charging room for recharge.

        Uses the `client.manipulation.replace_objectprop_b2_ind` method to update the `isIn` property
        of the robot to the charging room, and then synchronizes the changes with the buffered reasoner.

        :param robot_location: The current location of the robot.
        :return: None
        """
        rospy.loginfo("Robot's Battery is low and going to recharge room for recharge ")
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, self.charging_room, robot_location)
        client.utils.sync_buffered_reasoner()

class sensory2:
    """
    A class for managing sensor data and robot state.
    """

    def __init__(self):
        """
        Constructor method for the `sensory2` class.

        Initializes class attributes and subscribes to relevant ROS topics.

        :return: None
        """
        self.mutex = Lock()
        self.reset_states()

        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback_)
        self.planner_client = ActionClientcortex(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientcortex(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def reset_states(self) -> None:
        """
        Resets the class attributes to their initial state.

        :return: None
        """
        self._battery_low = False

    def battery_callback_(self, msg: Bool) -> None:
        """
        Callback function for the `anm.TOPIC_BATTERY_LOW` topic.

        Updates the `_battery_low` attribute with the received message data.

        :param msg: A ROS `Bool` message containing the battery status.
        :return: None
        """
        self.mutex.acquire()
        try:
            self._battery_low = msg.data
        finally:
            self.mutex.release()

    def is_battery_low(self) -> bool:
        """
        Returns the current battery status.

        :return: A boolean indicating whether the battery is low or not.
        """
        return self._battery_low

    def init_robot_pose(self, point) -> None:
        """
        Initializes the robot's position using a ROS service call.

        :param point: A `geometry_msgs.Point` message containing the initial robot position.
        :return: None
        """
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)
            rospy.loginfo("Setting initial robot position")
        except rospy.ServiceException as e:
            rospy.logerr("Cannot set current robot position: {}".format(e))



if __name__ == '__main__':   
    rospy.init_node('interface_cortex', anonymous=True)

    interface_cortex = sensory2()
    ontology = sensory1()
    while not rospy.is_shutdown():
        pass

    rospy.signal_shutdown('Node shut down')
