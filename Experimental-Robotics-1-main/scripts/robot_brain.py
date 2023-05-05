#!/usr/bin/env python3
"""
.. module:: robot_brain.py
   :platform: Unix
   :synopsis: Script for the initialization of the robots required data.  
.. moduleauthor:: yeshwanth guru krishnakumar

Robot brain data for control and planning executed in the script .
 
"""
import random
import rospy
import threading
import smach_ros
import smach
from smach import StateMachine,State
from std_msgs.msg import Bool
from robot_brain2 import sensory2
from Expo_assignment_1 import architecture_name_mapper as anm
from expo_assignment_1.msg import Point, ControlGoal, PlanGoal
from actionlib import SimpleActionServer
from expo_assignment_1.msg import ControlFeedback, ControlResult
from expo_assignment_1.msg import Point, PlanFeedback, PlanResult
from expo_assignment_1.srv import SetPose,GetPoseResponse, SetPoseResponse
from expo_assignment_1.srv import GetPose

import expo_assignment_1 

LOOP_SLEEP_TIME = 0.5

LOG_TAG = anm.NODE_SURVILANCE_BRAIN


class ControllingAction:
    def __init__(self):
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER, expo_assignment_1.msg.ControlAction,
                                       execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        self.log_message(f'{anm.ACTION_CONTROLLER} Action Server initialized. It will navigate through the plan with a delay between each via point spanning in [{self.get_random_motion_time()[0]}, {self.get_random_motion_time()[1]}).')

    def execute_callback(self, goal):
        if not self.check_goal(goal):
            return
        feedback = ControlFeedback()
        self.log_message('Server is controlling...')
        self.navigate_through_points(goal.via_points, feedback)
        self.set_succeeded(feedback)

    def check_goal(self, goal):
        
        if not (goal and goal.via_points):
            self.log_message('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return False
        return True

    def navigate_through_points(self, points, feedback):
        for point in points:
            if self._as.is_preempt_requested():
                self.log_message('Service has been canceled by the client!')
                self._as.set_preempted()
                return
            delay = self.get_random_motion_time()[0] + random.random() * (self.get_random_motion_time()[1] - self.get_random_motion_time()[0])
            rospy.sleep(delay)
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            self.log_message(f'Reaching point ({point.x}, {point.y}).')
            self.set_current_robot_position(point)
        self.log_message('All points navigated.')

    def set_current_robot_position(self, point):
        self.log_message(f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.')
        try:
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)
        except rospy.ServiceException as e:
            self.log_message(f'Server cannot set current robot position: {e}')

    def set_succeeded(self, feedback):
        result = ControlResult()
        result.reached_point = feedback.reached_point
        self.log_message('Motion control successes.')
        self._as.set_succeeded(result)

    def get_random_motion_time(self):
        return rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])

    def log_message(self, message):
        rospy.loginfo(str(anm.tag_log(message, LOG_TAG)))
        
class PlanningAction(object):
    """
    A class that creates a motion plan with a number of points spanning in a given range and each point generated
    with a delay spanning in another given range. The plan is executed from a start point to a target point within a
    given environment size.

    :ivar _random_plan_points: A list of two integers indicating the minimum and maximum number of points in the motion
                               plan.
    :vartype _random_plan_points: list[int]
    :ivar _random_plan_time: A list of two floats indicating the minimum and maximum delay time between each point
                             generation in the motion plan.
    :vartype _random_plan_time: list[float]
    :ivar _environment_size: A list of two floats indicating the maximum x and y values of the environment where the
                             motion plan is executed.
    :vartype _environment_size: list[float]
    :ivar _as: A SimpleActionServer object that handles the motion planning action.
    :vartype _as: SimpleActionServer
    """
    def __init__(self):
        self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        self._as = SimpleActionServer(anm.ACTION_PLANNER, expo_assignment_1.msg.PlanAction,execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
                   f'spanning in [{self._random_plan_points[0]}, {self._random_plan_points[1]}). Each point will be generated '
                   f'with a delay spanning in [{self._random_plan_time[0]}, {self._random_plan_time[1]}).')

    def execute_callback(self, goal):
        start_point = _get_pose_client()
        target_point = goal.target
        if start_point is None or target_point is None:
            rospy.logerr(anm.tag_log('Cannot have `None` start point nor target_point. This service will be aborted!.', LOG_TAG))
            self._as.set_aborted()
            return

        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            rospy.logerr(anm.tag_log(f'Start point ({start_point.x}, {start_point.y}) or target point ({target_point.x}, \
                        {target_point.y}) point out of the environment. This service will be aborted!.', LOG_TAG))
            self._as.set_aborted()
            return

        feedback = PlanFeedback(via_points=[start_point])        
        self._as.publish_feedback(feedback)
        for i in range(random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)):
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
                self._as.set_preempted()
                return

            new_point = Point()
            new_point.x = random.uniform(0, self._environment_size[0])
            new_point.y = random.uniform(0, self._environment_size[1])
            feedback.via_points.append(new_point)

            if i < self._random_plan_points[1]:
                self._as.publish_feedback(feedback)
                rospy.sleep(random.uniform(self._random_plan_time[0], self._random_plan_time[1]))
            else:
                feedback.via_points.append(target_point)

        result = PlanResult(via_points=feedback.via_points)
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def _is_valid(self, point):
        return 0.0 <= point.x <= self._environment_size[0] and 0.0 <= point.y <= self._environment_size[1]


def _get_pose_client():     
    service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
    response = service()       
    pose = response.pose       
    rospy.loginfo(anm.tag_log(f'Retrieving current robot position from the `{anm.NODE_ROBOTS_CONDITION}` node as: ({pose.x}, {pose.y}).', LOG_TAG))
    return pose
        
class RobotState:
    """
    This class represents the state of the robot in terms of its position and battery status.
    """
    def __init__(self):
        """
        Constructor for the `RobotState` class.

        Initializes the `_pose` attribute to `None`, `_battery_low` attribute to `False`,
        and `_randomness` attribute to the value of `anm.PARAM_RANDOM_ACTIVE` parameter.
        If `_randomness` is `True`, it initializes `_random_battery_time` attribute to the value of
        `anm.PARAM_BATTERY_TIME` parameter. Then, it creates two services: `anm.SERVER_GET_POSE`
        and `anm.SERVER_SET_POSE`, which can be used to get and set the robot's position, respectively.
        It also starts a thread that continuously checks if the battery level is low or not and
        publishes the battery status to the `anm.TOPIC_BATTERY_LOW` topic.

        Args:
            None

        Returns:
            None
        """
        self._pose, self._battery_low, self._randomness = None, False, rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness: self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 60.0])
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        th = threading.Thread(target=self.is_battery_low_)
        th.start()
        log_msg = (f'Initialise node `{anm.NODE_ROBOTS_CONDITION}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')        
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        
    def set_pose(self, request):
        """
        Service callback method for setting the robot's position.

        Args:
            request: An object of `SetPoseRequest` type, which contains the new pose for the robot.

        Returns:
            A response object of `SetPoseResponse` type.
        """
        self._pose = request.pose if request.pose is not None else rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        return SetPoseResponse()
    
    def get_pose(self, request):
        """
        Service callback method for getting the robot's current position.

        Args:
            request: An object of `GetPoseRequest` type, which contains no arguments.

        Returns:
            A response object of `GetPoseResponse` type, which contains the current pose of the robot.
        """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        response = GetPoseResponse()
        response.pose = self._pose
        return response
    
    def is_battery_low_(self):
        """
        A helper method that continuously checks if the battery level is low or not and
        publishes the battery status to the `anm.TOPIC_BATTERY_LOW` topic.

        Args:
            None

        Returns:
            None
        """
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        self.battery_notifier_(publisher) if self._randomness else None
    
    def battery_notifier_(self, publisher):
        """
        A helper method that publishes the current battery status to the `publisher` object.

        Args:
            publisher: A `rospy.Publisher` object that is used to publish the battery status.

        Returns:
            None
        """
        delay = 0
        while not rospy.is_shutdown():
            publisher.publish(Bool(self._battery_low))
            if self._battery_low:
                print("Robot battery is low after",delay,"seconds")
            else:
                print("Robot battery is full",delay,"seconds")
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1]) if self._randomness else None
            rospy.sleep(delay)
            self._battery_low = not self._battery_low 
             
            
class ChooseCorridor(smach.State):
    """
    A state that chooses a random point in the environment and sends a planning goal to the planner action server to find a path to that point through via-points.

    Parameters:
        interface_cortex (CortexInterface): The interface with the other nodes of the architecture.
        ontology_cortex (CortexOntology): The ontology interface with knowledge of the environment and robot's current position.

    Outputs:
        robot_position (string): The robot's current position in the environment.
        target (string): The target location for the robot to navigate to.
        random_plan (list of Points): The via-points planned by the planner to reach the target location.

    Outcome:
        'Robot_recharging': The robot's battery is low and it needs to go recharge.
        'corridor_chosen': The planner has finished computing a plan through via-points to reach the target location.
        'urgentroom_chosen': The target location is a room that requires an urgent transition.

    """

    def __init__(self, interface_cortex, ontology_cortex):
        """
        Initializes the ChooseCorridor state.

        Args:
            interface_cortex (CortexInterface): The interface with the other nodes of the architecture.
            ontology_cortex (CortexOntology): The ontology interface with knowledge of the environment and robot's current position.
        """
        self._cortex = interface_cortex
        self._ontology = ontology_cortex
        self.environment_size = rospy.get_param('config/environment_size')
        smach.State.__init__(self, outcomes=['Robot_recharging', 'corridor_chosen', 'urgentroom_chosen'], output_keys=['robot_position', 'target', 'random_plan'])

    def execute(self, userdata):
        """
        Executes the ChooseCorridor state.

        Args:
            userdata (UserData): The data to be passed between states.

        Returns:
            string: The outcome of the state.

        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, self.environment_size[0]),
                             y=random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._cortex.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._cortex`.
            self._cortex.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._cortex.is_battery_low():  # Higher priority
                    self._cortex.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return 'Robot_recharging'
                # If the target is a Rooom then planner cancels goals and transition TRANS_DECIDED_URGENT occurs
                if target == 'Room-R1' or target == 'Room-R2' or target == 'Room-R3' or target == 'Room-R4':
                    self._cortex.planner_client.cancel_goals()
                    return 'urgentroom_chosen'
                # If the planner finishes its computation, then take the TRANS_DECIDED_CORRIDOR transition
                if self._cortex.planner_client.is_done():
                    userdata.random_plan = self._cortex.planner_client.get_results().via_points
                    return 'corridor_chosen'
            finally:
                # Release the mutex to unblock the `self._cortex` subscription threads if they are waiting.
                self._cortex.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._cortex` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)
           



class Navigationway(smach.State):
    """
    A SMACH state for moving the robot along a planned path.

    Args:
        interface_cortex (CortexInterface): An instance of the CortexInterface class.
        ontology_cortex (OntologyCortex): An instance of the OntologyCortex class.

    Input keys:
        random_plan (list): A list of PoseStamped messages representing the planned path.
        robot_position (PoseStamped): The current pose of the robot.
        target (str): The target location.

    Output keys:
        robot_position (PoseStamped): The new pose of the robot.

    Outcomes:
        Robot_recharging: The robot's battery is low and it needs to recharge.
        robot_moved_corridor: The robot has reached its target location.
    """

    def __init__(self, interface_cortex, ontology_cortex):
        """
        Initialize the Navigationway state.

        Args:
            interface_cortex (CortexInterface): An instance of the CortexInterface class.
            ontology_cortex (OntologyCortex): An instance of the OntologyCortex class.
        """
        super(Navigationway, self).__init__(
            outcomes=['Robot_recharging', 'robot_moved_corridor'], 
            input_keys=["random_plan",'robot_position', 'target'], 
            output_keys=['robot_position']
        )

        self._cortex = interface_cortex
        self._ontology = ontology_cortex

    def execute(self, userdata):
        """
        Execute the Navigationway state.

        Args:
            userdata (UserData): An object that contains input and output keys.

        Returns:
            str: The outcome of the state.
        """
        plan = userdata.random_plan
        goal = ControlGoal(via_points=plan)
        self._cortex.controller_client.send_goal(goal)

        while not rospy.is_shutdown():
            self._cortex.mutex.acquire()
            try:
                if self._cortex.is_battery_low():
                    self._cortex.controller_client.cancel_goals()
                    self._ontology.go_to_recharge(userdata.robot_position)
                    return 'Robot_recharging'
                
                if self._cortex.controller_client.is_done():
                    self._ontology.move_location(userdata.target, userdata.robot_position)
                    
                    rospy.sleep(anm.MONITOR_TIME)
                    return 'robot_moved_corridor'
            finally:
                self._cortex.mutex.release()
            
            rospy.sleep(LOOP_SLEEP_TIME)


class emergency(smach.State):
    """
    This state handles emergency situations in the robot navigation system. It generates a random goal to be reached 
    through some via-points planned by an action server. If the battery is low or the target is a corridor, the planner 
    goal is cancelled and the appropriate transition is taken. When the controller finishes its computation, the 
    'urgentroom_chosen' transition is taken.
    
    Required parameters:
    - interface_cortex: an object that provides interfaces with other nodes of the architecture
    - ontology_cortex: an object that provides the robot's knowledge of the environment
    
    Optional parameters:
    - None
    
    Input keys:
    - None
    
    Output keys:
    - robot_position: the robot's current position
    - target: the target to be reached
    - random_plan: the planned via-points to reach the random goal
    
    Outcomes:
    - 'Robot_recharging': if the battery is low and the robot needs to go to the charging station
    - 'urgentroom_chosen': when the controller finishes its computation
    - 'corridor_chosen': if the target is a corridor and the robot needs to go there
    
    """

    def __init__(self, interface_cortex, ontology_cortex):
        """
        Constructor for the emergency state.
        
        @param interface_cortex: an object that provides interfaces with other nodes of the architecture
        @type interface_cortex: object
        
        @param ontology_cortex: an object that provides the robot's knowledge of the environment
        @type ontology_cortex: object
        
        @return: None
        """
        super(emergency, self).__init__(
            outcomes=['Robot_recharging', 'urgentroom_chosen', 'corridor_chosen'],
            output_keys=['robot_position', 'target', 'random_plan']
        )

        # Get a reference to the interfaces with the other nodes of the architecture.
        self._cortex = interface_cortex
        self._ontology = ontology_cortex
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        

    def execute(self, userdata):
        """
        Execute function for the emergency state.
        
        @param userdata: data input/output that serves as a communication channel between states
        @type userdata: UserData
        
        @return: the outcome of the state machine (one of the defined outcomes)
        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, self.environment_size[0]),
                            y=random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._cortex.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._cortex`.
            self._cortex.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._cortex.is_battery_low():  # Higher priority
                    self._cortex.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return 'Robot_recharging'
                #if the target is a corridor then planner cancels goal and transition TRANS_DECIDED_CORRIDOR occurs
                if  target=='Room-C1' or target=='Room-C2' or target=='Room-E':
                    self._cortex.planner_client.cancel_goals()
                    return 'corridor_chosen'
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._cortex.planner_client.is_done():
                    userdata.random_plan = self._cortex.planner_client.get_results().via_points
                    return 'urgentroom_chosen'


            finally:
                # Release the mutex to unblock the `self._cortex` subscription threads if they are waiting.
                self._cortex.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._cortex` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)



class emergencylocation(smach.State):
    """
    This class defines a state in a SMACH state machine that sends a robot to a target location using a given plan. 
    If the robot's battery is low, the state will cancel the plan and send the robot to recharge. 
    
    :param interface_cortex: An instance of the interface_cortex class used to communicate with the robot's control system.
    :type interface_cortex: interface_cortex
    
    :param ontology_cortex: An instance of the ontology_cortex class used to interact with the robot's knowledge base.
    :type ontology_cortex: ontology_cortex
    
    :return: The outcome of the state. Either 'Robot_recharging' if the robot needs to recharge, or 'robot_moved_urgent' if the robot reaches the target.
    
    :raises: Any exceptions thrown by the underlying code.
    """

    def __init__(self, interface_cortex, ontology_cortex):
        """
        Initialize the state.
        
        :param interface_cortex: An instance of the interface_cortex class used to communicate with the robot's control system.
        :type interface_cortex: interface_cortex
        
        :param ontology_cortex: An instance of the ontology_cortex class used to interact with the robot's knowledge base.
        :type ontology_cortex: ontology_cortex
        """
        super(emergencylocation, self).__init__(
            outcomes=['Robot_recharging', 'robot_moved_urgent'],
            input_keys=['random_plan', 'robot_position', 'target'],
            output_keys=['crobot_position']
        )
        self.interface_cortex = interface_cortex
        self.ontology_cortex = ontology_cortex
        
    def execute(self, userdata):
        """
        Execute the state.
        
        :param userdata: The user data provided to the state.
        :type userdata: smach.UserData
        
        :return: The outcome of the state. Either 'Robot_recharging' if the robot needs to recharge, or 'robot_moved_urgent' if the robot reaches the target.
        """
        plan = userdata.random_plan
        goal = ControlGoal(via_points=plan)
        self.interface_cortex.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            with self.interface_cortex.mutex:
                if self.interface_cortex.is_battery_low(): 
                    self.interface_cortex.controller_client.cancel_goals()
                    self.ontology_cortex.go_to_recharge(robot_position)
                    return 'Robot_recharging'
                if self.interface_cortex.controller_client.is_done():
                    self.ontology_cortex.move_location(target, robot_position)
                    rospy.sleep(anm.MONITOR_TIME)
                    return 'robot_moved_urgent'

            rospy.sleep(LOOP_SLEEP_TIME)


class Recharging(State):
    """
    This class defines a state in a SMACH state machine that sends a robot to recharge if its battery is low. 
    
    :param interface_cortex: An instance of the interface_cortex class used to communicate with the robot's control system.
    :type interface_cortex: interface_cortex
    
    :param ontology_cortex: An instance of the ontology_cortex class used to interact with the robot's knowledge base.
    :type ontology_cortex: ontology_cortex
    
    :return: The outcome of the state, which is always 'recharged'.
    
    :raises: Any exceptions thrown by the underlying code.
    """

    def __init__(self, interface_cortex, ontology_cortex):
        """
        Initialize the state.
        
        :param interface_cortex: An instance of the interface_cortex class used to communicate with the robot's control system.
        :type interface_cortex: interface_cortex
        
        :param ontology_cortex: An instance of the ontology_cortex class used to interact with the robot's knowledge base.
        :type ontology_cortex: ontology_cortex
        """
        super().__init__(outcomes=['recharged'])
        self._cortex = interface_cortex
        self._ontology = ontology_cortex

    def execute(self, userdata):
        """
        Execute the state.
        
        :param userdata: The user data provided to the state.
        :type userdata: smach.UserData
        
        :return: The outcome of the state, which is always 'recharged'.
        """
        while not rospy.is_shutdown():
            self._cortex.mutex.acquire()
            try:
                if not self._cortex.is_battery_low():
                    self._cortex.reset_states()
                    return 'recharged'
            finally:
                self._cortex.mutex.release()
            rospy.sleep(LOOP_SLEEP_TIME)

if __name__ == '__main__':
    rospy.init_node(anm.NODE_SURVILANCE_BRAIN, log_level=rospy.INFO)
    server = ControllingAction()
    server = PlanningAction()
    # Instantiate the node manager class
    RobotState()

    # Define the cortex
    cortex = sensory2()

    # Initialize robot position
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    cortex.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    rospy.spin()
