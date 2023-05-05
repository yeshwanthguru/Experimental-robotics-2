#!/usr/bin/env python3



import rospy

# The name of the parameter to define the environment size.
# It should be a list [max_x, max_y] such that x:[0, max_x) and y:[0, max_y).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'
# ---------------------------------------------------------
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'

# The name of the node that sets/gets the pose of the robot and manages its battery.
NODE_ROBOTS_CONDITION = 'robots_condition'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'
# ---------------------------------------------------------

TOPIC_BATTERY_LEVEL = '/battery_level'



# Parameter indicating the sleep time [s]
SLEEP_TIME = 0.3

# Parameter indicating the battery time [s]
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'
# ---------------------------------------------------------
MONITOR_TIME = 5

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The number of points in the plan. It should be a list [min_n, max_n],
# Where the number of points is a random value in the interval [min_n, max_n).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'
# -------------------------------------------------


# The name of the controller node.
NODE_SURVILANCE_BRAIN = 'Survilancebrain'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via points.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'
# -------------------------------------------------


def tag_log(msg, producer_tag):
    """
	Function used to label each log with a producer tag.
	
    Args:
        msg(Str): message that will be visualized
        producer_tag(Str): tag identifying the log producer
            
    Returns:
        log_msg(Str): message for the log
    """

    return f'@{producer_tag}>> {msg}'
