#!/usr/bin/env python3
import time
import os.path
import random
from geometry_msgs.msg import Twist
import smach
import sys
import rospy
import roslib
import smach_ros
import atexit
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import actionlib
from smach import StateMachine, State
from robot_brain2 import sensory2, sensory1
from expo_assignment_1.msg import Point, ControlGoal, PlanGoal
from armor_api.armor_client import ArmorClient
from Expo_assignment_1 import architecture_name_mapper as anm
from robot_brain import ChooseCorridor, Navigationway, emergency, emergencylocation, Recharging
import moveit_commander
from moveit_commander import MoveGroupCommander
from std_msgs.msg import Bool


client = ArmorClient("armor_client", "survilance_ontology")

        
class ScanEnvironment(smach.State):
    def __init__(self):
        State.__init__(self, outcomes=['Environment_Scanned'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            moveit_commander.roscpp_initialize(sys.argv)
            arm = MoveGroupCommander('arm')

            poses = ['back_45', 'home', 'home_2', 'right_45', 'home3', 'down', 'down_left', 'down_right', 'horse',
                     'horse_left', 'horse_right', 'left_45']

            for pose in poses:
                arm.set_named_target(pose)
                arm.go()

            moveit_commander.roscpp_shutdown()
            return 'Environment_Scanned'

        
def load_topology_plot():
    path = os.path.dirname(os.path.realpath(__file__)) + "/../Topological_map/"
    ontology_url = "http://bnc/exp-rob-lab/2022-23"

    client.utils.load_ref_from_file(
        path + "topology_plot.owl", ontology_url, True, "PELLET", False, False
    )

    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()


class LoadEnvironment_1(smach.State):

    def __init__(self):
       
        smach.State.__init__(self, outcomes=["map_loaded"])

    def execute(self, userdata):
       
        def Ontology_Initialization():
          
            # Define the room locations, doors, and robot location
            objects = ["Room-E", "Room-C1", "Room-C2", "Room-R1", "Room-R2", "Room-R3", "Room-R4"]
            locations = ["Room-E", "Room-C1", "Room-C2", "Room-R1", "Room-R2", "Room-R3", "Room-R4"]
            doors = {
                "Room-E": {"Door-D5", "Door-D6"},
                "Room-C1": {"Door-D1", "Door-D2", "Door-D5"},
                "Room-C2": {"Door-D3", "Door-D4", "Door-D5", "Door-D6"},
                "Room-R1": {"Door-D1"},
                "Room-R2": {"Door-D2"},
                "Room-R3": {"Door-D3"},
                "Room-R4": {"Door-D4"},
            }

            # Add locations and doors to ontology map
            for location in locations:
                client.manipulation.add_ind_to_class(location, "LOCATION")
                for door in doors[location]:
                    client.manipulation.add_objectprop_to_ind("hasDoor", location, door)

            # Add robot location to ontology map
            client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "Room-E")

            # Add disjointness constraint to ontology map
            client.call(
                "DISJOINT",
                "IND",
                "",
                ["Room-E", "Room-C1", "Room-C2", "Room-R1", "Room-R2", "Room-R3", "Room-R4", "Door-D1", "Door-D2", "Door-D3", "Door-D4", "Door-D5", "Door-D6"],
            )

            # Add current time to visitedAt dataproperty of room locations
            _actual_time = str(int(time.time()))
            for r in ["Room-R1", "Room-R2", "Room-R3", "Room-R4"]:
                client.manipulation.add_dataprop_to_ind("visitedAt", r, "Long", _actual_time)

            # Apply buffered changes and sync buffered reasoner
            client.utils.apply_buffered_changes()
            client.utils.sync_buffered_reasoner()
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            my_goal=MoveBaseActionGoal()
            print("\nRobot is Going to Corridor E ")
            my_goal.goal.target_pose.header.frame_id = "odom";
            my_goal.goal.target_pose.pose.orientation.w = 1;
            my_goal.goal.target_pose.pose.position.x = 1.5
            my_goal.goal.target_pose.pose.position.y = 8.0
            client.wait_for_server()
            client.send_goal(my_goal.goal)
            client.wait_for_result()
            if  client.get_state()==GoalStatus.SUCCEEDED:
                print("ONTOLOGY MAP LOADED")
                return "map_loaded"
      
        # Call Ontology_Initialization() method and return its outcome
        return Ontology_Initialization()



def main():
    # Initialize rospy node
    rospy.init_node("Robot_state_machine", log_level=rospy.INFO)

    
    # Instantiate the robot's sensors
    onto = sensory1()
    cortex = sensory2()
    
    Map = "Load_Environment"
    # Initialize the state machine
    sm_main = StateMachine(["Load_Environment"])

    with sm_main:
        StateMachine.add("scan", ScanEnvironment(),
                         transitions = {"Environment_Scanned": "Load_Environment_1"})
        # Load environment state
        StateMachine.add(
            "Load_Environment_1",
            LoadEnvironment_1(),
            transitions={"map_loaded": "Normal_mode"},
        )

        # Normal mode state: Choose the next corridor to move or switch to emergency mode
        StateMachine.add(           
                         "Normal_mode",
            ChooseCorridor(cortex, onto),            
            transitions={
                "Robot_recharging": "Robot_recharging",
                "corridor_chosen": "Navigationway",
                "urgentroom_chosen": "Emergencyroom",
            },
        )

        # Navigationway state: Navigate the robot through the selected corridor
        StateMachine.add(
            "Navigationway",
            Navigationway(cortex, onto),
            transitions={
                "Robot_recharging": "Robot_recharging",
                "robot_moved_corridor": "Normal_mode"
            },
        )

        # Emergencyroom state: Choose the most urgent room to attend to or switch to normal mode
        StateMachine.add(
            "Emergencyroom",
            emergency(cortex, onto),
            transitions={
                "Robot_recharging": "Robot_recharging",
                "urgentroom_chosen": "Emergencylocation",
                "corridor_chosen": "Normal_mode",
            },
        )        
        # Emergencylocation state: Navigate the robot to the most urgent room
        StateMachine.add(
            "Emergencylocation",
            emergencylocation(cortex, onto),
            transitions={
                "Robot_recharging": "Robot_recharging",
                "robot_moved_urgent": "Emergencyroom",
            },
        )

        # Robot recharging state: Navigate the robot to the nearest charging station
        StateMachine.add(
            "Robot_recharging",
            Recharging(cortex, onto),
            transitions={"recharged": "Normal_mode"},
        )

    # Start the state machine's introspection server and execute the state machine
    sis = smach_ros.IntrospectionServer("sm_introspection", sm_main, "Surveillance Robot")
    sis.start()
    outcome = sm_main.execute()
    sis.stop()


if __name__ == "__main__":
    main()