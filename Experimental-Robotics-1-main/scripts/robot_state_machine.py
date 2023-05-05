#!/usr/bin/env python3
"""
.. module:: robot_state_machine.py
   :platform: Unix
   :synopsis: Script for the initialization of the statemachine.  
.. moduleauthor:: yeshwanth guru krishnakumar

This script is a ROS-based state machine for a robot to navigate through a topological map of an environment. It loads the ontology map of the environment, initializes the robot's sensors, and defines a state machine with three states: Load_Environment, Normal_mode, and Emergency_mode.

In Normal_mode state, the robot chooses the next corridor to move or switch to Emergency_mode. In Navigationway state, the robot navigates through the selected corridor. In Emergency_mode state, the robot navigates to the nearest emergency location.

The robot's sensors are instantiated in the main() function, and the state machine is initialized using the StateMachine() class. The transitions between states are defined using the add() and add_transition() methods of the StateMachine class. The script uses the ArmorClient library to manipulate the ontology map of the environment.

The script also defines a function to load the ontology map from a file and apply buffered changes to the ontology, and a function to save the ontology file when the script ends. The script uses the rospy library to initialize a ROS node and log information. The script uses the smach library to define and execute the state machine.
 
"""
import time
import os.path
import random

import smach
import rospy
import roslib
import smach_ros
import atexit
from std_msgs.msg import Bool
from std_msgs.msg import String
from smach import StateMachine, State
from robot_brain2 import sensory2, sensory1
from expo_assignment_1.msg import Point, ControlGoal, PlanGoal
from armor_api.armor_client import ArmorClient
from Expo_assignment_1 import architecture_name_mapper as anm
from robot_brain import ChooseCorridor, Navigationway, emergency, emergencylocation, Recharging

client = ArmorClient("armor_client", "survilance_ontology")

def load_topology_plot():
    path = os.path.dirname(os.path.realpath(__file__)) + "/../Topological_map/"
    ontology_url = "http://bnc/exp-rob-lab/2022-23"

    client.utils.load_ref_from_file(
        path + "topology_plot.owl", ontology_url, True, "PELLET", False, False
    )

    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()

    # Define a function to save the ontology file when the script ends
    @atexit.register
    def save_ontology_file():
        client.utils.save_ontology(path + "1.owl")


class LoadEnvironment(smach.State):
   

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

            print("ONTOLOGY MAP LOADED")
            return "map_loaded"

        # Call Ontology_Initialization() method and return its outcome
        return Ontology_Initialization()

def callback(data):
    if data.data == True:
        # Execute your script here
        main()

def main():

    # Load the map
    load_topology_plot()
    # Instantiate the robot's sensors
    onto = sensory1()
    cortex = sensory2()
    
    # Initialize the state machine
    sm_main = StateMachine(["Load_Environment"])

    with sm_main:
        # Load environment state
        StateMachine.add(
            "Load_Environment",
            LoadEnvironment(),
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
    # Initialize rospy node
    rospy.init_node("Robot_state_machine", log_level=rospy.INFO)
    rospy.Subscriber('/decision', Bool, callback)
    rospy.spin()
