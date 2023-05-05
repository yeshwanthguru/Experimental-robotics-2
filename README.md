 # 🕵️ Experimental Robotics - The  ROS-Based Survilance 🤖                                                               

<p align="center">
  <img src="https://user-images.githubusercontent.com/72270080/218885084-5f1795ba-0d94-4278-b860-9c5e90a1a5f7.png" width="80" height="80" alt="Mobile Robot Icon"/>
</p>


<p align="center">
  <a href="https://www.python.org"><img src="https://img.shields.io/badge/language-python-blue.svg"></a>
  <a href="http://wiki.ros.org/noetic"><img src="https://img.shields.io/badge/ROS-Noetic-FF6F00.svg"></a>
  <a href="https://github.com/yeshwanthguru/Experimental-Robotics-2/pulls"><img src="https://img.shields.io/github/issues-pr/yeshwanthguru/Experimental-Robotics-2"></a>
  <a href="https://github.com/yeshwanthguru/Experimental_robotics-2/blob/main/LICENSE"><img src="https://img.shields.io/github/license/yeshwanthguru/Experimental-Robotics-2"></a>
  <a href="https://github.com/yeshwanthguru/Experimental-Robotics-2"><img src="https://img.shields.io/github/repo-size/yeshwanthguru/Experimental-Robotics-2"></a>
  <a href="https://github.com/yeshwanthguru/Experimental-Robotics-2/commits/main"><img src="https://img.shields.io/github/last-commit/yeshwanthguru/Experimental-Robotics-2"></a>
</p>



🤖 Name: **Yeshwanth Guru Krishnakumar**

🔌 Reg No: **5059111**

📧 Email: **yeshwanth445@gmail.com**

📜 Check out the Experimental_assignment2_Documentation for my latest robotic project:


🤖 [Experimental_assignment2_Documentation](https://yeshwanthguru.github.io/Experimental-robotics-2/) 🚀

## 🤖 INTRODUCTION: 🎲

Welcome to my latest ROS project, an investigation-inspired scenario that demonstrates the power of ROS packages in robotics! This project features an interactive scenario like a ontology based survilance robot. Check out the image below for a sneak peek of ROS package:
   




https://user-images.githubusercontent.com/72270080/236406070-dcd52fc5-dad4-4d5e-a0a4-7eaad2af5081.mp4



           
## 🧱 SOFTWARE ARCHITECTURE: 🏗️

This project features a robust software architecture that utilizes a range of cutting-edge technologies and methodologies to create a seamless user experience. Check out the diagrams below to see the high-level structure of the system:

### 🔹 UML Diagram: 📊
<br><br>
![UMLDiagram drawio (3)](https://user-images.githubusercontent.com/72270080/236412392-6abf096c-087b-4e3d-adc1-a51ea9e8ce7f.png)


<br><br>

# 🚀 ARCHITECTURE WORKING PROCESS: 🔧

## ROBOT_STATE_MACHINE : 
🤖🗺️ This script is a ROS-based state machine for a robot to navigate through a topological map of an environment. It 📥 loads the ontology map of the environment 🌍, 🤖 initializes the robot's sensors 🎛️, and defines a state machine with three states: Load_Environment 📥, Normal_mode 🏃, and Emergency_mode 🚨.

In Normal_mode state, the robot chooses the next corridor to move 🏃 or switch to Emergency_mode 🚨. In Navigationway state, the robot navigates 🚶 through the selected corridor. In Emergency_mode state, the robot navigates to the nearest emergency location 🚑🚨.

The robot's sensors are instantiated in the main() function, and the state machine is initialized using the StateMachine() class. The transitions between states are defined using the add() and add_transition() methods of the StateMachine class. The script uses the ArmorClient library to manipulate the ontology map of the environment 🧐.

The script also defines a function to load the ontology map from a file 📁 and apply buffered changes to the ontology, and a function to save the ontology file when the script ends 💾. The script uses the rospy library to initialize a ROS node 🌐 and log information 📝. The script uses the smach library to define and execute the state machine 🤖. 📝🏃‍♂️🏥
## 🧠 ROBOT_BRAIN:
🤖 A robot 🤖 is following a plan consisting of random points 🧭. The plan is generated by a Planning Action Server 🗺️, and the robot's movement is controlled by a Controlling Action Server 🕹️. The robot navigates through the plan by reaching each point with a random delay ⏰. The delay is between two values provided as parameters to the Controlling Action Server.

The start and end points of the plan are set using a service called SetPose 📍. The server checks if the points are within the environment limits 🌳. The robot's current position is also set using SetPose 🔍. The system logs messages at each step to keep track of the robot's progress 📝.

The robot is on a mission to explore the environment and reach its destination safely 🚀. It moves through the environment, encountering obstacles and making decisions on the fly. The robot's sensors help it detect the environment and make decisions based on the data 🤖🔬.
## 🔮 ROBOT_BRAIN2:
🤖📨 This script defines a class ActionClientcortex that implements a SimpleActionClient to send and cancel goals to a specified service. It also defines a class sensory1 that acquires locations from a database using ArmorClient, determines the robot's current location 🗺️, chooses a target location based on the robot's position 📍, reachable locations 🚶, and urgency 🚨.

In the working scenario, the robot uses sensory1 to determine its current location and choose a target location 🎯. Then, the ActionClientcortex is used to send a control action to move the robot to the target location 🏃. The robot moves to the target location 🚀, and the process repeats until a stopping condition is met 🛑.

During the execution of the script, the system logs messages to keep track of the robot's progress 📝. The robot's position and the target location are updated at each iteration to ensure the robot reaches the desired location 📍🎯. The urgency 🚨 of the target location can affect the robot's movement speed, and the system is designed to handle emergency situations accordingly 💼.
## bug_m:
🤖🌟 This script is a super cool implementation of a bug algorithm for a mobile robot! 🤖👏

The algorithm allows the robot to navigate towards a desired position while avoiding obstacles in real-time using laser range data. 🚀🔬

What's even cooler is that it uses ROS (Robot Operating System) for communication between nodes, message passing, and service calls. 🤖💬 This makes it a powerful tool for autonomous navigation of robots in various settings, including warehouses, factories, and search and rescue operations. 🔍🏭🚒

With the use of this algorithm, robots can safely navigate around obstacles while reaching their target destination. 🤖🧭 And that's not all, it opens up a whole new world of possibilities for robotics and automation! 🤖🌎

Overall, this script showcases the amazing potential of robotics and how we can leverage technology to create intelligent systems that can help us in various fields. 🙌🤖
## NAV : 
🤖🚀 This script is perfect for controlling a mobile robot to move towards a given point in space in real-time applications! 🌟

The program has three states that work together to guide the robot towards the target position. 🤖💻

State 1: Rotate towards the goal position 🔄

The program calculates the error between the desired orientation and the robot's orientation.
Sends a Twist message to the robot to rotate in the direction that reduces the error.
The robot continues to rotate until it reaches an acceptable error represented by yaw_precision_2_.
State 2: Move straight ahead 🚶

The program calculates the error between the desired position and the robot's position.
Sends a Twist message to the robot to move forward in the direction that reduces the error.
The robot continues moving forward until it reaches an acceptable distance represented by dist_precision_.
State 3: Goal reached 🏁

The program sends a Twist message to stop the robot.
If the robot is too far from the goal position, the program returns to state 1 and starts again.
The program also listens to a service call to start and stop the robot's movement and a topic to receive the goal position. 📡👂

Running at a rate of 20 Hz, this program is perfect for autonomous navigation of robots in various real-time settings, such as warehouses, factories, and search and rescue operations. 🌎👨‍🚒👷‍♂️
## SEND GOAL TO ARM :
🦾🤖 This Python script is a powerful tool for controlling a robotic arm using the MoveIt! package and ROS. Here's how it works:

Initializes the ROS and MoveIt! nodes. 🚀🌟
This sets up the environment for controlling the robotic arm.
Instantiates a MoveGroupCommander object for the robotic arm. 🤖💻
This object allows the program to control the arm's movements.
Sets named targets for the arm to move to. 🎯👀
These targets represent specific positions for the arm to move to, such as "home", "left_45", "horse", etc.
Plans and executes a trajectory to reach each target. 🛣️🤖
The program uses the MoveIt! package to plan a trajectory for the arm to reach each target position and then executes the trajectory to move the arm.
Shuts down the MoveIt! and ROS nodes. 🔌👋
Once the arm has completed all movements, the program shuts down the MoveIt! and ROS nodes.
Initializes a new ROS node. 🚀🌟
This sets up a new node for the program to perform additional tasks.
Publishes a boolean message on the "/decision" topic. 📡💬
The program publishes a message to the "/decision" topic.
Sleeps for 1 second to wait for the publisher to initialize. 💤⏰
This ensures that the publisher has time to initialize before continuing with the program.
Sets the boolean message to True and publishes it. ✅📡
The program sets the message to True and publishes it to the "/decision" topic.
Signals to ROS that the program is done. 🛑🤖
The program signals to ROS that it has completed all tasks.
## WALL_FOLLOW : 
This Python script implements a wall-following behavior for a robot using laser sensors in ROS. 🤖👀🚶‍♂️ The script defines functions to control the robot's linear and angular velocities, and a class to send control actions to move the robot. The main() function initializes the ROS node, creates publishers, subscribers, and a service to switch the wall-follower on and off. The callback function clbk_laser() updates the distances to obstacles in different directions. The take_action() function determines the state of the robot based on the distances, calls the appropriate function to generate the Twist message, and changes the robot's state. The find_wall() function makes the robot move forward and turn left to find a wall. The turn_left() function makes the robot turn left. The follow_the_wall() function makes the robot follow the wall. 🏃‍♂️🔄🧱 
## 🛠️ Setup and Working Process:
🖥️ This project is developed using [Docker](https://hub.docker.com/r/carms84/exproblab) which has all the necessary dependencies installed. If you don't want to use Docker and prefer to install the dependencies manually, you'll need to install Armor, SMACH, and since this is a ROS Noetic based project. Once the dependencies are installed, you'll need to clone the following Git link to your workspace and load the OWL file as per the command described in the script, adding the path of the OWL file manually at your convenience.

                                             https://github.com/yeshwanthguru/Experimental-Robotics-2.git


in your workspace and load the owl file as per the command described in the script as your convenience manually adding the path of the owl file . Then do

                                                catkin_make


rocketOnce everything build.To execute the script with simulation do roslaunch command  and a python command in three different terminal in order.
                                         
                                         roslaunch expo_assignment_1 survailence_robot.launch
                                         roslaunch assignment2 assignment.launch
                                         python send_goal_to_arm.py

Once after the execution of the planning then a /decision topic is published to the statemachine for executing further process based on that further working takes place in the system.And moveit is used for  the motion planning where the following can beed found in the folder [ass2](https://github.com/yeshwanthguru/Experimental-robotics-2/tree/main/ass2)

## Moveit
here are the benefits of using MoveIt in ROS Noetic with even more emojis:

🚀 Simplified motion planning: MoveIt provides a simplified interface for motion planning that allows the user to specify a goal configuration and generate a trajectory to reach that configuration.

🤖 Support for various types of robots: MoveIt supports a wide range of robots, including manipulators, mobile robots, and humanoid robots, and provides a unified interface to control their motion.

🔌 Integration with ROS: MoveIt is designed to work seamlessly with ROS, allowing users to leverage other ROS packages and tools for perception, navigation, and control.

🧩 Modular design: MoveIt has a modular design, which makes it easy to add new capabilities or modify existing ones.

🎓 Support for multiple kinematic solvers: MoveIt provides support for multiple kinematic solvers, making it possible to switch between solvers based on the robot's capabilities or the application requirements.

🔍 Advanced collision detection: MoveIt provides advanced collision detection algorithms that can efficiently check for collisions between the robot and the environment.

📈 Trajectory optimization: MoveIt can optimize trajectories to minimize joint accelerations, joint velocities, or other criteria, resulting in smoother and more natural motion.

🎨 Visualization tools: MoveIt provides visualization tools that allow the user to visualize the robot's motion and the environment, making it easier to debug and validate the robot's behavior.

🎮 Integration with Gazebo: MoveIt can be integrated with Gazebo, a popular robot simulator, allowing users to simulate and test their motion planning algorithms in a virtual environment.

👨‍👩‍👧‍👦 Large user community: MoveIt has a large and active user community that provides support, documentation, and examples, making it easier for new users to get started and for experienced users to share their knowledge and best practices.
# Results
The following has been attached at initially gave the result with the simulation environment:


### System Limitations
🛑 Our system currently has a few limitations that users should be aware of. The ontology needs to be loaded manually and the possible hypotheses are limited.

### Improvements

🤖🗺️ This assignment is a promising starting point for a ROS-based state machine that enables a robot to navigate a topological map of an environment with ontology. However, several technical improvements can be made to enhance its capabilities, including:

🎛️ Sensor Fusion: By combining data from multiple sensors, such as cameras, lidars, and IMUs, the robot can create a more accurate map of the environment, improving its navigation and decision-making capabilities.

🧠 Machine Learning Integration: Incorporating machine learning algorithms can enable the robot to learn from its experiences and optimize its navigation strategy based on the environment it operates in.

🌐 Multi-Robot Coordination: By coordinating with other robots in the same environment, the robot can avoid collisions and optimize its path planning, ultimately improving overall efficiency.

📈 Online Map Updating: Updating the ontology map in real-time can help the robot adapt to changes in the environment, such as moving obstacles or dynamic objects, making its navigation more accurate and reliable.

🏥 Emergency Response Planning: Enhancing the emergency mode by incorporating more complex planning and response strategies can help the robot respond to emergency situations more efficiently, potentially saving lives. 🚑🚨


### ADITIONAL IMPROVEMENT INSPIRED 

🤖 By leveraging quantum-inspired algorithms, it is possible to implement autonomous navigation and decision-making capabilities in mobile robots, allowing them to operate in complex and dynamic environments with greater efficiency and accuracy. This can open up new possibilities for applications such as 🚨 search and rescue, 🏭 industrial automation, and more. With ongoing research and development in this area, the potential for quantum-inspired robotics is only set to 📈 grow in the future.It has been done with the integration of ros and qiskit.which was inspired from the 
http://www.quantum-robot.org/

where the following task can be executed by the decision making node [Decision-make node](https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_main/tiago_testmain2.py)


which has the single sensory integration for reference meanwhile the multi sensroy can be done based on the scenario that need to be achieved .
By making these improvements, we aim to provide a more attractive and user-friendly system for our users. Our goal is to help investigators to solve crimes quickly and efficiently, ultimately making our communities safer.This proposal is for radical architecture approch with bio inspired.

         

