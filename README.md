Research Track 2 - Assignment
================================

Code Documentation
------------
The Doxygen-generated project documentation may be obtained at the following website:

[__samuelepedrazzi.github.io/RT2_Assignment/index.html__](https://samuelepedrazzi.github.io/RT2_Assignment/index.html)

Notebook Jupyter
------------
For the jupyter notebook it is available in the following file:[__Assignment.ipynb__](https://github.com/samuelepedrazzi/RT2_Assignment/blob/master/Assignment.ipynb)

Statistics Analysis
------------
The results of the statistics analysis for the first assignment of Research Track 1, computed with Matlab, can be found in the [__Report__](https://github.com/samuelepedrazzi/RT2_Assignment/blob/master/Statistics_Analysis.pdf).




Introduction <img src= "https://media2.giphy.com/media/fLsd17IO7HTCR85bDY/giphy.gif?cid=ecf05e47y9qetlendf7s1str4q1hzuzdr4ykg086vprnnccc&rid=giphy.gif&ct=s" width=100 height=60>
------------

>This is a Gazebo and Rviz-based ROS Robotics Simulator.
The goal of this project is to build three mobility modes that the user could choose from to allow the robot to explore an unknown but constrained territory inside the simulation.
On a user-interface terminal window, the user can pick the robot's desired behavior; the modalities include:

* __1 - Autonomous Drive__: The user can select a goal position for the robot, and it will reach there on its own.
* __2 - Free Drive__: The user can use the keyboard to drive the robot in the environment.
* __3 - Driver Assistant__: The operator can direct the robot's movement with the keyboard, but an collision avoidance algorithm will keep the robot from crashing with walls.

Installing and Running <img src="https://media0.giphy.com/media/XqYKfpjBL2bjUcWVQD/200w.webp?cid=ecf05e47f0avfktds1q4ksjx91r0uw1m2unss4u1btdrzy12&rid=200w.webp&ct=s" width="50"></h2>
--------

This simulator is built on the [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) platform, specifically the NOETIC version.

For the specific project the program requires the installation of the following packages and tools before it can begin:

* [Slam Gmapping package](https://github.com/CarmineD8/slam_gmapping)

which can be installed with the bash command:

```bash
	$ git clone https://github.com/CarmineD8/slam_gmapping.git
```

* xterm
	
Easy to install with:

```bash
	$ sudo apt-get install -y xterm
```
 
* Ros navigation stack
	
To Install it:

```bash
	$ sudo apt-get install ros-<ros_distro>-navigation
```

Before running the simulation, first you have to run ROS (using ```$ roscore &``` and ```$ catkin_make``` ), then the simulation begins when the user has all of the required packages and runs a .launch file called:

__Assignment3_RT1.launch__


```console
<?xml version="1.0"?>

<launch>
  <include file="$(find Assignment3_RT1)/launch/simulation_gmapping.launch"/>
  <include file="$(find Assignment3_RT1)/launch/move_base.launch"/>
 
 <!-- Run the UI node -->
  <node pkg="Assignment3_RT1" type="UI" respawn="false" name="UI" output="screen" launch-prefix="xterm -e" required="true"/>
 
</launch>
```

## Environment and mapping

Rviz (a 3D visualizer for the Robot Operating System (ROS) framework) and Gazebo (an open-source 3D Robotics simulator) appear on the screen as soon as the simulator starts:
Thanks to its sensors, the robot can see what's going on in the world around it.

ROS creates the environment described in the __world__ folder's file `house.world.`

The robot moves in the ambience in the figure (Gazebo view):

<p align="center">
    
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/Gazebo_map.png" width="600" height="350">
    
</p>

The robot knows only the portion of the surroundings that its sensors allow it to detect from its starting point because it does not know all of the map's boundaries.

The robot's map knowledge grows as it moves around the map. The robot's known surroundings is graphed and updated at each time instant on Rviz.

<p align="center">
	
Rviz map not explored      |  Rviz map explored 
:-------------------------:|:-------------------------:
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map.png" width="400" height="550"> |  <img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rviz_map_explored.png" width="400" height="550">

</p>

User-Interface <img src="https://media4.giphy.com/media/o8QCgJacJR5balxq8Y/200w.webp?cid=ecf05e47v2cy2r25cqior5ftits1w4lipka50hjfqkj4jhz4&rid=200w.webp&ct=s" width="40"></h2>
--------------

This is the primary node, and it was the first to appear.

This node shows the user a little image that explains how to choose the robot's movement modes. It also manages user input by modifying the ros parameters that allow the activation of the nodes defined for each mode if the command is correct.

The following are some of the commands that can be used:

<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/ui.png" width="450" height="450">

Depending on the user input it select the correct action to do, such as running one of the nodes,

```cpp
system("rosrun Assignment3_RT1 achieveGoalPosition"),
```

close the program (exiting from the main function) or reset the simulation with 

```cpp
ros::service::call("/gazebo/reset_simulation", reset)
```

For the nodes regarding the movement, which is a dynamic action, I choose to use a __non-blocking function__ to get the user input, which is good for speeding up program execution and improving consumer experience (you don't have to press enter key every time).
For the node concerning the setting and achievement of the preset coordinates I did not consider it necessary.

The repository I found on Github and changed a little bit for my purposes is from `kbNonBlock` at [teleop_twist_keyboard_repo](https://gist.github.com/whyrusleeping/3983293).

Achieve Goal Position node <img src= "https://cdn-icons-png.flaticon.com/128/854/854894.png" width=40>
--------------

The first needed feature is implemented by the achieveGoalPosition node. In fact, it gives the robot a new goal based on the user's preferences.
This node's goal is to drive the robot into the correct location in the environment once the position coordinates have been specified. 
At first the user is asked for the goal's x and y coordinates, after that the program generates and publishes a message of type `move_base_msgs/MoveBaseActionGoal` in the `/move_base/goal` topic. The node keeps track of each objective by assigning it an id that is generated at random within the node.

A `/move_base/status` message handler is used to determine whether the robot has reached the goal. It examines the messages that have been published on the previously indicated subject.

The initial status code is '__1__', which indicates that the robot is on his way and the goal is active.
When the robot comes to a halt, the status code changes to '__3__' if the robot has reached the goal position, and to '__4__' if the robot is unable to reach the given location.
Other statuses that have been managed are the goal lost with status identifier '__5__' or the rejected status with the code '__9__'.

After having received the feedback of the status and the robot is stopped, so there isn't any active pending goal, the function `CancelGoal()` is called, a message of type `actionlib_msgs/GoalID` is generated and then published into the `/move_base/cancel` topic.

Finally the user can choose to continue and set another or to quit to the user interface.

User Drive Not Assisted node
--------------


In this node the user can simply control the robot movement with the keypad (remember to click on BLOC NUM, otherwise the correct ascii code will not be read):

<center>

|| Turn left | Do not turn | Turn right|							
|:--------:|:--------:|:----------:|:----------:|
|__Go forward__|`7`|`8`|`9`
|__Stop__|`4`|`5`|`6`
|__Go backward__|`1`|`2`|`3`

</center>


The user can change the robot velocity of 10% by pressing the following keys:

<center>

|| Linear and Angular | Linear only | Angular only|
|:--------:|:--------:|:----------:|:----------:|
|__Increment__|`*`|`+`|`a`
|__Reset__|`R or r`|`e`|`w`
|__Decrease__|`/`|`-`|`z`

</center>

The speed is calculated by multiplying the relative speed by the selected direction, which is an integer between -1 and 1. 
1 indicates that the robot must go ahead or turn left, -1 indicates that the robot must move backward (or turn right), and 0 indicates that the robot must stop.

When the robot starts moving the move_base node publishes the right velocity and orientation on the cmd_vel topic.

## User Drive Assisted node <img src= "https://media1.giphy.com/media/2Mn5rVOQSGnlRquUkM/200w.webp?cid=ecf05e47wixskor4jhxrjrz9it6ww1p8gd7giv8tq64fke67&rid=200w.webp&ct=s" width="100" height="50">

Essentially, it intends to allow the user to control the robot in the environment using the keyboard, but we also want to enable automatic obstacle avoidance in this instance.

The node, in particular, reads the same exact user inputs as the userDriveNotAssisted node with non-blocking getchar, but it also checks what the robot's laser scanner sees.

This is done by subscribing to the `/scan` topic and using the message it receives to detect walls that are too close to the robot. This topic is made up of 720 _ranges_, each of which contains all of the detected distances to the walls on its right, left and front.

Here below the code to select the right action to do in proximity of a wall:

```cpp
// check if the distance of the robot to a wall is less than the threshold set before and update
// the speed in such a way that the robot cannot collide with circuit delimitations,
// it is able to continue driving avoiding walls.
if (front < th_min)
{
        if (robot_vel.linear.x > 0)
        {
            // Stop the robot, it can only turn now
            lin_dir = 0;
            std::cout << RED << "Be careful, wall on the front!\n"
                      << NC;
        }
}
    
if (right < th_min)
{   
        if (robot_vel.angular.z < 0)
        {
            // Stop the twist, it can only go straight now
            angle_dir = 0;
            std::cout << RED << "Be careful, wall on the right!\n"
                      << NC;
        }
}
    
if (left < th_min)
{
        if (robot_vel.angular.z > 0)
        {
            // Stop the twist, it can only go straight now
            angle_dir = 0;
            std::cout << RED << "Be careful, wall on the left!\n"
                      << NC;
        }
}
```

The code examines the minimum distance between two points within these ranges, and if a wall is closer than `th_min= 1`, the robot is prevented from approaching it. The robot cannot progress if the front wall is too close, and the robot cannot turn in that direction if one of the barriers on the left or right is too close.

The functions merely edit the linear and angular direction according to the requirements above, setting them to '__0__' when required, to activate this security feature.

Finally, the user is prompted with a red risk warning text.

RQT_Graph
---------------------
The project graph, which illustrates the relationships between the nodes, is shown below. Keep in mind that this is a graph built by forcing all three nodes to execute at the same time in order to obtain a full graph. This does not happen during normal execution.

The graph can be generated using this command:
 
```console
$ rqt_graph
``` 

<p align="center">
	<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/rosgraph.png" width="1000">
</p>



Flowchart
------------------

Here below a general flowchart of the whole project:

<p align="center">
<img src="https://github.com/samuelepedrazzi/Assignment3_RT1/blob/noetic/images/Flowchart_Assignment3_RT1.png" width="850" height="750">
</p>

Conclusions and future improvements<img src= "https://media1.giphy.com/media/HGn4DKP2K6HLMTtzf9/200w.webp?cid=ecf05e47d9q1lels5jeofny61n0cbjmyhpl0zas1si8bxxbo&rid=200w.webp&ct=s" width=100 height=60>
-------------------
Even though the objectives for this assignment were quite challeging, I'm pleased with the final product because all three driving modes function properly. In terms of improvements, I've noticed that the feedback to check if the robot has reached the target takes a long time to be detected by the base scan/status topic in the achieveGoalPosition modality, therefore it could be a good idea to manage and optimize the feedback in a different method.
