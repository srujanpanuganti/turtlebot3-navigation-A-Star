# differential_drive_a_star
A* implementation for a differential drive robot - simulated on Turtlebot 2 using ROS Gazebo
This code required Python 3

following packages have to be imported for running this file 
numpy
math
queue
argparse
matplotlib.pyplot


The code is divided into four parts

main.py
obstacle_space_offset.py
action_set_offset.py
publisher.py

the main.py imports the action_set_offset.py and obstacle_space_offset.py while it is being executed. The input for executing this needs to be given as following

python main.py [start_position] [goal_position]

The sample imput for running main.py are given as below 
python main.py 1,2 10,10
python main.py 0,0 15,30  --> used for generating the video (this is generated for low rpm's, rpm1 = 2 and rpm2 = 3)
it's good to take rpm1 = 5 and rpm2 = 4 for relatively efficient performance of the algorithm 

after this program is executed, it will generate the converted_velocities.txt file which will be supplied to the ROS Gazebo.

now open a terminal to initialize the roscore

in the package that I we have created should also consist the rrl_map and the launch file which is named as sru.launch
open a new terminal and now, launch the ROS gazebo using the following launch commands to load the rrl_map as follwoing 


ROBOT_INITIAL_POSE="-x [x coordinated of start position] -y [y coordinated of the start position]" roslaunch planning sru.launch
the above command will spawn the turtlebot at the start position. The sample command is given as below 

note: the gazebo takes the spawn coordinated in meters, hence we need to give start_position divided by 10 which is my resolution
ROBOT_INITIAL_POSE="-x 0.1 -y 0.2" roslaunch planning sru.launch


now open a new terminal to run the publisher.py file as following 
rosrun [package-name] publisher.py

The sample command is given as below. In my case the name of the package that I have created is planning. 
rosrun planning publisher.py


Additional attachments:
the path generated from the A* algorithm has been attached path.png in the codes folder
the simulation videos have been attached with the name turtlebot1.mp4 and turtlebot2.mp4
turtlebot1.mp4 is a longer video whereas turtlebot2.mp4 is a shorter video for a smaller distance 

As the turtlebot is moving relatively slowly, it took more than 30 seconds to reach the goal.
