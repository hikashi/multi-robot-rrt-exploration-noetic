# Multirobot Turtlebot3 Simulation Exploration based on RRT for Ubuntu 20.04 ROS Noetic
This package is a compilation of the RRT package in a much complete package rather than figuring map merging and other functions from other resipotary. 
This would serve as a self-contained package for the exploration module using simulation for the turtlebot.
This is tested on Ubuntu 20.04.03LTS with ROS Noetic.

**A portion of the code with Python 2 has been updated to Python 3.**
**Note that the outcome might differ from the original calculation but should be approximately identical.**

credit to Hasauino for creating the RRT exploration packages.

[RRT Exploration package](https://github.com/hasauino/rrt_exploration "RRT Exploration").

[RRT Exploration Tutorial package](https://github.com/hasauino/rrt_exploration_tutorials "RRT Exploration").


# update 06 Nov 2023
- remove camera for better performance.
- update the fps for better performance in the lower specification computer.




## Requirements
The following code is executed in ROS Noetic in Ubuntu 20.04 LTS

The following libraries are required to be installed before proceeding to run the code

    $ sudo apt-get install ros-noetic-turtlebot3*
    $ sudo apt-get install ros-noetic-gmapping
    $ sudo apt-get install ros-noetic-navigation
    $ sudo apt-get install python-opencv
    $ sudo apt-get install python-numpy
    $ sudo apt-get install python-scikits-learn
    $ sudo apt-get install ros-noetic-teb-local-planner
    $ sudo apt-get install ros-noetic-multirobot-map-merge


## Installation Process
Create a new folder called "catkin_explore/src" by executing the following comment:

    $ mkdir -p ~/catkin_explore/src
    $ cd ~/catkin_explore/src/
    $ git clone https://github.com/hikashi/multi-robot-rrt-exploration-noetic.git
    $ cd ~/catkin_explore
    $ catkin_make

## Add in Amazon Map
Download Amazon world map by executing the following comments:

    $ cd ~/catkin_explore/src
    $ git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
    $ git clone https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
    $ cd ~/catkin_explore/
    $ catkin_make
    

## Execution for Single Robot
The program can be executed using the following comments in three different terminals:

Terminal 1

     # roscore 
Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # export TURTLEBOT3_MODEL=waffle_pi
     # roslaunch ros_multi_tb3 single_tb3_house.launch
Terminal 3

     # source ~/catkin_explore/devel/setup.bash 
     # export TURTLEBOT3_MODEL=waffle_pi
     # roslaunch rrt_exploration single_robot.launch 

## Execution for Multirobot
The program can be executed using the following comments in three different terminals:

Terminal 1

     # roscore 
Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # export TURTLEBOT3_MODEL=waffle_pi
     # roslaunch ros_multi_tb3 multiple_tb3_house.launch 
Terminal 3

     # source ~/catkin_explore/devel/setup.bash 
     # export TURTLEBOT3_MODEL=waffle_pi
     # roslaunch rrt_exploration three_robots.launch 



## Exploration Process
The exploration relies on the correct sequence of starting clicked points or else there will be an incorrect sequence of the exploration boundary.
The idea is to start the exploration within a given boundary and not traverse outside of the boundary.
1. Top Left
2. Bottom Left
3. Bottom Right
4. Top Right
5. Initial Point
 ![Instruction](/instruction2.png)
 Things to note: 5th initial point should be within the known map or preferably close to the robot starting point. 
 
 
## Saving Map
Save the map for a single robot using the following command:

    # rosrun map_server map_saver -f mymap map:=/tb3_0/map
    
    
Save the map for multi-robot using the following command:

    # rosrun map_server map_saver -f mymap map:=/map
    
    
## How to speed up Gazebo
- Try to download the [online models](https://github.com/osrf/gazebo_models) and put inside "~/.gazebo/models/" folder 
- Try not to run a lot of processes in the background (simulation is CPU intensive)
- RVIZ might take up some of the computing power, can try to drop some topics if needed.

## known issues
1. The map merging will shift a lot of the slam drifting is too severe.
2. Shifting causes the frontier point to remain even after being explored.
3. Sometimes robots will take some time to move to a new plan.
4. The revenue function is awkwardly computing large negative values and hence the navigation cost seems insignificant. (inherited from the original RRT exploration)
5.  The assignment of the goal is not close to suboptimal. May need to perform optimization on the goal allocation.
6.  Some may encounter deadlock for robots in some maps due to the suboptimal configuration of the robot.
7.  Some calculations might be different from the original as there are some changes in the algorithm in the midst of changing the script from Python 2 to Python 3.
