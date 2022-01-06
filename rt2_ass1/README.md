# First Assignment of the Research Track 2 course

Shahrzad Eskandari Majdar (5060737)


## Architecture of the system:
The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.

The robot starts at the orign (0,0) without any orientation or velocity. After being requested by the user or pressing 1, it starts moving towards the first random generated position. The user can command the robot to stop at any time by pressing 0. However, it will stop after reaching the target. When the next 'start' command is called, the robot will move to the next random generated position.

### Node: 
- PositionServer
- GoToPoint
- StateMachine
- UserInterface

With two nodes implemented in Python (.py) and the other two implemented in C++ (.cpp).

### Service:
- random position service
- start or stop the robot 
- drive the robot toward a point in the environment

### The launch file will open:
- The simulation environment
- The node PositionServer, which implements a random position service with random values for x, y, and theta, where x and y should be limited between some min and max values
- The node GoToPoint, which implements a service to drive a robot toward a point in space (x,y) with angle (theta) in the environment
- The node FSM, which implements a service to start or stop the robot, and calls the other two services to drive the robot. The robot can be stopped only when it reaches a target
- The UserInterface, which asks the user to start/stop the robot, and calls the service implemented in the FSM node

This package should contain two additional branches: **action** and **ros2**, which will be mentioned later. Other than that, a VREP scene which interacts with the simulation is also required.


## How to run the node:

To run the code, open 2 terminals in advance. One is for running the code below and launch Gazebo simulation environment:

```
roslaunch rt2_ass1 sim.launch
```

In another terminal, bedore running the code, make sure either the ros or ros2 workspace is active by uncommenting the correct source files in the bash file. In order to open the Coppelia Sim Software, go to folder 'CoppeliaSim_Edu_V4_2_0_Ubuntu20_04' and run:

```
./coppeliaSim.sh
```
After that, open the correct scene and modify the file until the (Vrep).ttt file is created and saved.



