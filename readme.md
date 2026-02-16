# 3806ICT Assignment 3 by:

-  Callam Hartley | s5113156
-  Daniel Jacobsen | s5262721
-  Todd Cooper | s2681289
-  James Hudson | s5182091

This project requires an Ubuntu environment with ROS, Gazebo, and PAT installed. It models a submarine as it explores its environment in a linear 2D fashion. The environment is resolved into a grid, where each position can contain a hostile entity, a survivor, or a marker to indicate whether it has been visited.

Gazebo simulates the environment and provides a 3D representation in real-time. ROS drives the submarine's main control loop, which extracts a list of moves to achieve pre-defined goals from PAT. For example, PAT is called via the command line to achieve a goal, such as "return home", and returns a list of moves to a text file. ROS reads this text file and iteratively translates this moveset into a positioning system, using emulated sensors to react to its immediate surroundings.

To successfully run this project on your own machine:

1. Clone this repository into the catkin workspace folder (under catkin_ws/src).
2. Compile the binaries in catkin_ws/src using catkin_make.
3. Launch the pre-set world which includes a birds-eye camera angle:
   `roslaunch assignment_3 launch_world.launch`
4. Run the update_grid node: `rosrun assignment_3 update_grid` in a new terminal window. This hosts three services used by the main submarine controller:

   -  hostile_sensor: Emulates a sensor (such as a sonar) to detect hostiles within a given grid-range
   -  survivor_sensor: Emulates a sensor (such as infrared) to detect survivors within a given grid-range
   -  update_grid: This facilitates the communication between the main submarine controller and update_grid. It allows for gazebo to continue simulating the submarine as it moves throughout the environment and picks up survivors.

5. Run the search_rescue node: `rosrun assignment_3 search_rescue` in a new terminal window. This is the main driver of the submarine which communicates with PAT and translates the provided moveset into actions. It maintains an internal representation of where the submarine has visited which is updated each time it detects a hostile or survivor (as informed by the hostile_sensor and survivor_sensor services).

**Please note that the project depends on the direct path to the PAT installation. Currently, it is set to `/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe`. If this path is incorrect, either relocate PAT to the expected path, or change `PAT_EXE_DIR` in the program to the correct path. This `#define` is located in `search_rescue.cpp` on line 29**
