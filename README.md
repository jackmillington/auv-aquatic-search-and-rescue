# 3806ICT Assignment 3 (Multi-Agent) by:

- Jack Millington
- Jayden Knight
- Callum Brown
- Yusuf Abushaaban

This project requires an Ubuntu environment with **ROS**, **Gazebo**, and **PAT** installed. It models a **multi-agent** aquatic Search and Rescue (SAR) scenario in a 2D grid world. The environment is resolved into a grid, where each position can contain a hostile entity, a survivor, or a marker to indicate whether it has been visited.

Gazebo simulates the environment and provides a 3D representation in real-time. ROS drives the control loops for two specialised agents: a **Search AUV** that explores and maps the world, and a **Rescue AUV** that collects survivors. A **Manager** node coordinates the mission phases (search then rescue). High-level plans are generated in PAT (via `.csp` models), returned as a list of moves, and executed by ROS using emulated directional sensors.

To successfully run this project on your own machine:

1. Clone this repository into your catkin workspace (under `catkin_ws/src`).
2. Compile in your catkin workspace using `catkin_make`.
3. Launch the pre-set world (includes birds-eye camera angle):
   `roslaunch assignment_3 launch_world.launch`
4. Run the grid/services node in a new terminal:
   `rosrun assignment_3 update_grid`
5. Run the manager (starts and coordinates search + rescue) in a new terminal:
   `rosrun assignment_3 search_rescue_manager`

Notes:
- Core nodes: `search_rescue_search.cpp` (search AUV), `search_rescue_rescue.cpp` (rescue AUV), `search_rescue_manager.cpp` (coordination), `update_grid.cpp` (Gazebo/grid services).
- The project depends on the direct path to the PAT installation (configured in code via a `PAT_EXE_DIR` / equivalent define). If PAT is not found, relocate PAT to the expected path or update the constant in the relevant source file(s).
