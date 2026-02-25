# Travel Ethiopia
 This project implements various AI search algorithms for pathfinding across Ethiopia's city network and simulates robot navigation using **Gazebo** with ROS 2.

### Search Algorithms
- **BFS / DFS** — Uninformed search for finding paths
- **UCS** — Uniform Cost Search for optimal paths
- **A\*** — Heuristic search for faster pathfinding
- **MiniMax** — Adversarial search for coffee quality optimization

### Run Search Algorithms
```bash
python3 -m main


## ROS 2 AI Search & Robot Navigation Simulation

Travel Ethiopia is a ROS 2 Python package that integrates classical AI
search algorithms with autonomous robot navigation inside Gazebo.

The robot navigates an abstracted Ethiopia city graph while performing:

-   BFS / DFS path planning\
-   Global route generation\
-   Path following\
-   Obstacle avoidance\
-   Sensor fusion\
-   Vision-based correction

------------------------------------------------------------------------

#   Package Structure

   
```
travel_ethiopia/
├── data/                        # Graph data and city coordinates
│   ├── graph1.py               # BFS/DFS graph
│   ├── graph2.py               # UCS/A* weighted graph
│   ├── graph3.py               # Heuristics
│   ├── graph4.py               # iniMax)
│   └── graph5.py               # Robot navigation graph
├── algorithms/                 # Search algorithm implementations
│   ├── bfs_dfs.py             # BFS and DFS
│   ├── ucs.py                 # Uniform Cost Search
│   ├── astart.py              # A* Search
│   ├── advs.py                # MiniMax
│   └── robot.py               # Robot navigation logic
├── src
    └── travel_robot
        ├── launch
        │   └── world.launch.py
        ├── package.xml
        ├── resource
        │   └── travel_robot
        ├── setup.cfg
        ├── setup.py
        ├── travel_robot
        │   ├── __init__.py
        │   ├── __pycache__
        │   │   ├── __init__.cpython-312.pyc
        │   │   ├── path_follower.cpython-312.pyc
        │   │   ├── robot_planner.cpython-312.pyc
        │   │   └── vision_navigation.cpython-312.pyc
        │   ├── data
        │   │   └── graph5.py
        │   ├── obstacle_avoidance.py
        │   ├── path_follower.py
        │   ├── robot_planner.py
        │   ├── search
        │   │   ├── __init__.py
        │   │   └── bfs_dfs.py
        │   ├── sensor_fusion.py
        │   └── vision_navigation.py
        ├── urdf
        │   └── three_wheel_robot.urdf
        ├── visualizer
        │   └── index.html
        └── worlds
            └── ethiopia_relaxed.world
  
```

------------------------------------------------------------------------

#  Search Algorithms

Located in:

    travel_robot/search/bfs_dfs.py

Implemented:

-   Breadth-First Search (BFS)
-   Depth-First Search (DFS)

Used by:

    robot_planner.py

------------------------------------------------------------------------

#   Executable ROS 2 Nodes

Defined in setup.py:

-   robot_planner
-   vision_navigation
-   sensor_fusion
-   obstacle_avoidance
-   path_follower

Run example:

    ros2 run travel_robot robot_planner

------------------------------------------------------------------------

#   Simulation

Gazebo world:

    worlds/ethiopia_relaxed.world

Robot model:

    urdf/three_wheel_robot.urdf

Launch full simulation:

    ros2 launch travel_robot world.launch.py

------------------------------------------------------------------------

#   Installation

## Requirements

-   Ubuntu 22.04
-   ROS 2 Jazzy
-   Gazebo
-   Python 3.10+

------------------------------------------------------------------------

## Install Gazebo ROS Integration

    sudo apt update
    sudo apt install ros-jazzy-gazebo-ros-pkgs
    source /opt/ros/jazzy/setup.bash

------------------------------------------------------------------------

## Build Workspace

From workspace root:

    colcon build --symlink-install
    source install/setup.bash

------------------------------------------------------------------------

#   Testing

    colcon test
    colcon test-result --verbose

------------------------------------------------------------------------

#   Architecture Overview

1.  robot_planner computes global path using BFS/DFS\
2.  path_follower executes waypoints\
3.  obstacle_avoidance handles local hazards\
4.  sensor_fusion refines state estimate\
5.  vision_navigation improves alignment

------------------------------------------------------------------------

#  Educational Purpose

This project demonstrates:

-   AI search algorithms in robotics\
-   ROS 2 Python package design\
-   Modular autonomous navigation\
-   Gazebo-based simulation workflows

------------------------------------------------------------------------

