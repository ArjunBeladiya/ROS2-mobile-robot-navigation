# ROS2 Mobile Robot Navigation & Behavior Suite

A progressive ROS2 (Humble) workspace built around simulated mobile robots (TurtleBot3 and TIAGo), covering reactive control, multithreaded execution, custom action-based collision avoidance, and full autonomous navigation with Nav2.

The project was built in four stages, each one adding a more advanced robotics capability on top of the last.

<!-- 
Add a demo GIF or screenshot here — this is the single most effective way to make a
robotics repo stand out. Record ~10-15 seconds of the robot in RViz/Gazebo and drop
it in a `media/` folder, then reference it like this:
![Demo](media/demo.gif)
-->
## Demo
![Follow_wall_Demo](follow_wall.mp4)
![tf_follower](tf_follower.mp4)

## Overview

| Stage | Package(s) | What it does |
|---|---|---|
| 1 | `drive_to_wall`, `follow_wall` | Reactive control using `LaserScan` data: stop before hitting a wall, then follow it using a state machine |
| 2 | `rainbow_circle` | Drives the robot in a circular path while cycling colors; compares single-threaded vs. multi-threaded ROS2 executors |
| 3 | `collision_avoidance`, `custom_interfaces` | Danger-zone based collision avoidance using a custom ROS2 action (`TravelNoCrashing`) and custom message (`DangerZones`) |
| 4 | `my_nav2`, `initial_pose_publisher`, `tf_follower` | Full Nav2 integration: initial pose publishing, TF-based robot following, and sending navigation goals via an action client |

## Tech Stack

- **ROS2 Humble**
- **Gazebo** (simulation)
- **Nav2** (autonomous navigation stack)
- **TurtleBot3** and **TIAGo** robot models
- **Docker / Docker Compose** for containerized simulation environments
- Python (`rclpy`)

## Repository Structure

```
.
├── src/
│   ├── stage_1/
│   │   ├── drive_to_wall/       # Stops the robot before it collides with a wall
│   │   └── follow_wall/         # State-machine wall-following behavior
│   ├── stage_2/
│   │   └── rainbow_circle/      # Circular motion + executor comparison (single vs multi-threaded)
│   ├── stage_3/
│   │   ├── collision_avoidance/ # Zone-based (WHITE/GREEN/YELLOW/RED) collision avoidance via custom action
│   │   └── custom_interfaces/   # Custom .msg / .action definitions
│   └── stage_4/
│       ├── my_nav2/                  # Nav2 goal publishing and action client
│       ├── initial_pose_publisher/   # Publishes initial AMCL pose for different simulation worlds
│       └── tf_follower/              # Follows a robot using TF transforms
├── config/       # Robot configuration (TIAGo stock/custom)
├── maps/         # Pre-built occupancy grid maps for navigation
├── docker/       # Dockerfiles and Compose files for simulation environments
├── scripts/      # Helper shell scripts
└── my_bashrc     # Container shell environment setup
```

## Stage Details

### Stage 1 — Reactive Wall Following
- **`drive_to_wall`**: subscribes to `/scan`, drives forward and stops the robot once an obstacle is detected within a threshold distance.
- **`follow_wall`**: extends this into a 4-state behavior (`MOVE_FORWARD`, `TURN_LEFT`, `WALL_FOLLOW`, `STOPPED`) that lets the robot trace along a wall using laser scan geometry.

### Stage 2 — Executors & Motion Patterns
- **`rainbow_circle`**: drives the robot in a circular trajectory while publishing color changes, implemented twice — once with a single-threaded executor and once with a multi-threaded executor — to compare responsiveness and callback handling under ROS2's concurrency models.

### Stage 3 — Action-Based Collision Avoidance
- **`custom_interfaces`**: defines a custom `DangerZones` message and a `TravelNoCrashing` action.
- **`collision_avoidance`**: an action server (`collision_avoider_node`) that classifies surrounding space into four danger zones (WHITE → RED) using `collision_detector_node`, and safely halts or reroutes robot motion based on the current zone, using `ReentrantCallbackGroup` and a `MultiThreadedExecutor` for concurrent goal handling.

### Stage 4 — Autonomous Navigation with Nav2
- **`initial_pose_publisher`**: publishes the robot's starting pose to AMCL for different simulation worlds (`initial_pose_world`, `initial_pose_dqn_stage4`).
- **`tf_follower`**: tracks and follows a target frame using ROS2 TF transforms.
- **`my_nav2`**: sends navigation goals to the Nav2 stack via `goal_pub` and `goal_action_client`, enabling autonomous point-to-point navigation.

## Getting Started

### Prerequisites
- ROS2 Humble
- Gazebo
- `colcon` build tools
- TurtleBot3 and/or TIAGo simulation packages
- Docker & Docker Compose (optional, for containerized runs)

> Note: the Docker images referenced in `docker/docker-compose-*.yml` point to a private university registry and won't be pullable outside that environment — use them as a reference for the intended container setup, or swap in your own ROS2 Humble + Gazebo image.

### Build

```bash
# from the workspace root
colcon build
source install/setup.bash
```

### Run examples

```bash
# Stage 1 — stop before a wall
ros2 run drive_to_wall drive_to_wall_node

# Stage 1 — follow a wall
ros2 run follow_wall follow_wall_node

# Stage 2 — circular motion with a multithreaded executor
ros2 run rainbow_circle multithreading_node

# Stage 3 — collision avoidance action server
ros2 run collision_avoidance collision_avoider_node

# Stage 4 — send a navigation goal via Nav2
ros2 run my_nav2 goal_pub
```

Each package can also be launched inside the provided Gazebo simulation worlds — see `docker/` and `maps/` for the corresponding world and map configurations.

## Maps

Pre-generated occupancy grid maps used for Nav2 localization and path planning are included in `maps/`, including a custom-built map (`myfirstmap`) and a TurtleBot3 DQN stage map (`dqn_stage4`).

## Notes

This project was developed as part of a university robotics course, progressing from basic reactive control to full autonomous navigation. Each stage builds directly on ROS2 concepts introduced in the previous one: topics and sensor processing → concurrency and executors → actions and custom interfaces → the Nav2 navigation stack.
