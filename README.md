
````markdown
 Custom DWA Local Planner – ROS 2 Humble (TurtleBot3)

Author: Koyaneni Yaswanth
**Assignment: Robotics Deployment Intern – 10xConstruction
**Date: November 2025

---

Overview

This repository contains a Custom Dynamic Window Approach (DWA) Local Planner implemented from scratch in Python for the **TurtleBot3** robot using **ROS 2 Humble** and **Gazebo Classic**.

The planner computes safe and efficient velocity commands (`/cmd_vel`) to drive the robot toward a goal pose while actively avoiding obstacles. It achieves this completely independently, **without using Nav2’s built-in DWB controller.**

Features
* Dynamic Sampling: Samples `(v, w)` velocity pairs within the robot's acceleration and velocity limits.
* Trajectory Prediction:** Simulates future trajectories for each velocity sample.
* Cost Evaluation: Scores trajectories based on three weighted criteria:
    * Heading Cost: Deviation from the goal.
    * Obstacle Cost: Proximity to obstacles (Safety).
    * Velocity Cost: Magnitude of velocity (Efficiency).
* Command Execution: Selects the optimal trajectory and publishes drive commands to `/cmd_vel`.
* Visualization: Publishes candidate trajectories to `/dwa_trajectories` for debugging in RViz.

---

Software Stack

| Component | Version / Package |
| :--- | :--- |
| OS | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| ROS 2| Humble Hawksbill |
| Simulator | Gazebo Classic 11 |
| Robot | TurtleBot3 Burger |
| Visualization | RViz 2 |

---

Installation & Build Steps

Install ROS 2 Humble + Gazebo
Ensure your system is up to date and install the required ROS 2 packages.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop \
  ros-humble-turtlebot3* \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup

Set the TurtleBot3 Model in your environment
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/humble/setup.bash
````

Create Workspace & Clone

```bash
mkdir -p ~/10x_ws/src
cd ~/10x_ws/src
git clone [https://github.com/koyaneni-yaswanth/custom_dwa_planner.git](https://github.com/koyaneni-yaswanth/custom_dwa_planner.git)
```

 Build the Package

```bash
cd ~/10x_ws
colcon build
source install/setup.bash
```

-----

Run Instructions

1. Launch Gazebo Simulation

In a new terminal, launch the simulation world:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Start the Custom Planner

In a second terminal, run the planner node:

```bash
source ~/10x_ws/install/setup.bash
ros2 run custom_dwa_planner dwa_planner_node --ros-args --log-level info
```

Expected Output:

```text
[INFO] [custom_dwa_planner]: Custom DWA Planner node started.
```

3. Send a Goal

In a third terminal, publish a goal pose to the `/goal_pose` topic. The planner will detect this and begin navigation.

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1
```

Planner Log Output:

```text
[INFO] [custom_dwa_planner]: New goal received: x=1.00, y=0.00
```

*The robot should now begin rolling toward the coordinate (1.0, 0.0).*

-----

 Algorithm Details

 1\. Inputs

The node subscribes to the following topics to make decisions:

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/Odometry` | Current robot pose $(x, y, \theta)$ & velocity. |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data for obstacle detection. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Desired target destination. |
| `/cmd_vel` | `geometry_msgs/Twist` | **Output:** Computed velocity command. |

2\. Dynamic Window Sampling

The planner generates a search space of feasible linear ($v$) and angular ($w$) velocities considering the robot's acceleration limits:

```math
v \in [v_{min}, v_{max}], \quad w \in [-\omega_{max}, \omega_{max}]
```

3\. Forward Simulation

For every $(v, w)$ pair, the planner predicts the robot's pose over a fixed horizon (`sim_time`) using discrete steps (`dt`):

```cpp
x_{t+1} = x_t + v * cos(theta_t) * dt
y_{t+1} = y_t + v * sin(theta_t) * dt
theta_{t+1} = theta_t + w * dt
```
4\. Collision Check

Each simulated point is compared against the LaserScan data. If any trajectory point lies within `robot_radius * safety_factor` of an obstacle, the trajectory is marked invalid (infinite cost).

5\. Cost Function

Valid trajectories are scored to find the best path:

```makefile
Total Cost = (heading_weight * heading_cost) +
             (velocity_weight * velocity_cost) +
             (obstacle_weight * obstacle_cost)
```

  * **Heading Cost:** Minimizes the angular difference between the robot and the goal.
  * **Velocity Cost:** Rewards higher velocities to prevent stagnation.
  * **Obstacle Cost:** Penalizes getting close to obstacles (even if no collision occurs).

**The trajectory with the lowest cost is selected.**

6\. Visualization

The planner publishes all considered paths as a `MarkerArray` (using `LINE_STRIP`) to the topic `/dwa_trajectories`.

-----

Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `max_vel_x` | 0.5 m/s | Maximum linear speed. |
| `max_vel_theta` | 4.0 rad/s | Maximum angular speed. |
| `sim_time` | 2.0 s | Prediction horizon (how far to look ahead). |
| `dt` | 0.1 s | Time step for simulation integration. |
| `robot_radius` | 0.10 m | Radius used for collision checking. |
| `heading_weight` | 1.0 | Weight for alignment with the goal. |
| `velocity_weight` | 0.5 | Weight for encouraging forward motion. |
| `obstacle_weight` | 0.8 | Weight for avoiding obstacles. |

**Runtime Override Example:**

```bash
ros2 run custom_dwa_planner dwa_planner_node --ros-args \
  -p max_vel_x:=0.6 -p obstacle_weight:=0.5
```

-----

RViz Visualization

Launch RViz and add the following Displays to visualize the planner's logic:

| Display Type | Topic | Description |
| :--- | :--- | :--- |
| Odometry | `/odom` | Shows the robot's current path/trail. |
| LaserScan | `/scan` | Shows the obstacle cloud. |
| Pose | `/goal_pose` | Visualizes the goal arrow. |
| MarkerArray | `/dwa_trajectories` | Visualizes all candidate paths (Green Lines). |

-----

Testing Results

  * Performance: The TurtleBot3 Burger successfully reaches the goal while autonomously avoiding static obstacles.
  * Visuals: Trajectories are clearly visible as green lines in RViz, updating in real-time.
  * Speed: The planner runs at approximately 10 Hz, maintaining a steady velocity of \~0.25 m/s in open spaces.
  * Safety: No collisions occurred during testing in moderate obstacle environments.

<img width="1204" height="856" alt="rviz" src="https://github.com/user-attachments/assets/9162756d-b482-4874-a11f-8143ddc79fa9" />

-----

compass: Troubleshooting

| Issue | Potential Fix |
| :--- | :--- |
| Robot not moving | Check `/cmd_vel`. Is `linear.x` non-zero? Ensure goal is set. |
| No LaserScan data | Verify the topic name. You may need to remap to `/turtlebot3/scan`. |
| Stops unexpectedly | Reduce `obstacle_weight` or slightly increase `robot_radius` (if getting stuck in cost minima). |
| Gazebo frozen | Check if the simulation is paused. Try resetting the world. |

-----

Future Improvements

  * Integrate with Nav2 as a formal controller plugin.
  * Implement Dynamic Parameter Reconfigure for runtime tuning.
  * Add adaptive sampling for dense obstacle environments.
  * Extend support for Ackermann or non-holonomic constraints.

-----

License

Released under the MIT License. You are free to use, modify, and distribute this software with attribution.

Acknowledgements

  * [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
  * [ROBOTIS TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
  * Fox et al., 1997 – *The Dynamic Window Approach to Collision Avoidance*.

<!-- end list -->

```
```
