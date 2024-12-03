# ROS 2 Project: Modular Robot Architecture

## **Overview**
This project enables a robot to dynamically navigate toward a given goal while avoiding obstacles. The architecture integrates sensor data, odometry, SLAM, and trajectory planning to compute real-time paths and commands for the robot.

---

## **System Components**

### **1. Sensor Initialization (Robot)**
- **Purpose**: Initializes all sensors and odometry on the robot.
- **Launch File**:
  - `sensor_interface_launch.py`: This file initializes:
    - **LIDAR**: Collects data for obstacle detection.
    - **Depth Camera**: Captures 3D depth data for environmental mapping.
    - **MIPI Camera**: Captures visual data for debugging or navigation.
    - **Odometry**: Provides real-time position updates for the robot.

---

### **2. Trajectory Planner (Host or Robot)**
- **Purpose**: Processes sensor and odometry data to compute the robot's trajectory.
- **Key Features**:
  - **SLAM**: Generates a real-time map of the environment.
  - **Path Planning**: Calculates the optimal trajectory to the goal.
  - **Robot Control**: Converts the trajectory into velocity commands for the robot.
- **Topics**:
  - `/map`: Real-time map of the environment.
  - `/planned_trajectory`: Planned path toward the goal.
  - `/cmd_vel`: Velocity commands for robot movement.

---

### **3. Simulation and Debugging (Host)**
- **Purpose**: Simulates the robot's actions and facilitates debugging.
- **Key Features**:
  - **RViz Integration**: Visualize sensor data, maps, and trajectories in RViz.
  - **System Logging**: Logs sensor and trajectory data for offline analysis.

---

## **System Workflow**

1. **Sensor Initialization (Robot)**:
   - Run the `sensor_interface_launch.py` file on the robot to start all sensors and odometry.

2. **Trajectory Planning (Host or Robot)**:
   - The planner subscribes to sensor and odometry topics.
   - It performs SLAM, plans paths, and publishes velocity commands to `/cmd_vel`.

3. **Simulation and Debugging (Host)**:
   - RViz displays sensor data, maps, and trajectories.
   - Logs runtime data for debugging and analysis.

---

## **Topics Overview**

| **Topic Name**           | **Message Type**             | **Description**                                       |
|---------------------------|------------------------------|-------------------------------------------------------|
| `/scan`                  | `sensor_msgs/LaserScan`      | LIDAR data for obstacle detection and SLAM.           |
| `/depth_image`           | `sensor_msgs/Image`          | Depth camera data for 3D environmental mapping.       |
| `/camera_image`          | `sensor_msgs/Image`          | Visual data from the MIPI camera for debugging.       |
| `/odom`                  | `nav_msgs/Odometry`          | Real-time position and velocity data of the robot.    |
| `/map`                   | `nav_msgs/OccupancyGrid`     | Real-time occupancy grid map of the environment.      |
| `/goal_pose`             | `geometry_msgs/PoseStamped`  | Target position for the robot to navigate to.         |
| `/trajectory`            | `nav_msgs/Path`              | Sequence of waypoints representing the computed path. |
| `/cmd_vel`               | `geometry_msgs/Twist`        | Velocity commands for robot motion control.           |
| `/tf`                    | `tf2_msgs/TFMessage`         | Transformation tree for robot localization and mapping. |

---

## **Project Directory Structure**
```plaintext
rosmaster/
├── src/
│   ├── sensor_interface/
│   │   ├── launch/
│   │   │   └── sensor_interface_launch.py      # Launches sensors and odometry
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── trajectory_planner/
│   │   ├── launch/
│   │   │   └── trajectory_launch.py            # Runs SLAM, planning, and control
│   │   ├── robot_trajectory_planner/
│   │   │   ├── slam.py                         # SLAM implementation
│   │   │   ├── trajectory.py                   # Trajectory planning class
│   │   │   └── robot_controls.py               # Robot control class
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── utilities/
│   │   ├── launch/
│   │   │   └── simulation_launch.py            # RViz simulation for debugging
│   │   ├── config/
│   │   │   └── config.yaml                     # Parameters for sensors and planning
│   │   ├── utilities/
│   │   │   ├── simulation.py                   # RViz simulation handler
│   │   │   ├── system_logger.py                # Logs data for debugging
│   │   │   └── config_generator.py             # Handles `config.yaml` creation
│   │   ├── setup.py
│   │   └── package.xml
│   │
├── build/
├── install/
└── log/
```
---

## **Usage Instructions**

### **Build the Workspace**
```bash
cd ~/rosmaster
colcon build
source install/setup.bash
```

### **Run Individual Components**
- **Sensor Interface**:
  ```bash
  ros2 launch sensor_interface sensor_launch.py
  ```
- **Trajectory Planner**:
  ```bash
  ros2 launch trajectory_planner trajectory_launch.py
  ```
- **System Manager**:
  ```bash
  ros2 launch utilities navigation_launch.py
  ```
---

## **Authors**
- **Professor**
    - Prof. Gabriel Gomes
- **Researcher**
    - Fernando Pulma
    - Gustav Martin Friede
    - Haoyang Zhou
    - Jasper Hu
    - Jonathan Goenadibrata
    - Wendy Cheng