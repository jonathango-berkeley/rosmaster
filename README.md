# ROS 2 Project: Modular Robot Architecture

## **Overview**
This project's goal is to control a robot to a specific path towards a given goal while updating its trajectory based on obstacles found along the way.

1. **`sensor_interface`**: Handles data acquisition from the robot's sensors (MIPI Camera, Depth Camera, and LIDAR).
2. **`trajectory_planner`**: Processes sensor data to perform SLAM, plan trajectories, and control the robot's movements.
3. **`utilities`**: Integrates all components, facilitates simulations using RViz, and provides debugging tools.

---

## **Packages and Features**

### **1. sensor_interface**
- **Purpose**: Collects and publishes data from sensors.
- **Key Components**:
  - **MIPICamera**: Captures and processes image data.
  - **DepthCamera**: Captures depth frames and processes 3D data.
  - **LIDAR**: Collects and processes scan data for obstacle detection.
- **Topics**:
  - `/mipi_camera_data`: Raw or processed camera data.
  - `/depth_camera_data`: Depth camera data.
  - `/lidar_data`: Processed LIDAR scan data.
- **Launch File**: `sensor_launch.py` initializes all sensor nodes.

---

### **2. trajectory_planner**
- **Purpose**: Plans robot trajectories based on sensor data.
- **Key Components**:
  - **SLAM**: Generates maps of the environment.
  - **PathPlanner**: Computes the optimal path to the goal.
  - **ControlModule**: Converts trajectories into velocity commands.
- **Topics**:
  - `/map`: Generated map of the environment.
  - `/planned_trajectory`: Planned path for the robot.
  - `/cmd_vel`: Robot velocity commands.
- **Launch File**: `trajectory_launch.py` runs and debugs SLAM, planning, and control subsystems.

---

### **3. utilities**
- **Purpose**: Manages the entire system and supports simulation and logging.
- **Key Components**:
  - **RVizSimulation**: Simulates robot actions in RViz.
  - **SystemLogger**: Logs data from all components for debugging.
  - **ConfigGenerator**: Loads parameters and goals from `config.yaml`.
- **Features**:
  - **Simulation**: Visualize the robot and its actions in RViz.
  - **Logging**: Collect runtime data for analysis.
  - **Main Launch File**: `navigation_launch.py` initializes the full system.
- **Config File**: `config/config.yaml` contains parameters such as sensor settings, start and end goals, and simulation options.

---

## **System Workflow**

1. **Sensor Data Acquisition**:
   - `sensor_interface` collects data from all sensors and publishes it to ROS topics.

2. **Trajectory Planning**:
   - `trajectory_planner` subscribes to sensor topics, performs SLAM, plans trajectories, and generates robot commands.

3. **System Management**:
   - `utilities` integrates components, runs RViz simulations, and logs data for debugging.

---

## **Project Directory Structure**
```plaintext
rosmaster/
├── src/
│   ├── sensor_interface/
│   │   ├── launch/
│   │   │   └── sensor_launch.py                # Launch file for all sensors
│   │   ├── sensor_interface/
│   │   │   ├── __init__.py
│   │   │   ├── mipi_camera.py                  # MIPI Camera class
│   │   │   ├── depth_camera.py                 # Depth Camera class
│   │   │   └── lidar.py                        # LIDAR class
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── trajectory_planner/
│   │   ├── launch/
│   │   │   └── trajectory_launch.py            # Launch file for SLAM, trajectory, and control
│   │   ├── robot_trajectory_planner/
│   │   │   ├── __init__.py
│   │   │   ├── slam.py                         # SLAM implementation
│   │   │   ├── trajectory.py                   # Trajectory planning class
│   │   │   └── robot_controls.py               # Robot control class
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── utilities/
│   │   ├── launch/
│   │   │   ├── navigation_launch.py            # Main launch file for the system
│   │   │   └── simulation_launch.py            # Launch file for RViz simulation
│   │   ├── utilities/
│   │   │   ├── __init__.py
│   │   │   ├── system_logger.py                # Logs data from all components
│   │   │   ├── simulation.py                   # RViz simulation handler
│   │   │   └── config_generator.py             # Handles `config.yaml` loading
│   │   ├── config/
│   │   │   └── config.yaml                     # Configuration for parameters, goals, etc.
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