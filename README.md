# ROS 2 Project: Modular Robot Architecture

## **Overview**
This project's goal is to control a robot to a specific path towards a given goal while updating its trajectory based on obstacles found along the way.

1. **`sensor_interface`**: Handles data acquisition from the robot's sensors (MIPI Camera and LIDAR).
2. **`trajectory_planner`**: Processes sensor data to perform SLAM, plan trajectories, and control the robot's movements.
3. **`utilities`**: Integrates all components, facilitates simulations using RViz, and provides debugging tools.

---

## **Packages and Features**

### **1. sensor_interface**
- **Purpose**: Collects and publishes data from sensors.
- **Key Components**:
  - **MIPICamera**: Captures and processes image data.
  - **LIDAR**: Collects and processes scan data for obstacle detection.
- **Topics**:
  - `/camera/image`: Raw or processed camera data.
  - `/scan`: Processed LIDAR scan data.
- **Launch File**: `robot_launch.py` initializes all sensor nodes.

---

### **2. trajectory_planner**
- **Purpose**: Plans robot trajectories based on sensor data.
- **Key Components**:
  - **SLAM**: Generates maps of the environment.
  - **PathPlanner**: Computes the optimal path to the goal.
  - **ControlModule**: Converts trajectories into velocity commands.
- **Topics**:
  - `/map`: Generated map of the environment.
  - `/trajectory`: Planned path for the robot.
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
│   │   │   ├── magnet.py                       # Magnet Class
│   │   │   └── mipi_camera.py                  # MIPI Camera class
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── trajectory_planner/
│   │   ├── launch/
│   │   │   └── navigation_launch.py            # Launch file for SLAM, trajectory, and control
│   │   ├── trajectory_planner/
│   │   │   ├── __init__.py
│   │   │   └── exploration.py                  # Trajectory Planning & Exploration Algorithm
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── utilities/
│   │   ├── launch/
│   │   │   ├── robot_launch.py            # Main launch file for the system
│   │   │   └── rviz_launch.py             # Launch file for RViz simulation
│   │   ├── utilities/
│   │   │   ├── __init__.py
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

### **Run Instructions**

For the instruction, you will need 4 terminals (terminal 1, 2, & 3 on the robot and terminal 4 on the machine):

1. In **terminal 1** (on the robot):
   ```bash
   ros2 launch utilities robot_launch.py
   ```
2. In **terminal 2** (on the robot):
   ```bash
   ros2 launch yahboomcar_nav map_cartographer_launch.py
   ```
3. Stop **terminal 2** (CTRL+C).
4. In **terminal 4** (on the machine):
   ```bash
   ros2 launch yahboomcar_rviz yahboomcar_nav_launch.py
   ```
5. In **terminal 3** (on the robot):
   ```bash
   ros2 launch yahboomcar_nav navigation_dwb_launch.py
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

