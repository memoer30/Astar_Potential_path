# TurtleBot3 Custom Navigation System

A custom navigation system for TurtleBot3 implemented in ROS Noetic without using the standard ROS Navigation Stack. This package provides autonomous navigation capabilities using custom-built path planning, obstacle avoidance, and control algorithms.

## 🚀 Features

- **Custom A* Global Path Planner**: Efficient pathfinding with obstacle inflation for safe navigation
- **Kinematic Controller**: Sequential movement controller with rotation-then-forward motion
- **Potential Fields Local Planner**: Dynamic obstacle avoidance using artificial potential fields
- **Integrated Navigation System**: Seamless coordination between global planning and local control
- **Real-time Obstacle Avoidance**: Reactive navigation around static and dynamic obstacles
- **Recovery Behaviors**: Automatic recovery from deadlock situations

## 📋 Prerequisites

- **ROS Noetic** (Ubuntu 20.04)
- **Gazebo** simulation environment
- **TurtleBot3 packages**:
  ```bash
  sudo apt install ros-noetic-turtlebot3*
  ```
- **Required ROS packages**:
  ```bash
  sudo apt install ros-noetic-map-server ros-noetic-amcl ros-noetic-gmapping
  ```

## 🛠️ Installation

1. **Clone the repository** into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone git@github.com:memoer30/Astar_Potential_path.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

4. **Set TurtleBot3 model** (add to ~/.bashrc):
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

## 🎮 Usage

### Basic Navigation

1. **Launch the navigation system**:
   ```bash
   roslaunch assignment_turtlebot3 navigation.launch
   ```

2. **Set a navigation goal** in RViz:
   - Use the "2D Nav Goal" tool
   - Click and drag to set the target position and orientation

### Creating Maps

1. **Launch mapping mode**:
   ```bash
   roslaunch assignment_turtlebot3 mapping.launch
   ```

2. **Manually control the robot** to explore the environment:
   ```bash
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

3. **Save the map**:
   ```bash
   rosrun map_server map_saver -f ~/catkin_ws/src/assignment_turtlebot3/maps/my_map
   ```

## 🏗️ Architecture

### Core Components

#### 1. Global Planner (`global_planner.py`)
- **Algorithm**: A* pathfinding
- **Features**: 
  - Euclidean distance heuristic
  - 4-connectivity movement (no diagonal)
  - Obstacle inflation for safety margins

#### 2. Kinematic Controller (`kinematic_controller.py`)
- **Strategy**: Sequential movement (rotate → move forward)
- **Features**:
  - Precise waypoint tracking
  - Configurable velocity limits
  - Angular and linear motion separation

#### 3. Potential Fields (`potential_fields.py`)
- **Algorithm**: Artificial Potential Fields
- **Features**:
  - Multi-zone repulsive forces
  - Attractive force toward goals
  - Real-time obstacle avoidance
  - Force balancing and limiting

#### 4. Navigation Controller (`navigator.py`)
- **Role**: System integration and coordination
- **Features**:
  - Path planning coordination
  - Mode switching (path following ↔ obstacle avoidance)
  - Recovery behaviors
  - ROS topic integration

### System Flow

```
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
│ Global Goal │ -> │ A* Planner   │ -> │ Waypoint Queue  │
└─────────────┘    └──────────────┘    └─────────────────┘
                                                │
                                                ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
│ Laser Scan  │ -> │ Potential    │ -> │ Velocity        │
│             │    │ Fields       │    │ Commands        │
└─────────────┘    └──────────────┘    └─────────────────┘
                                                │
                                                ▼
                           ┌─────────────────────────────────┐
                           │ Navigator (Mode Switching)      │
                           │ • Path Following                │
                           │ • Obstacle Avoidance           │
                           │ • Recovery Behaviors           │
                           └─────────────────────────────────┘
```

## ⚙️ Configuration

### Navigation Parameters

Key parameters can be tuned in the navigation scripts:

#### Obstacle Avoidance Thresholds
```python
obstacle_enter_threshold = 0.35  # Distance to enter avoidance mode
obstacle_exit_threshold = 0.5    # Distance to exit avoidance mode
critical_distance_threshold = 0.15  # Emergency stop distance
```

#### Potential Field Parameters
```python
k_rep = 0.4   # Repulsive force strength
k_att = 0.5   # Attractive force strength
d0 = 0.6      # Influence distance
```

#### Motion Parameters
```python
max_linear_velocity = 0.15   # m/s
max_angular_velocity = 0.8   # rad/s
waypoint_tolerance = 0.35    # m
```

### World and Map Configuration

- **Default World**: `turtlebot3_world.world`
- **Map Files**: Located in `maps/` directory
- **RViz Config**: Custom configuration in `rviz/config.rviz`

## 📁 Package Structure

```
assignment_turtlebot3/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── navigation.launch       # Main navigation launch file
├── scripts/
│   ├── global_planner.py      # A* path planning
│   ├── kinematic_controller.py # Motion control
│   ├── potential_fields.py    # Obstacle avoidance
│   └── navigator.py           # Main navigation node
├── maps/
│   ├── map.yaml              # Map metadata
│   └── map.pgm               # Occupancy grid map
├── rviz/
│   └── config.rviz           # RViz visualization config
└── worlds/
    └── turtlebot3_world.world # Custom Gazebo world
```

## 🔬 Algorithm Details

### A* Global Planning
- **Grid-based** planning on occupancy maps
- **Obstacle inflation** for robot safety margins
- **Optimal path** finding with configurable heuristics

### Potential Fields Local Planning
- **Multi-zone repulsive forces**:
  - Critical zone (< 0.3m): Strong avoidance
  - Danger zone (< 0.45m): Moderate avoidance  
  - Warning zone (< 0.6m): Gentle steering
- **Force balancing** to prevent oscillations
- **Deadlock detection** and recovery

### Sequential Motion Control
- **Two-phase movement**:
  1. **Rotation phase**: Align with target direction
  2. **Forward phase**: Move toward waypoint
- **Precise waypoint tracking** with configurable tolerances

## 🐛 Troubleshooting

### Common Issues

1. **Robot gets stuck in obstacle avoidance**:
   - Reduce `k_rep` parameter
   - Increase `obstacle_exit_threshold`

2. **Path planning fails**:
   - Check map inflation parameters
   - Verify start/goal positions are in free space

3. **Robot moves too slowly**:
   - Increase `max_linear_velocity`
   - Reduce `waypoint_tolerance`

### Debug Information

Enable debug output by monitoring these topics:
```bash
# Navigation status
rostopic echo /navigator/status

# Laser scan data
rostopic echo /scan

# Velocity commands
rostopic echo /cmd_vel
```

## 📊 Performance Metrics

The system provides real-time logging of:
- **Path planning time**: A* computation duration
- **Obstacle avoidance frequency**: Mode switching events
- **Navigation efficiency**: Path length vs. optimal
- **Safety metrics**: Minimum obstacle distances


## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


## 👥 Authors

- **MEHMET ER* - (https://github.com/memoer30/Astar_Potential_path/)

## 🙏 Acknowledgments

- ROBOTIS for TurtleBot3 platform
- ROS community for the navigation framework
- Contributors to open-source robotics libraries
