# Spot Micro Autonomous SLAM

Autonomous frontier-based exploration and SLAM mapping package for the Spot Micro quadruped robot.

This package implements automatic map generation through hybrid frontier detection combined with reactive navigation, allowing the Spot Micro to autonomously explore and map unknown environments with intelligent goal-directed behavior.

## Features

- **Hybrid Exploration**: Combines frontier-based goal selection with reactive obstacle avoidance
- **Forward-Biased Frontier Selection**: Prioritizes exploration targets ahead of the robot to minimize unnecessary turns
- **Intelligent Turn Direction**: Goal-aware turning that considers both frontier objectives and obstacle clearance
- **Wall-Following Recovery**: Automatic stuck detection with wall-following behavior to escape tight spaces
- **Interactive Path Planning**: Click goals in RViz to override autonomous exploration and manually direct the robot
- **Goal Visualization**: Real-time markers showing frontier goals (green spheres) and manual goals (red spheres)
- **PyBullet Simulation**: Full support for simulation testing before real-world deployment
- **Configurable Navigation**: Tune speeds, obstacle distances, and frontier parameters for your environment

## System Architecture
```
┌─────────────────────────────────────────────────┐
│       Hybrid Exploration System                 │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─────────────┐        ┌─────────────────┐   │
│  │ RPLidar     │───────→│ Hector SLAM     │   │
│  │ /scan       │        │ /map            │   │
│  └─────────────┘        └─────────────────┘   │
│         ↓                         ↓             │
│  ┌─────────────────────────────────────────┐  │
│  │  Hybrid Explorer                        │  │
│  │  - Frontier Detection                   │  │
│  │  - Forward-Biased Goal Selection        │  │
│  │  - Reactive Obstacle Avoidance          │  │
│  │  - Wall-Following Recovery              │  │
│  └─────────────────────────────────────────┘  │
│         ↓                         ↓             │
│  ┌─────────────┐        ┌─────────────────┐   │
│  │ Path        │───────→│ Motion Control  │   │
│  │ Planner     │        │ /cmd_vel        │   │
│  │ (Optional)  │        │                 │   │
│  └─────────────┘        └─────────────────┘   │
│         ↑                                       │
│  ┌─────────────────────────────────────────┐  │
│  │  RViz Goal Clicks                       │  │
│  │  /move_base_simple/goal                 │  │
│  └─────────────────────────────────────────┘  │
│                                                 │
└─────────────────────────────────────────────────┘
```

## Installation

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Spot Micro motion control package: `spot_micro_motion_cmd`
- Spot Micro RViz visualization: `spot_micro_rviz`
- Python dependencies:
```bash
sudo apt-get install python3-scipy python3-numpy
```

### Build
```bash
cd ~/catkin_ws
catkin build spot_micro_autonomous_slam
source devel/setup.bash
```

## Quick Start

### Simulation Mode (PyBullet)
```bash
# Terminal 1: Launch exploration with simulation
roslaunch spot_micro_autonomous_slam simple_exploration.launch \
  use_pybullet:=true \
  enabled:=true \
  rviz:=true

# Terminal 2 (optional): Launch path planner for manual goals
rosrun spot_micro_autonomous_slam path_planner.py
```

### Real Robot Mode
```bash
# Terminal 1: Start SLAM and exploration system
roslaunch spot_micro_autonomous_slam simple_exploration.launch \
  use_pybullet:=false \
  enabled:=false \
  rviz:=true

# Terminal 2: Enable exploration when ready
rostopic pub -1 /enable_exploration std_msgs/Bool "data: true"

# Terminal 3 (optional): Launch path planner for manual goals
rosrun spot_micro_autonomous_slam path_planner.py
```

## Configuration

### Launch File Parameters (simple_exploration.launch)
```xml
    
             
          
```

### Hybrid Explorer Parameters

Configure in the launch file or via rosparam:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | false | Start exploration automatically on launch |
| `max_forward_speed` | 0.35 m/s | Maximum linear velocity |
| `max_turn_speed` | 0.50 rad/s | Maximum angular velocity |
| `obstacle_distance` | 0.4 m | Distance threshold for obstacle detection |
| `turn_angle` | π/2 rad | Turn angle when obstacle detected (90°) |
| `frontier_min_size` | 5 cells | Minimum frontier cluster size |
| `max_exploration_distance` | 4.0 m | Maximum distance to consider frontiers |
| `goal_reached_threshold` | 0.2 m | Distance to consider goal reached |

### Frontier Scoring Weights

The hybrid explorer scores frontiers based on three factors (configured in code):

- **20%** - Distance (closer is better)
- **40%** - Size (larger frontiers preferred)
- **40%** - Angular alignment (frontiers ahead preferred)

### Path Planner Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_forward_speed` | 0.35 m/s | Speed when navigating to clicked goal |
| `max_turn_speed` | 0.50 rad/s | Turn speed for manual goals |
| `goal_threshold` | 0.3 m | Distance to consider manual goal reached |

## ROS Topics

### Published

- `/cmd_vel` (Twist): Velocity commands to robot
- `/frontiers` (MarkerArray): Frontier goal visualization (green sphere)
- `/goal_marker` (MarkerArray): Manual goal visualization (red sphere)
- `/enable_exploration` (Bool): Exploration enable/disable state

### Subscribed

- `/scan` (LaserScan): Lidar sensor data for obstacle detection
- `/map` (OccupancyGrid): SLAM map from Hector SLAM
- `/enable_exploration` (Bool): Enable/disable autonomous exploration
- `/move_base_simple/goal` (PoseStamped): Manual goals from RViz (path planner)

### TF Frames

- `map` → `base_footprint`: Robot pose for navigation

## Usage Examples

### Enable/Disable Exploration
```bash
# Enable autonomous exploration
rostopic pub -1 /enable_exploration std_msgs/Bool "data: true"

# Disable exploration
rostopic pub -1 /enable_exploration std_msgs/Bool "data: false"
```

### Send Manual Goals via RViz

1. Ensure `path_planner.py` is running
2. In RViz, click "2D Nav Goal" button
3. Click destination on map
4. Robot will navigate to clicked point (exploration temporarily paused)
5. After reaching goal, exploration resumes automatically



## Future Enhancements

- [ ] Multi-goal planning (explore multiple areas in sequence)
- [ ] Information gain optimization (prefer high-info frontiers)
- [ ] Loop closure detection and map correction
- [ ] Dynamic costmap reconfiguration
- [ ] Path optimization to minimize total distance
- [ ] Support for multiple robot coordination

## References

- [ROS Navigation](http://wiki.ros.org/navigation)
- [Move Base Documentation](http://wiki.ros.org/move_base)
- [Hector SLAM](http://wiki.ros.org/hector_slam)
- [Frontier Exploration](http://wiki.ros.org/frontier_exploration)

## License

Apache License 2.0 - Same as parent Spot Micro project


For detailed usage instructions, see [USAGE_GUIDE.md](USAGE_GUIDE.md)




