# Spot Micro Autonomous SLAM

Autonomous frontier-based exploration and SLAM mapping package for the Spot Micro quadruped robot.

This package implements automatic map generation through frontier detection and intelligent navigation, allowing the Spot Micro to autonomously explore and map unknown environments.

## Features

- **Frontier Detection**: Identifies boundaries between known and unknown space
- **Intelligent Path Planning**: Uses ROS navigation stack for safe autonomous navigation  
- **Automatic Map Saving**: Periodically saves maps during exploration
- **PyBullet Simulation**: Full support for simulation testing before real-world deployment
- **Configurable Exploration**: Tune exploration parameters for your environment
- **Status Monitoring**: Real-time feedback on exploration progress

## System Architecture

```
┌─────────────────────────────────────────────────┐
│       Autonomous SLAM System                    │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─────────────┐        ┌─────────────────┐   │
│  │ RPLidar     │───────→│ Hector SLAM     │   │
│  │ /scan       │        │ /map            │   │
│  └─────────────┘        └─────────────────┘   │
│         ↓                         ↑             │
│  ┌─────────────┐        ┌─────────────────┐   │
│  │ Motion Cmd  │───────→│ Move Base       │   │
│  │ Odometry    │        │ Navigation      │   │
│  └─────────────┘        └─────────────────┘   │
│         ↑                         ↓             │
│  ┌─────────────────────────────────────────┐  │
│  │  Autonomous Explorer                   │  │
│  │  - Frontier Detection                  │  │
│  │  - Goal Selection                      │  │
│  │  - Exploration Control                 │  │
│  └─────────────────────────────────────────┘  │
│         ↓                                      │
│  ┌─────────────────────────────────────────┐  │
│  │  Map Saver                              │  │
│  │  - Periodic Saves                       │  │
│  │  - Completion Saves                     │  │
│  │  - Archive Management                  │  │
│  └─────────────────────────────────────────┘  │
│                                                 │
└─────────────────────────────────────────────────┘
```

## Installation

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Spot Micro motion control package: `spot_micro_motion_cmd`
- Spot Micro RViz visualization: `spot_micro_rviz`
- Navigation stack packages:

```bash
sudo apt-get install ros-noetic-navigation ros-noetic-move-base \
  ros-noetic-costmap-2d ros-noetic-clear-costmap-recovery \
  ros-noetic-rotate-recovery ros-noetic-map-server
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
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=true \
  auto_explore:=true \
  use_rviz:=true
```

### Real Robot Mode

```bash
# Terminal 1: Start autonomous SLAM
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=false \
  auto_explore:=false \
  use_rviz:=true

# Terminal 2: Start exploration
rostopic pub -1 /start_exploration std_msgs/Bool "data: true"
```

## Configuration

### Main Parameters (autonomous_slam.launch)

```xml
<arg name="use_pybullet" default="false"/>        <!-- Use simulation -->
<arg name="auto_explore" default="false"/>         <!-- Start auto-exploration -->
<arg name="exploration_timeout" value="600"/>      <!-- Timeout in seconds -->
<arg name="map_save_dir" default="~/spot_micro_maps"/>
```

### Exploration Parameters (autonomous_explorer.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `exploration_mode` | frontier | Exploration algorithm |
| `goal_tolerance` | 0.5 m | Goal reaching tolerance |
| `min_frontier_distance` | 0.3 m | Minimum frontier distance |
| `max_frontier_distance` | 5.0 m | Maximum frontier distance |
| `exploration_timeout` | 600 s | Timeout (0 = unlimited) |

### Navigation Parameters (config/)

Configuration files for move_base and costmaps are pre-tuned for Spot Micro:

- **Robot size**: 15cm × 25cm
- **Max velocity**: 0.4 m/s forward, 0.35 rad/s rotation
- **Safety buffer**: 20cm inflation radius
- **Map resolution**: 5cm per cell

Fine-tune in `config/` directory for your specific environment.

## ROS Topics

### Published

- `/exploration_status` (String): Exploration state updates
- `/exploration_goal` (PoseStamped): Current navigation goal
- `/frontiers` (Point): Frontier positions for visualization
- `/map_save_status` (String): Map saving status
- `/move_base/status` (GoalStatusArray): Navigation status

### Subscribed

- `/map` (OccupancyGrid): SLAM map
- `/odom` (Odometry): Robot odometry
- `/scan` (LaserScan): Lidar data
- `/start_exploration` (Bool): Start exploration command
- `/stop_exploration` (Bool): Stop exploration command

## Usage Examples

### Start Exploration

```bash
rostopic pub -1 /start_exploration std_msgs/Bool "data: true"
```

### Stop Exploration

```bash
rostopic pub -1 /stop_exploration std_msgs/Bool "data: true"
```

### Save Map Manually

```bash
rostopic pub -1 /save_map_now std_msgs/Bool "data: true"
```

### Monitor Exploration

```bash
rostopic echo /exploration_status
```

### View RViz Visualization

- Launch includes RViz with pre-configured SLAM display
- Shows map, robot position, exploration goals, and frontiers
- Fronters displayed as point cloud at `/frontiers` topic

## Map Outputs

Maps are saved to `~/spot_micro_maps/` (configurable):

```
map_20260120_153022.yaml       # Map metadata
map_20260120_153022.pgm        # Map image (grayscale)
final_exploration_map.yaml     # Final map at completion
final_exploration_map.pgm
```

Use with ROS map_server:

```bash
rosrun map_server map_server ~/spot_micro_maps/final_exploration_map.yaml
```

## How It Works

### Frontier Detection Algorithm

1. **Scan occupancy grid** from Hector SLAM output
2. **Identify frontiers**: Unknown cells (-1) adjacent to free space (0)
3. **Cluster frontiers**: Group nearby frontier cells
4. **Compute centers**: Calculate cluster centroid positions

### Exploration Strategy

1. **Detect frontiers** in current map
2. **Select best frontier** based on:
   - Distance from robot (prefer closer)
   - Information gain heuristic
   - Distance constraints (0.3m - 5.0m)
3. **Send goal** to move_base navigation
4. **Navigate** using:
   - Global planner: navfn (finds path through map)
   - Local planner: DWA (real-time obstacle avoidance)
5. **Repeat** until no frontiers remain

## Nodes

### autonomous_explorer.py

Main frontier detection and exploration control node.

**Publishes**: `/exploration_status`, `/exploration_goal`, `/frontiers`  
**Subscribes**: `/map`, `/odom`, `/start_exploration`, `/stop_exploration`

**Features**:
- Frontier cell detection using occupancy grid analysis
- Frontier clustering and center calculation
- Goal selection with distance constraints
- Exploration state machine
- Stuck detection and recovery

### map_saver.py

Automatic map archive management.

**Publishes**: `/map_save_status`  
**Subscribes**: `/map`, `/exploration_status`, `/save_map_now`

**Features**:
- Periodic map saving (configurable interval)
- Save on exploration completion
- Save on timeout/failure
- On-demand saving via topic
- Map metadata logging

### frontier_exploration_client.py

Alternative frontier exploration using ROS frontier_exploration server.

**Optional**: Only used if frontier_exploration package is installed.

## Troubleshooting

### Map Not Updating

1. Check Hector SLAM is publishing to `/map`:
   ```bash
   rostopic list | grep map
   rostopic hz /map
   ```

2. Verify lidar is publishing `/scan`:
   ```bash
   rostopic hz /scan
   ```

### Robot Not Moving

1. Check move_base is publishing `/cmd_vel`:
   ```bash
   rostopic echo /cmd_vel
   ```

2. Verify motion_cmd node is running and responding

### No Frontiers Found

1. Ensure robot has explored some area
2. Check `/exploration_status` for error messages
3. Increase `max_frontier_distance` parameter
4. Verify occupancy grid has enough unknown cells

## Parameters Tuning

### For Faster Exploration
- Increase `max_frontier_distance` (up to ~10m)
- Decrease `goal_tolerance` (down to ~0.25m)
- Increase `exploration_timeout` if needed

### For Safer Navigation
- Decrease `max_frontier_distance` (down to ~2m)
- Increase `goal_tolerance` (up to ~1m)
- Reduce `max_vel_x` and `max_vel_theta` in base_local_planner_params

### For Better Map Quality
- Use lower map resolution (0.025m in costmap_common_params)
- Increase `save_interval` for more frequent saves
- Run with `exploration_timeout: 0` for unlimited exploration

## Performance Considerations

| Factor | Impact | Tuning |
|--------|--------|--------|
| Frontier distance | Detection range | Increase for larger spaces |
| Map resolution | Accuracy vs. speed | Lower = better but slower |
| Planner frequency | Responsiveness | 1Hz good for slow exploration |
| Controller frequency | Smoothness | 5Hz minimum for good control |

## Future Enhancements

- [ ] Multi-goal planning (explore multiple areas in sequence)
- [ ] Information gain optimization (prefer high-info frontiers)
- [ ] Loop closure detection and map correction
- [ ] Dynamic costmap reconfiguration
- [ ] Path optimization to minimize total distance
- [ ] Support for multiple robot coordination

## Contributing

To add features or improvements:

1. Fork/branch the repository
2. Modify relevant files (autonomous_explorer.py, map_saver.py, configs)
3. Test thoroughly in simulation first
4. Document changes in USAGE_GUIDE.md

## References

- [ROS Navigation](http://wiki.ros.org/navigation)
- [Move Base Documentation](http://wiki.ros.org/move_base)
- [Hector SLAM](http://wiki.ros.org/hector_slam)
- [Frontier Exploration](http://wiki.ros.org/frontier_exploration)

## License

Apache License 2.0 - Same as parent Spot Micro project

## Authors

Autonomous SLAM Package for Spot Micro  
Based on Spot Micro PFE 2025 project

---

For detailed usage instructions, see [USAGE_GUIDE.md](USAGE_GUIDE.md)
