# Path Planning & Exploration Integration Guide

## Overview

This system integrates autonomous exploration with user-directed path planning for the Spot Micro robot. The robot can:

1. **Exploration Mode** (Default): Autonomously explores unknown areas using laser scan-based obstacle avoidance
2. **Path Planning Mode**: Navigates to user-clicked destinations in RVIZ
3. **Seamless Switching**: User can switch between modes at any time

## Architecture

### Components

- **simple_explorer.py**: Handles autonomous exploration with obstacle avoidance
- **path_planner.py**: Manages user-directed navigation and mode switching
- **path_planner_control.py**: Command-line utility for testing and scripted goals

### Communication Flow

```
RVIZ (/clicked_point) ──────────┐
                                 │
                                 v
                          path_planner.py
                                 │
    ┌──────────────────────────┬─┴─┬──────────────────────────┐
    │                          │   │                          │
    v                          v   v                          v
move_base              simple_explorer        /enable_exploration
(navigation)         (exploration control)    (True/False)
    │                          │
    └──────────────┬───────────┘
                   v
              /cmd_vel
           (robot motion)
```

## Usage

### 1. Launch the System

```bash
# Using the combined launch file
roslaunch spot_micro_autonomous_slam exploration_with_path_planning.launch

# Or launch individually
roslaunch spot_micro_autonomous_slam simple_explorer.launch
roslaunch spot_micro_autonomous_slam path_planner.launch
```

### 2. Set a Goal via RVIZ

In RVIZ:
1. Select "Publish Point" tool
2. Click on any location on the map
3. The robot will:
   - Switch to path planning mode
   - Stop exploration
   - Navigate to the clicked point using move_base

### 3. Return to Exploration

After reaching the goal (or anytime during navigation):

**Option A: Command Line**
```bash
rostopic pub /return_to_exploration std_msgs/Bool "data: true"
```

**Option B: Python Script**
```bash
python3 path_planner_control.py return
```

**Option C: Programmatically**
```python
import rospy
from std_msgs.msg import Bool

pub = rospy.Publisher('/return_to_exploration', Bool, queue_size=1)
pub.publish(Bool(True))
```

### 4. Set Goals Programmatically

```bash
# Set goal at coordinates (1.5, 2.0)
python3 path_planner_control.py goal 1.5 2.0
```

Or in Python:
```python
import rospy
from geometry_msgs.msg import PointStamped, Point

pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
msg = PointStamped()
msg.header.frame_id = 'map'
msg.header.stamp = rospy.Time.now()
msg.point = Point(x=1.5, y=2.0, z=0.0)
pub.publish(msg)
```

## Parameters

### simple_explorer.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | False | Start in exploration mode |
| `max_forward_speed` | 0.15 m/s | Maximum forward velocity |
| `max_turn_speed` | 0.3 rad/s | Maximum angular velocity |
| `obstacle_distance` | 0.4 m | Minimum safe distance to obstacles |
| `safe_distance` | 0.5 m | Safety margin |

### path_planner.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `exploration_enabled` | True | Start in exploration mode |
| `max_forward_speed` | 0.15 m/s | Maximum forward velocity |
| `max_turn_speed` | 0.3 rad/s | Maximum angular velocity |
| `goal_tolerance` | 0.3 m | Distance tolerance for goal reached |

## Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Robot velocity commands |
| `/current_goal` | PointStamped | Currently active goal |
| `/planner_mode` | String | Current mode: "exploration" or "path_planning" |
| `/enable_exploration` | Bool | Control exploration state |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/clicked_point` | PointStamped | RVIZ clicked points (new goals) |
| `/return_to_exploration` | Bool | Command to return to exploration |
| `/move_base/status` | GoalStatusArray | Move base feedback |
| `/scan` | LaserScan | Laser scanner data |
| `/map` | OccupancyGrid | Occupancy grid map |
| `/enable_exploration` | Bool | Exploration enable control |

## System Behavior

### In Exploration Mode

```
┌─────────────────────────────────┐
│   EXPLORATION MODE (Default)    │
└─────────────────────────────────┘
        │
        └─→ Receive /clicked_point
            │
            └─→ Disable exploration
            └─→ Enter path planning mode
```

The robot:
- Autonomously explores using laser scan-based obstacle avoidance
- Detects obstacles using front laser scan sector
- Turns left or right based on open space analysis
- Recovers from stuck situations with increasing turn angles

### In Path Planning Mode

```
┌──────────────────────────────────┐
│   PATH PLANNING MODE (Manual)    │
└──────────────────────────────────┘
        │
        ├─→ Use move_base for navigation (if available)
        │   or manual PID-based navigation
        │
        ├─→ Monitor goal status
        │
        └─→ Receive /return_to_exploration
            │
            └─→ Cancel goal
            └─→ Stop robot
            └─→ Re-enable exploration
```

The robot:
- Navigates directly to clicked coordinates
- Uses move_base action server for path planning
- Falls back to manual PID navigation if move_base unavailable
- Notifies when goal is reached
- Waits for user command to return to exploration

## Mode Switching Logic

```
START (Exploration Mode)
    │
    ├─ User clicks on map
    │   └─ Switches to Path Planning Mode
    │       └─ Disables autonomous exploration
    │       └─ Sends goal to move_base
    │
    ├─ Goal reached OR user publishes /return_to_exploration
    │   └─ Switches back to Exploration Mode
    │       └─ Cancels active goal
    │       └─ Re-enables autonomous exploration
    │
    └─ Loop continues...
```

## Troubleshooting

### move_base not available

If move_base is not running, the path planner will use manual navigation with simple PID control:
- Angular: PID-based heading adjustment (proportional control)
- Linear: Reduced speed while turning, full speed when aligned

To enable move_base:
```bash
roslaunch spot_micro_motion_cmd move_base.launch
```

### Robot not responding to goals

1. Check if path_planner node is running:
   ```bash
   rosnode list | grep path_planner
   ```

2. Verify topics are being published:
   ```bash
   rostopic list | grep -E "clicked_point|cmd_vel|planner_mode"
   ```

3. Check RVIZ tool selection - ensure "Publish Point" tool is selected

### Exploration won't restart

Verify the enable_exploration topic is working:
```bash
rostopic pub /enable_exploration std_msgs/Bool "data: true"
```

Monitor the simple_explorer logs:
```bash
rosnode info /simple_explorer
```

## Advanced Configuration

### Automatic Return to Exploration

To automatically return to exploration after goal reached:

Edit `path_planner.py` and uncomment in the `run()` method:
```python
if self.goal_reached:
    return_delay = rospy.get_param('~auto_return_delay', 0)  # seconds
    if return_delay > 0:
        rospy.sleep(return_delay)
        self.return_to_exploration_mode()
```

Then launch with:
```bash
roslaunch spot_micro_autonomous_slam exploration_with_path_planning.launch
rosrun spot_micro_autonomous_slam path_planner.py _auto_return_delay:=5.0
```

### Waypoint Navigation

Create a waypoint list and navigate through them:

```python
#!/usr/bin/env python3
import rospy
from path_planner_control import PathPlannerControl
import time

waypoints = [(1.0, 1.0), (2.5, 1.5), (2.5, 3.0), (1.0, 3.0)]
control = PathPlannerControl()

for i, (x, y) in enumerate(waypoints):
    print(f"Going to waypoint {i+1}/{len(waypoints)}: ({x}, {y})")
    control.set_goal(x, y)
    rospy.sleep(30)  # Wait 30 seconds for goal
    if control.get_status()['mode'] == 'path_planning':
        print("Goal reached, moving to next waypoint...")
        control.return_to_exploration()
        rospy.sleep(2.0)

print("All waypoints completed!")
```

## Files

- `simple_explorer.py` - Autonomous exploration node
- `path_planner.py` - Path planning and mode control node
- `path_planner_control.py` - Command-line control utility
- `exploration_with_path_planning.launch` - Combined launch file

## Future Enhancements

1. **Costmap-based Navigation**: Use global costmap for better path planning
2. **Obstacle Avoidance in Path Planning**: Dynamic obstacle avoidance during goal navigation
3. **Multi-Goal Queue**: Set multiple sequential goals
4. **Persistence**: Save explored maps and path history
5. **Remote Control**: Web interface for goal selection
6. **Performance Metrics**: Track exploration coverage and navigation efficiency

## References

- [ROS move_base](http://wiki.ros.org/move_base)
- [RVIZ Interaction Tutorial](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- [Spot Micro Project](https://github.com/OpenQuadruped/spot_micro_exoskeleton)
