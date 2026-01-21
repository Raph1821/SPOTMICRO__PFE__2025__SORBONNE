# QUICK START GUIDE - Spot Micro Autonomous SLAM

## 🎯 TL;DR - 5 Minute Setup

### 1. Install Dependencies
```bash
sudo apt-get install ros-noetic-navigation ros-noetic-move-base \
  ros-noetic-costmap-2d ros-noetic-clear-costmap-recovery \
  ros-noetic-rotate-recovery ros-noetic-map-server
```

### 2. Build Package
```bash
cd ~/catkin_ws
catkin build spot_micro_autonomous_slam
source devel/setup.bash
```

### 3. Launch (Simulation)
```bash
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=true auto_explore:=true
```

### 4. Launch (Real Robot)
```bash
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=false
# Then start exploration:
rostopic pub -1 /start_exploration std_msgs/Bool "data: true"
```

---

## 📋 Command Cheat Sheet

| Task | Command |
|------|---------|
| Build | `catkin build spot_micro_autonomous_slam` |
| Launch (Sim, auto) | `roslaunch spot_micro_autonomous_slam autonomous_slam.launch use_pybullet:=true auto_explore:=true` |
| Launch (Real, manual) | `roslaunch spot_micro_autonomous_slam autonomous_slam.launch use_pybullet:=false` |
| Start exploration | `rostopic pub -1 /start_exploration std_msgs/Bool "data: true"` |
| Stop exploration | `rostopic pub -1 /stop_exploration std_msgs/Bool "data: true"` |
| Save map now | `rostopic pub -1 /save_map_now std_msgs/Bool "data: true"` |
| Check status | `rostopic echo /exploration_status` |
| List nodes | `rosnode list \| grep autonomous` |
| View map location | `ls -la ~/spot_micro_maps/` |

---

## 🔴 Common Issues & Fixes

### Issue: Build Fails - Missing Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
catkin clean
catkin build spot_micro_autonomous_slam
```

### Issue: "move_base: command not found"
```bash
# Make sure navigation is installed
sudo apt-get install ros-noetic-navigation
# Rebuild
catkin build spot_micro_autonomous_slam
```

### Issue: Robot Not Moving / RViz shows no frontiers
1. Check lidar is publishing:
   ```bash
   rostopic hz /scan
   # Should show 3-5 Hz updates
   ```

2. Check SLAM map is publishing:
   ```bash
   rostopic hz /map
   # Should show updates every few seconds
   ```

3. Check exploration node is running:
   ```bash
   rosnode list | grep autonomous_explorer
   ```

4. Check status:
   ```bash
   rostopic echo /exploration_status
   ```

### Issue: "No frontiers found" - Exploration stops immediately
**Cause**: Robot hasn't moved/explored yet, or all space is already mapped

**Fix**:
- Manually move robot with keyboard to explore some area first
- Then start autonomous exploration
- Or increase `max_frontier_distance` to ~8m

### Issue: Maps not saving to ~/spot_micro_maps/
1. Check directory exists:
   ```bash
   ls -la ~/spot_micro_maps/
   mkdir -p ~/spot_micro_maps/  # Create if needed
   ```

2. Check map_saver is running:
   ```bash
   rosnode list | grep map_saver
   ```

3. Manually trigger save:
   ```bash
   rostopic pub -1 /save_map_now std_msgs/Bool "data: true"
   ```

### Issue: PyBullet crashes / window freezes
```bash
# Try headless mode (no GUI)
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=true use_gui:=false
```

---

## 🎮 Parameter Tuning Quick Reference

### Want Faster Exploration?
Edit `autonomous_slam.launch`:
```xml
<param name="max_frontier_distance" value="8.0"/>    <!-- Increase from 5.0 -->
<param name="min_frontier_distance" value="0.2"/>    <!-- Decrease from 0.3 -->
```

Edit `config/base_local_planner_params.yaml`:
```yaml
max_vel_x: 0.5          # Increase from 0.4
max_vel_theta: 0.5      # Increase from 0.35
```

### Want Safer/Slower Navigation?
Edit `config/base_local_planner_params.yaml`:
```yaml
max_vel_x: 0.2          # Decrease from 0.4
max_vel_theta: 0.2      # Decrease from 0.35
inflation_radius: 0.3   # Increase from 0.2
```

### Want Better Maps (Slower)?
Edit `config/costmap_common_params.yaml`:
```yaml
resolution: 0.025       # Decrease from 0.05 (2x detail, 4x slower)
```

---

## 🐛 Debug Mode

Enable verbose logging:

```bash
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=true \
  debug_mode:=true
```

Then monitor in separate terminals:

```bash
# Terminal 1: Explorer debug output
rosnode list | grep autonomous_explorer
rostopic echo -p /exploration_status

# Terminal 2: Navigation status
rostopic echo -p /move_base/status

# Terminal 3: Frontiers
rostopic echo -p /frontiers

# Terminal 4: Map updates
rostopic hz /map
```

---

## 📊 Monitoring Exploration

### Real-time Status
```bash
# Watch exploration progress
watch -n 1 "rostopic echo -p /exploration_status"

# Count frontiers
watch -n 1 "rostopic list | grep frontiers"
```

### Check Navigation
```bash
# See velocity commands being sent
rostopic echo /cmd_vel

# See where robot thinks it is
rostopic echo /tf | grep base_link

# See planned path
# (visible in RViz as white line from robot)
```

### Monitor Map Growth
```bash
# Watch map being saved
watch -n 5 "ls -ltr ~/spot_micro_maps/ | tail -5"

# Check total map size
du -sh ~/spot_micro_maps/
```

---

## 🎯 Testing Workflow

### Step 1: Test in Simulation First
```bash
# Launch with PyBullet
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=true \
  use_gui:=true \
  auto_explore:=true
```

Expected: Robot should move around in simulated environment

### Step 2: Verify ROS Topics
```bash
# In new terminal - watch ROS graph
roswtf

# Check all topics are connected
rostopic list | wc -l  # Should show ~30+ topics
```

### Step 3: Test Manual Movement First
```bash
# Launch without auto-explore
roslaunch spot_micro_autonomous_slam autonomous_slam.launch \
  use_pybullet:=false \
  auto_explore:=false

# In another terminal, manually move robot with existing keyboard control
roslaunch spot_micro_keyboard_command keyboard_command.launch
# Use W/A/S/D to move robot around
```

### Step 4: Start Autonomous Exploration
```bash
# After robot has moved around a bit:
rostopic pub -1 /start_exploration std_msgs/Bool "data: true"
```

---

## 📁 Directory Structure Reference

```
spot_micro_autonomous_slam/
├── README.md                          ← Overview
├── USAGE_GUIDE.md                     ← Detailed guide
├── QUICK_START.md                     ← This file
│
├── CMakeLists.txt                     ← Build config (don't edit unless adding files)
├── package.xml                        ← Dependencies (edit to add new deps)
│
├── config/                            ← All parameters editable here
│   ├── costmap_common_params.yaml     ← Robot size, inflation
│   ├── local_costmap_params.yaml      ← Local planner costmap
│   ├── global_costmap_params.yaml     ← Global planner costmap
│   ├── base_local_planner_params.yaml ← SPEED & ACCELERATION
│   └── move_base_params.yaml          ← Planning settings
│
├── launch/                            ← How things start
│   ├── autonomous_slam.launch         ← Main entry point (edit for defaults)
│   └── move_base.launch               ← Navigation setup
│
└── scripts/                           ← Python nodes
    ├── autonomous_explorer.py         ← Main frontier algorithm
    ├── frontier_exploration_client.py ← Alternative server
    └── map_saver.py                   ← Auto map saving
```

---

## 🔧 File Editing Quick Reference

### To Change Exploration Behavior
Edit: `launch/autonomous_slam.launch` (lines ~60-70)

### To Change Robot Speed
Edit: `config/base_local_planner_params.yaml` (lines 5-8)

### To Change Map Save Interval
Edit: `launch/autonomous_slam.launch` (line ~93)

### To Change Robot Footprint
Edit: `config/costmap_common_params.yaml` (line ~12)

---

## ⏱️ Expected Performance

| Scenario | Time | Notes |
|----------|------|-------|
| Small room (5m×5m) | 2-3 min | PyBullet simulation |
| Medium room (10m×10m) | 5-10 min | Real robot, depends on obstacles |
| Large apartment | 30+ min | May need to increase timeout |
| Exploration timeout | 10 min | Default, editable in launch file |

---

## 🎓 Learning Path

1. **Start Here**: Read this Quick Start
2. **Then**: Run simulation mode (see commands above)
3. **Next**: Read [README.md](README.md) for full context
4. **Deep Dive**: Read [USAGE_GUIDE.md](USAGE_GUIDE.md) for detailed info
5. **Tune**: Edit config files based on your environment

---

## 🆘 Get Help

1. **Check logs**: `rosout` aggregates all node output
2. **Monitor topics**: `rostopic echo /exploration_status` shows real-time status
3. **Verify setup**: `rosnode list` and `rostopic list` show all ROS components
4. **Debug script**: Run nodes individually with output to terminal

---

## 📞 Emergency Stop

If robot starts exploring unexpectedly:

```bash
# Immediate stop
rostopic pub -1 /stop_exploration std_msgs/Bool "data: true"

# Or kill the node
rosnode kill autonomous_explorer

# Or hard stop in keyboard control (if still running):
# Press 'u' then 'i' to enter idle mode
```

---

## ✅ Verification Checklist

Before real robot deployment:

- [ ] Package builds without errors: `catkin build`
- [ ] Simulation mode works: `use_pybullet:=true auto_explore:=true`
- [ ] Topics are publishing: `rostopic list | wc -l` > 20
- [ ] SLAM map updates: `rostopic hz /map` shows 1+ Hz
- [ ] Robot odometry works: `rostopic echo /odom`
- [ ] Motion commands accepted: `rostopic echo /cmd_vel`
- [ ] Maps save to correct directory: `ls ~/spot_micro_maps/`
- [ ] Manual control works first (keyboard)
- [ ] Robot explored some area before auto-explore
- [ ] Exploration stops correctly: `rostopic pub stop_exploration`

---

**Ready to go? Start with the 5-minute setup above! 🚀**
