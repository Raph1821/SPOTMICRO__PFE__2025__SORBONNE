# Spot Micro Robot Installation Guide

Complete installation guide for Spot Micro Quadruped Robot on Raspberry Pi 4B 8GB with Ubuntu 20.04 and ROS Noetic.

---

## Table of Contents

1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [Installation Steps](#installation-steps)
   - [Step 1: Install Ubuntu 20.04 on Raspberry Pi](#step-1-install-ubuntu-2004-on-raspberry-pi)
   - [Step 2: Install ROS Noetic](#step-2-install-ros-noetic)
   - [Step 3: Install Dependencies](#step-3-install-dependencies)
   - [Step 4: Setup Catkin Workspace](#step-4-setup-catkin-workspace)
   - [Step 5: Clone Project Repository](#step-5-clone-project-repository)
   - [Step 6: Make Scripts Executable](#step-6-make-scripts-executable)
   - [Step 7: Build the Workspace](#step-7-build-the-workspace)
   - [Step 8: Configure Environment](#step-8-configure-environment)
4. [Testing the Installation](#testing-the-installation)
5. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

- **Raspberry Pi 4B 8GB** (recommended for ROS Noetic)
- MicroSD card (32GB or larger, Class 10 or better)
- 12x MG996R servos (or compatible)
- PCA9685 I2C PWM board
- RPLidar A1 (for SLAM)
- PS4 or Xbox controller (optional, for joystick control)
- 2S LiPo battery (4000mAh)
- 2 UBEC (HKU5 5V/5A ubec) 
- I2C LCD display 16x2 (optional)
- Ultrasonic sensor 
- IMU (MPU6050)

---

## Software Requirements

- **Ubuntu 20.04 LTS Server** (64-bit for ARM)
- **ROS Noetic Ninjemys** (latest stable version)
- Python 3.8+
- Git

---

## Installation Steps

### Step 1: Install Ubuntu 20.04 on Raspberry Pi

1. **Download Ubuntu 20.04 Server (64-bit) for Raspberry Pi:**
   - Download from: https://ubuntu.com/download/raspberry-pi
   - Choose: Ubuntu Server 20.04.X LTS (64-bit)

2. **Flash the image to microSD card:**
   ```bash
   # Using Raspberry Pi Imager (recommended)
   # Or use dd command on Linux:
   sudo dd if=ubuntu-20.04-preinstalled-server-arm64+raspi.img of=/dev/sdX bs=4M status=progress
   sync
   ```

3. **Boot Raspberry Pi and complete initial setup:**
   ```bash
   # Default credentials:
   # Username: ubuntu
   # Password: ubuntu (you'll be prompted to change it)
   
   # Update system
   sudo apt update
   sudo apt upgrade -y
   ```

4. **Configure WiFi (if needed):**
   ```bash
   sudo nano /etc/netplan/50-cloud-init.yaml
   ```
   
   Add WiFi configuration:
   ```yaml
   network:
     version: 2
     wifis:
       wlan0:
         dhcp4: true
         optional: true
         access-points:
           "YourSSID":
             password: "YourPassword"
   ```
   
   Apply configuration:
   ```bash
   sudo netplan apply
   ```

5. **Enable I2C (for servos and LCD):**
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options → I2C → Enable
   
   # Verify I2C is enabled
   sudo apt install i2c-tools
   sudo i2cdetect -y 1
   ```

---

### Step 2: Install ROS Noetic

1. **Setup ROS repository:**
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

2. **Add ROS key:**
   ```bash
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

3. **Update package index:**
   ```bash
   sudo apt update
   ```

4. **Install ROS Noetic Desktop (for development) or Base (for robot):**
   
   **On Development PC:**
   ```bash
   sudo apt install ros-noetic-desktop-full -y
   ```
   
   **On Raspberry Pi (headless):**
   ```bash
   sudo apt install ros-noetic-ros-base -y
   ```

5. **Setup ROS environment:**
   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

6. **Install ROS dependencies:**
   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
   ```

7. **Initialize rosdep:**
   ```bash
   sudo rosdep init
   rosdep update
   ```

---

### Step 3: Install Dependencies

1. **Install catkin build tools:**
   ```bash
   sudo apt install python3-catkin-tools -y
   ```

2. **Install ROS packages:**
   ```bash
   sudo apt install -y \
       ros-noetic-joy \
       ros-noetic-rplidar-ros \
       ros-noetic-hector-slam \
       ros-noetic-joint-state-publisher \
       ros-noetic-joint-state-publisher-gui \
       ros-noetic-xacro \
       ros-noetic-robot-state-publisher \
       ros-noetic-rviz
   ```

3. **Install I2C libraries:**
   ```bash
   sudo apt install -y \
       libi2c-dev \
       i2c-tools \
       python3-smbus
   ```

4. **Install Python dependencies:**
   ```bash
   sudo apt install -y \
       python3-pip \
       python3-numpy \
       python3-matplotlib
   
   # Additional Python packages
   pip3 install --user matplotlib numpy
   ```

5. **Install development tools:**
   ```bash
   sudo apt install -y \
       git \
       cmake \
       libeigen3-dev
   ```

---

### Step 4: Setup Catkin Workspace

1. **Create workspace directory:**
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   ```

2. **Initialize catkin workspace:**
   ```bash
   source /opt/ros/noetic/setup.bash
   catkin init
   ```

3. **Configure catkin:**
   ```bash
   catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Verify workspace structure:**
   ```bash
   ls -la
   # Should show: build/ devel/ logs/ src/ .catkin_tools/
   ```

---

### Step 5: Clone Project Repository

1. **Navigate to workspace source directory:**
   ```bash
   cd ~/catkin_ws/src
   ```

2. **Clone the Spot Micro repository with submodules:**
   ```bash
   # Clone with --recursive flag to automatically initialize all submodules
   git clone --recursive https://github.com/Raph1821/SPOTMICRO__PFE__2025__SORBONNE__SACLAY.git
   
   # Navigate to repository
   cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY
   ```

3. **If submodules are missing, clone them manually:**
   
   **Check if submodules exist:**
   ```bash
   ls ros-i2cpwmboard/
   ls spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/
   ls spot_micro_plot/scripts/spot_micro_kinematics_python/
   ```
   
   **If folder doesn't exist or is empty, clone manually:**
   ```bash
   # Clone i2cpwm_board package
   git clone https://github.com/mentor-dyun/ros-i2cpwmboard.git ros-i2cpwmboard
   
   # Clone spot_micro_kinematics_cpp
   mkdir -p spot_micro_motion_cmd/libs
   cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_motion_cmd/libs
   git clone https://github.com/mike4192/spot_micro_kinematics_cpp.git 
   
   # Clone spot_micro_kinematics_python
   mkdir -p spot_micro_plot/scripts
   cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_plot/scripts
   git clone https://github.com/mike4192/spot_micro_kinematics_python.git 
   ```

4. **Verify all packages are present:**
   ```bash
   cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY
   # Check i2cpwm_board
   ls ros-i2cpwmboard/
   # Should show: CMakeLists.txt, package.xml, src/, etc.
   
   # Check kinematics libraries
   ls spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp/
   ls spot_micro_plot/scripts/spot_micro_kinematics_python/
   ```

---

### Step 6: Make Scripts Executable

**⚠️ This step only needs to be done ONCE after cloning the repository.**

**Run the automatic executable script:**

```bash
cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY
./make_py_executable.sh
```

**Expected output:**
```
Searching for Python scripts in all packages...
✅ All Python scripts are now executable!

Made executable:
./lcd_monitor/scripts/sm_lcd_node.py
./servo_move_keyboard/scripts/servoConfigTest.py
./servo_move_keyboard/scripts/servoMoveKeyboard.py
./spot_micro_joy/scripts/spotMicroJoystickMove.py
./spot_micro_keyboard_command/scripts/spotMicroKeyboardMove.py
./spot_micro_plot/scripts/spotMicroPlot.py
```

**Alternative (if script doesn't run):**
```bash
# First make the script itself executable
chmod +x make_py_executable.sh
# Then run it
./make_py_executable.sh
```

**Manual method (if script fails):**
```bash
chmod +x lcd_monitor/scripts/sm_lcd_node.py
chmod +x servo_move_keyboard/scripts/servoConfigTest.py
chmod +x servo_move_keyboard/scripts/servoMoveKeyboard.py
chmod +x spot_micro_joy/scripts/spotMicroJoystickMove.py
chmod +x spot_micro_keyboard_command/scripts/spotMicroKeyboardMove.py
chmod +x spot_micro_plot/scripts/spotMicroPlot.py
```

---

### Step 7: Build the Workspace

1. **Install package dependencies:**
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Build all packages:**
   ```bash
   catkin build
   ```
   
   **Expected output:**
   ```
   [build] Found '8' packages in 0.0 seconds.
   [build] Package table is up to date.
   Starting  >>> i2cpwm_board
   Starting  >>> lcd_monitor
   Starting  >>> servo_move_keyboard
   Starting  >>> spot_micro_plot
   Starting  >>> spot_micro_rviz
   Finished  <<< i2cpwm_board              [ 12.3 seconds ]
   Starting  >>> spot_micro_motion_cmd
   Finished  <<< lcd_monitor               [ 8.1 seconds ]
   Finished  <<< servo_move_keyboard       [ 8.4 seconds ]
   Finished  <<< spot_micro_plot           [ 8.2 seconds ]
   Finished  <<< spot_micro_rviz           [ 8.5 seconds ]
   Finished  <<< spot_micro_motion_cmd     [ 45.2 seconds ]
   Starting  >>> spot_micro_keyboard_command
   Starting  >>> spot_micro_joy
   Starting  >>> spot_micro_launch
   Finished  <<< spot_micro_keyboard_command [ 3.2 seconds ]
   Finished  <<< spot_micro_joy             [ 3.1 seconds ]
   Finished  <<< spot_micro_launch          [ 2.9 seconds ]
   [build] Summary: All 8 packages succeeded!
   ```

3. **If build fails, check errors and fix, then rebuild:**
   ```bash
   catkin clean  # Clean build files
   catkin build  # Rebuild
   ```

---

### Step 8: Configure Environment

1. **Source the workspace:**
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

2. **Add to bashrc for automatic sourcing:**
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   ```

3. **Verify ROS environment:**
   ```bash
   echo $ROS_PACKAGE_PATH
   # Should include: /home/ubuntu/catkin_ws/src:/opt/ros/noetic/share
   ```

4. **Configure servo calibration (if needed):**
   ```bash
   # Edit servo configuration file
   nano ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml
   
   # Adjust servo min/max/center values as needed
   ```

---

## Testing the Installation

### Test 1: Verify Packages are Found

```bash
rospack list | grep spot_micro
```

**Expected output:**
```
spot_micro_joy /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_joy
spot_micro_keyboard_command /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_keyboard_command
spot_micro_launch /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_launch
spot_micro_motion_cmd /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_motion_cmd
spot_micro_plot /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_plot
spot_micro_rviz /home/ubuntu/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_rviz
```

---

### Test 2: Launch Motion Command (Standalone Mode)

**Terminal 1:**
```bash
roslaunch spot_micro_motion_cmd motion_cmd.launch run_standalone:=true debug_mode:=true
```

**Expected output:**
```
[INFO] [WallTime: ...] Spot Micro Motion Cmd Node Starting...
[INFO] [WallTime: ...] State: IDLE
[INFO] [WallTime: ...] Debug mode enabled
```

---

### Test 3: Keyboard Control

**Terminal 1 (if not already running):**
```bash
roslaunch spot_micro_motion_cmd motion_cmd.launch run_standalone:=true debug_mode:=true
```

**Terminal 2:**
```bash
roslaunch spot_micro_keyboard_command keyboard_command.launch run_rviz:=true run_plot:=true
```

**Expected output:**
```
Spot Micro Keyboard Controller
-------------------------------
Controls:
  w : Move forward
  a : Turn left
  s : Move backward
  d : Turn right
  q : Idle mode
  e : Stand mode
  
Press Ctrl-C to quit
```

**Try pressing keys:** `e` (stand), `w` (forward), `q` (idle)

---

### Test 4: View ROS Node Graph

**Terminal 3:**
```bash
rqt_graph
```

**Expected nodes:**
- `/spot_micro_motion_cmd`
- `/spot_micro_keyboard_command`

**Expected topics:**
- `/state_command`
- `/cmd_vel`
- `/motion_command`
- `/servo_array`

---

### Test 5: RVIZ Visualization (Development PC only)

```bash
roslaunch spot_micro_keyboard_command keyboard_command.launch run_rviz:=true
```

**Expected:** RVIZ window opens showing 3D robot model

---

### Test 6: I2C Hardware Test (On Raspberry Pi with servos connected)

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Should show PCA9685 at address 0x40 (default)
```

**Test servo movement:**
```bash
roslaunch servo_move_keyboard keyboard_move.launch
```

---

## Troubleshooting

### Issue 1: `catkin build` fails with "Could not find package"

**Solution:**
```bash
# Update rosdep
rosdep update

# Install missing dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Try building again
catkin build
```

---

### Issue 2: Python scripts not executable

**Solution:**
```bash
cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY
./make_executable.sh

# If script itself not executable:
chmod +x make_executable.sh
./make_executable.sh
```

---

### Issue 3: Submodules not initialized

**Symptoms:** Build fails with missing files in `libs/` or `scripts/` folders

**Solution:**
```bash
cd ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY

# Try initializing submodules first
git submodule update --init --recursive

# If that doesn't work, clone manually:
git clone https://gitlab.com/bradanlane/ros-i2cpwmboard.git ros-i2cpwmboard

mkdir -p spot_micro_motion_cmd/libs
git clone https://github.com/mike4192/spot_micro_kinematics_cpp.git spot_micro_motion_cmd/libs/spot_micro_kinematics_cpp

mkdir -p spot_micro_plot/scripts
git clone https://github.com/mike4192/spot_micro_kinematics_python.git spot_micro_plot/scripts/spot_micro_kinematics_python
```

---

### Issue 4: `roslaunch` command not found

**Solution:**
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Add to bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

### Issue 5: I2C permission denied

**Solution:**
```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Reboot or logout/login
sudo reboot
```

---

### Issue 6: Servo configuration not loading

**Solution:**
```bash
# Check config file exists
ls ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_motion_cmd/config/

# Edit if needed
nano ~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE__SACLAY/spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml

# Rebuild
cd ~/catkin_ws
catkin build spot_micro_motion_cmd
```

---

### Issue 7: CMake version error

**Symptoms:** `CMake 3.0.2 or higher is required`

**Solution:**
```bash
sudo apt install cmake
cmake --version  # Should be 3.16+ on Ubuntu 20.04
```

---

### Issue 8: Python import errors

**Symptoms:** `ModuleNotFoundError: No module named 'matplotlib'`

**Solution:**
```bash
pip3 install --user matplotlib numpy
# Or system-wide:
sudo apt install python3-matplotlib python3-numpy
```

---

## Next Steps

After successful installation:

1. **Servo Calibration:** Follow `docs/servo_calibration.md`
2. **Joystick Setup:** Follow `docs/joystick_control.md`
3. **SLAM Configuration:** Follow `docs/slam_information.md`
4. **Hardware Assembly:** Follow `docs/additional_hardware_description.md`

---

## Quick Reference Commands

```bash
# Start motion control (standalone)
roslaunch spot_micro_motion_cmd motion_cmd.launch run_standalone:=true debug_mode:=true

# Start keyboard control
roslaunch spot_micro_keyboard_command keyboard_command.launch

# Start with RVIZ
roslaunch spot_micro_keyboard_command keyboard_command.launch run_rviz:=true

# View node graph
rqt_graph

# Check ROS topics
rostopic list

# Monitor a topic
rostopic echo /state_command

# Check I2C devices
sudo i2cdetect -y 1

# Rebuild workspace
cd ~/catkin_ws
catkin clean
catkin build
```

---

## Support

For issues or questions:
- GitHub Issues: [Your Repository URL]
- Documentation: Check `docs/` folder
- ROS Answers: https://answers.ros.org/

---

**Installation complete! Your Spot Micro is ready to run.** 🤖🎉