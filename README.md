
# Base ROS2 workspace for F1Tenth Platform

see [wiki](https://github.com/UIUC-Robotics/f1tenth_ws/wiki) for usage and documentation

---

# F1TENTH SLAM MVP Simulation (ROS 2 Humble)

This README describes how to reproduce the simple “fake SLAM” MVP that overlays a simulated LiDAR reconstruction (red) over the ground-truth map (green) using the **f1tenth_ws** workspace.

---

## 1. Create workspace & clone repos

```bash
mkdir -p ~/f1tenth_project_ws/src
cd ~/f1tenth_project_ws/src

git clone git@github.com:sriramcu/f1tenth_ws.git

git clone https://github.com/UIUC-Robotics/f1tenth_simulator.git
```

##  2. Folder Structure

Your workspace should look like this:

```
~/f1tenth_project_ws/
├── src/
│   ├── f1tenth_ws/                
│   ├── f1tenth_simulator/         
```

---

##  3. Install Python Dependencies


```bash
python3 -m pip install --user "gym==0.21.0" "pyglet==1.5.27" "numba==0.57.1" "llvmlite==0.40.1" "numpy==1.24.4" matplotlib pyyaml
```

If 3D import warnings for Matplotlib appear, ignore them, they’re harmless.

---


##  4. Build and Source

From the root of your workspace:

```bash
cd ~/f1tenth_project_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select f1tenth_control f1tenth_gym_ros
source install/setup.bash
```

---

##  5. Run the Simulation

## 5.1 Simple Way:

Open **three terminals** (all sourced with `source install/setup.bash`).

### Terminal A – Run Simulator

```bash
ros2 run f1tenth_gym_ros gym_bridge --ros-args \
  -p config:=$(ros2 pkg prefix f1tenth_gym_ros)/share/f1tenth_gym_ros/config/sim.yaml
```

You’ll see `/ego_racecar/odom` and `/ego_racecar/scan` topics appear:

```bash
ros2 topic list | grep ego_racecar
```

###  Terminal B - Drive the Vehicle in a Straight Line

```bash
ros2 topic pub -r 10 /ego_racecar/drive ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 1.0, steering_angle: 0.0}}"
```

Let it run for ~20 seconds.

### Terminal C – Run Fake SLAM

```bash
export MPLBACKEND=Agg
ros2 run f1tenth_control mvp_fake_slam --ros-args \
  -p gt_map_yaml:=$(ros2 pkg prefix f1tenth_gym_ros)/share/f1tenth_gym_ros/maps/levine.yaml \
  -p odom_topic:=/ego_racecar/odom \
  -p scan_topic:=/ego_racecar/scan \
  -p lap_seconds:=25.0
```

## 5.2 Manual Way:

Open **three terminals** (all sourced with `source install/setup.bash`).

### Terminal A – Run Simulator

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

###  Terminal B - Drive the Vehicle Manually via keyboard inputs

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```  

Controls:

i → forward (straight)

, → backward (straight)

j → rotate left in place (no forward speed)

l → rotate right in place (no forward speed)

u → forward + left

o → forward + right

m → backward + left

. → backward + right

k → stop



### Terminal C – Run Fake SLAM

```bash
export MPLBACKEND=Agg
ros2 run f1tenth_control mvp_fake_slam --ros-args \
  -p gt_map_yaml:=$(ros2 pkg prefix f1tenth_gym_ros)/share/f1tenth_gym_ros/maps/levine.yaml \
  -p odom_topic:=/ego_racecar/odom \
  -p scan_topic:=/ego_racecar/scan \
  -p lap_seconds:=100.0
```
---

##  7. Output

After the fake SLAM node finishes, a PNG is saved to:

```
~/mvp_fake_slam_overlay.png
```

This image shows:

* **Green** — ground-truth track (from the YAML)
* **Red** — accumulated LiDAR “SLAM” points




For longer runs, just increase `lap_seconds`.

