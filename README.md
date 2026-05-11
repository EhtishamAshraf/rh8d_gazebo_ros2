# RH8D Hand Simulation (ROS 2 + Gazebo Sim)

This project provides a ROS 2 and Gazebo-based simulation setup for the RH8D robotic hand. It includes:
- A fully actuated RH8D simulation.
- An underactuated tendon-approximated RH8D simulation.
- ros2_control integration.
- Gazebo Sim integration.
- Position control.
- Hybrid position-effort control.
- The simulation includes manipulation of two objects: a Coke can and a ball.

Tested on:
- ROS 2 Humble
- Gazebo Sim 6.17.1 / Ignition Gazebo Fortress

![Coke Can Simulation](https://github.com/EhtishamAshraf/rh8d_gazebo_ros2/blob/8f57d3f1642807379a44ccdaa9483567b4fa4144/assests/1-Images/4-Coke_Can_3.png)

---

## Project Overview

This project simulates the **RH8D tendon-driven robotic hand** in Gazebo using ROS 2 and `ros2_control`.

The real RH8D hand is tendon-driven. However, Gazebo Sim does not support native tendon actuation in the same way as MuJoCo. Therefore, tendon-like behavior is approximated using **mimic joints** for the underactuated version.

The project contains two simulation configurations, each available in a separate branch:

- **Underactuated RH8D hand**
  - Tendon behavior is approximated using mimic joints.
  - Wrist joints are controlled directly while finger joints follow through mimic relationships.

- **Fully actuated RH8D hand**
  - Each joint is controlled independently.
  - No custom control code was written for this version.
  - Joint motion can be commanded using `rqt_joint_trajectory_controller`.

Demo video (click on the below image):
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_gazebo_ros2/blob/8f57d3f1642807379a44ccdaa9483567b4fa4144/assests/1-Images/4-Coke_Can_4.png)](https://youtu.be/vms1Rzq0_Kk)

---

## System Requirements

The system must use:

- ROS 2 Humble
- Gazebo Fortress / Gazebo Sim 
- ros2_control framework
- gz_ros2_control plugin

Install the Gazebo ROS 2 control plugin:

```bash
sudo apt update
sudo apt install ros-humble-gz-ros2-control
```

This installs the `libgz_ros2_control` package, which provides the Gazebo Sim plugin required to connect simulated Gazebo joints with the ROS 2 `ros2_control` controller framework.

Reference: [gz_ros2_control GitHub](https://github.com/ros-controls/gz_ros2_control)

---

## Build and Launch Instructions

Go to the workspace. Build the packages:

```bash
colcon build
```
Source the workspace:

```bash
source install/setup.bash
```

### Underactuated RH8D Simulation

For the Hybrid position-effort underactuated setup:
```bash
ros2 launch rh8d_control rh8dL_ros2_control.launch.py
```
```bash
ros2 run rh8d_control rh8dL_object_pick
```

For the position-only underactuated setup:
```bash
ros2 launch rh8d_control rh8dL_ros2_control_pos.launch.py
```
```bash
ros2 run rh8d_control rh8dL_object_pick_pos
```
Demo video (click on the below image):
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_gazebo_ros2/blob/8f57d3f1642807379a44ccdaa9483567b4fa4144/assests/1-Images/5-Ball_1.png)](https://youtu.be/uuPhGCBPrJ4)

Hand Gesture Controller:
```bash
ros2 run rh8d_control rh8dL_gestures
```
Demo video (click on the below image):
[![Demo Video](https://github.com/EhtishamAshraf/rh8d_gazebo_ros2/blob/8f57d3f1642807379a44ccdaa9483567b4fa4144/assests/1-Images/2-VictorySign.png)](https://youtu.be/kn62hzurok4)

### Fully Actuated RH8D Simulation

```bash
ros2 launch rh8d_control rh8dL_ros2_control.launch.py
```
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover -t -v
```
---

## Controllers Verification Procedure

After launching the simulation, verify that the full control stack is working. The controllers should be in the `active` state.

Check active controllers:
```bash
ros2 control list_controllers
```

The following command sends a position trajectory to the joint trajectory controller:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'forearm:1--base:1',
    'palm_axis:1--forearm:1',
    'palmL:1--palm_axis:1',
    'Thumb_axis--palmL:1',
    'Thumb_Methacarpal--Thumb_axis',
    'Index_Proximal--palmL:1',
    'Middle_Proximal--palmL:1',
    'Ring_Proximal--palmL:1'
  ],
  points: [{
    positions: [
      0.0, 0.0, 0.0, 1.57,
      1.57, 1.57, 1.57, 1.57
    ],
    time_from_start: {sec: 1}
  }]
}"
```

The underactuated setup can also use a hybrid control approach:

- Palm/wrist joints are controlled using position commands.
- Finger joints are controlled using effort commands.

```bash
ros2 topic pub --once /palm_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'forearm:1--base:1',
    'palm_axis:1--forearm:1',
    'palmL:1--palm_axis:1',
    'Thumb_axis--palmL:1'
  ],
  points: [{
    positions: [-1.57, 0.0, 0.0, 1.57],
    time_from_start: {sec: 1}
  }]
}"
```

```bash
ros2 topic pub --once /finger_effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1]"
```
---

## Downloading Gazebo Models and Spawning Objects into the Gazebo World

Extra Gazebo models can be downloaded from [Ignition Fuel](https://app.gazebosim.org/fuel/models).

Objects can be spawned into the running Gazebo world using the Ignition service interface:

```bash
ign service -s /world/empty/create \
--reqtype ignition.msgs.EntityFactory \
--reptype ignition.msgs.Boolean \
--timeout 3000 \
--req 'sdf_filename: "/home/ehtisham/.ignition/fuel/fuel.gazebosim.org/openrobotics/models/coke/6/model.sdf"
name: "can"
pose {
  position { x: 0.6 y: 0.0 z: 0.0 }
  orientation { w: 1.0 }
}'
```

After spawning the required models, save the Gazebo world so the objects remain available in future launches.

---

## Common Issues and Fixes

### Robot spawns but meshes are missing

Cause:

- Meshes are not installed into the workspace install directory.
- `GZ_SIM_RESOURCE_PATH` is not configured correctly.

Fix:

```bash
colcon build
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/your_workspace/install/your_package/share
```
---

### Controllers are not active

Check controllers:

```bash
ros2 control list_controllers
```

If controllers are not active, check:

- The controller YAML file.
- Joint names in the URDF.
- Whether `libgz_ros2_control-system.so` is loaded correctly.
- Whether the controller manager started successfully.
---

### Gazebo plugin does not load

Check plugin installation:

```bash
ls /opt/ros/humble/lib | grep libgz_ros2_control
```

Check that the URDF contains:

```xml
<plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
</plugin>
```
---
