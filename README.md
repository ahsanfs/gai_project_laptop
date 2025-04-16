# 🤖 ARM1 Controller – ROS 2 Interface for Left-Side Robotic Arm

This repository contains scripts and command-line examples to control the **ARM1**, the left-side robotic arm in a dual-arm setup, using **ROS 2** and **TM Robot drivers**.

The control interface includes robot motion commands as well as IO control for gripper manipulation. It also includes support for remote access via `ngrok`, allowing web-based or remote command forwarding.

---

## 📁 Repository Structure


- tmr_ros2/ └── scripts/ └── script/ └── laptop_ngrok_controller.py


- `laptop_ngrok_controller.py`: Python script for controlling the left-side arm via ngrok tunnel and issuing commands over ROS 2.

---

## 🧠 Features

- Move ARM1 to any target 3D pose using ROS 2 services.
- Turn gripper ON/OFF using IO pin commands.
- Remote access setup using `ngrok` for public endpoint exposure.

---

## 🛠️ Prerequisites

- ROS 2 installed (tested on ROS 2 Foxy)
- TM Robot SDK / `tm_msgs` package installed
- TM Robot hardware with IP (example: `192.168.11.2`)
- Ngrok (for remote tunnel)

---

## 🚀 Command Line Examples

### ✅ Move Robot to Position
```bash
ros2 service call /move_to_position tm_msgs/srv/SetPosition "{
  motion_type: 1,
  pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0},
  velocity: 1.0,
  acc_time: 0.2,
  blend_percentage: 0,
  fine_goal: true
}"
```


✅ Gripper ON
```bash
ros2 service call /set_io tm_msgs/srv/SetIO "{
  module: 1,
  type: 1,
  pin: 0,
  state: 1
}"
```

✅ Gripper OFF
```bash
ros2 service call /set_io tm_msgs/srv/SetIO "{
  module: 1,
  type: 1,
  pin: 0,
  state: 0
}"
```

## 🧪 How to run the full system

Terminal 1 – Start ngrok Tunnel
```bash
ngrok http http://127.0.0.1:5000
# OR, sorry I forgot which one is the correct one:
ngrok http http://127.0.0.3:9000
```
This exposes your local Flask/ROS server to a public URL via ngrok.

Terminal 2 – Launch ROS 2 Driver
```bash
cd ~/gai_ws/
source devel/setup.bash
ros2 run tm_driver tm_driver robot_ip:=192.168.11.2
```
This connects to the TM Robot via its IP and starts the driver.

Terminal 3 – Run Control Script
```bash
cd ~/gai_ws/src/tmr_ros2/scripts/script/
source ~/gai_ws/devel/setup.bash
python3 laptop_ngrok_controller.py
```
This Python script connects to ngrok and sends control commands to the arm via ROS services.

## 📌 Notes
1. Make sure your TM Robot is on the same network as your ROS 2 PC.
2. Make sure you are working on the correct directory