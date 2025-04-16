1. Services
    /tm_driver/set_io: Control the end-effector or other I/O functions.
    /tm_driver/set_position: Set the robot's position with parameters like motion type, position, velocity, etc.

2. Topics
    /tm_driver/joint_states: Subscribe to the robot’s joint states to monitor its current position.
    /tm_driver/io_states: Subscribe to the I/O state updates.


Pick up position

Move to
ros2 service call /tm_driver/set_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: [0.5, 0.0, 0.2, 0.0, 0.0, 0.0], velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

Gripper
ros2 service call /tm_driver/set_io tm_msgs/srv/SetIO "{module: 1, type: 1, pin: 0, state: 1.0}"

Put down

Move to
ros2 service call /tm_driver/set_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: [0.5, 0.0, 0.2, 0.0, 0.0, 0.0], velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

Release gripper
ros2 service call /tm_driver/set_io tm_msgs/srv/SetIO "{module: 1, type: 1, pin: 0, state: 0.0}"

# Move forward
ros2 service call /tm_driver/set_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: [current_x + 0.1, 0.0, 0.2, 0.0, 0.0, 0.0], velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /tm_driver/set_position tm_msgs/srv/SetPosition "{motion_type: <1 for PTP>, pose: [x, y, z, roll, pitch, yaw], velocity: <velocity>, acc_time: <acc_time>, blend_percentage: <blend>, fine_goal: <true or false>}"

ros2 service call /tm_driver/set_io tm_msgs/srv/SetIO "{module: 1, type: 1, pin: 0, state: <1.0 for close, 0.0 for open>}"



New Instruction

ros2 service call /move_to_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # true to close, false to open

ros2 service call /insert tm_msgs/srv/SetPosition "{motion_type: 2, pose: {x: 0.5, y: 0.1, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 0.1, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /move_forward example_interfaces/srv/SetFloat64 "{data: 0.1}"  # Move forward by 0.1 units

ros2 service call /go_home tm_msgs/srv/SetPosition "{pose: {x: 0.0, y: -0.5, z: 0.5, r: 0.0, p: 0.0, yw: 0.0}}"


#run ARM1
ros2 run tm_driver tm_driver robot_ip:=192.168.10.2



example task plan:
case 1

# pickup arm1 red_wire table
pickup arm1 red_wire table

call "pick up" function:

move to + control gripper

ros2 service call /move_to_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # true to close, false to open

# putdown arm1 red_wire power_supply_5

ros2 service call /move_to_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # true to close, false to open


# gohome


# lock arm2 red_wirepower_supply_5

still not finish



case 2
# pickup arm1 red_wire table
pickup arm1 red_wire table

call "pick up" function:

move to + control gripper

ros2 service call /move_to_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # true to close, false to open

# lock arm2 red_wirepower_supply_5

still not finish


# insert arm1 red_wire power_supply_5

ros2 service call /move_to_position tm_msgs/srv/SetPosition "{motion_type: 1, pose: {x: 0.5, y: 0.0, z: 0.2, r: 0.0, p: 0.0, yw: 0.0}, velocity: 1.0, acc_time: 0.2, blend_percentage: 0, fine_goal: true}"

ros2 service call /control_gripper std_srvs/srv/SetBool "{data: true}"  # true to close, false to open



# gohome


##How to run

Tab1
ngrok http http://127.0.0.1:5000
or
ngrok http http://127.0.0.3:9000

Tab2
cd ~/gai_ws/
source devel/setup.bash
ros2 run tm_driver tm_driver robot_ip:=192.168.11.2

Tab3
cd ~/gai_ws/src/tmr_ros2/scripts/script/
source ~/gai_ws/devel/setup.bash
python3 laptop_ngrok_controller.py
