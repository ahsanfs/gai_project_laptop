import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions, SetIO
import math
import subprocess
import threading
from geometry_msgs.msg import PoseStamped
import time
import json

class ARM2_Skills(Node):

    def __init__(self):
        super().__init__('ARM2_Skills')
        self.set_positions_client = self.create_client(SetPositions, '/set_positions')
        self.set_io_client = self.create_client(SetIO, '/set_io')

        while not self.set_positions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_positions service...')
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_io service...')

        # Subscribe to the /tool_pose topic
        self.tool_pose = None
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/tool_pose',
            self.pose_callback,
            10  # QoS profile for reliability
        )

    def pose_callback(self, msg):
        """Callback to update tool_pose."""
        self.tool_pose = msg.pose  # Extract the pose
        # self.get_logger().info(f"Updated tool_pose: {self.tool_pose}")

    def millimeters_to_meters(self, units):
        "Convert from mm to m"
        return units /1000

    def degrees_to_radians(self, degrees):
        """Convert degrees to radians."""
        return degrees * (math.pi / 180)

    def call_set_positions(self, x, y, z, rx, ry, rz, velocity=0.5, acc_time=0.5, blend_percentage=0, fine_goal=True):
        """Send a command to move the robot to a position with specified parameters."""
        req = SetPositions.Request()
        req.motion_type = 4  # Always use motion type 2
        req.positions = [
            self.millimeters_to_meters(x),
            self.millimeters_to_meters(y),
            self.millimeters_to_meters(z),
            self.degrees_to_radians(rx),
            self.degrees_to_radians(ry),
            self.degrees_to_radians(rz),
        ]
        req.velocity = velocity
        req.acc_time = acc_time
        req.blend_percentage = blend_percentage
        req.fine_goal = fine_goal

        future = self.set_positions_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Set positions result: {future.result()}')
        else:
            self.get_logger().error('Failed to call /set_positions')

    def arm2_home(self, velocity=1.0):
        """Move the robot to the home position."""
        self.call_set_positions(
            x=140.51, y=-214.04, z=253.07,
            rx=150.26, ry=-0.02, rz=-0.59,
            velocity=velocity
        )
        print(f"ARM 2 moving to {131.62, -48.95, 571.27}")
        self.get_logger().info('Moving to home position.')

    def move_to(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot to a specified position."""
        self.call_set_positions(x, y, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving to [{x}, {y}, {z}] with rotation [{rx}, {ry}, {rz}] and velocity {velocity}.')

    def move_forward(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot forward."""
        self.call_set_positions(x, y + 50, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving forward from [{x}, {y}, {z}] with velocity {velocity}.')

    def move_back(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot backward."""
        self.call_set_positions(x, y - 50, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving back from [{x}, {y}, {z}] with velocity {velocity}.')

    def find(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Find an object (dummy implementation)."""

        self.get_logger().info('Finding object...')
        self.call_set_positions(x, y, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving back from [{x}, {y}, {z}] with velocity {velocity}.')

    def cam():
        id_ = "Hucenrotia_2"
        script = "ScriptExit()"
        self.call_send_script(id_, script)

        return True
        
# def call_lock_screw():
#     subprocess.run(['python3', 'screwdriver_connect.py'])

    def call_lock_screw(self):
        """Trigger the screwdriver script and wait for it to complete."""
        screwdriver_process = subprocess.Popen(['python3', 'screwdriver_connect.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        screwdriver_process.wait()  # Wait for the subprocess to finish
        if screwdriver_process.returncode == 0:
            print("Screwdriver task completed successfully.")
            return True
        else:
            print(f"Screwdriver task failed with return code {screwdriver_process.returncode}.")
            return False

    def is_pose_ready(self, pose, target_position, tolerance=0.01):
        """Check if the tool pose is within tolerance of the target position."""
        dx = abs(pose.position.x - target_position[0])
        dy = abs(pose.position.y - target_position[1])
        dz = abs(pose.position.z - target_position[2])

        return dx <= tolerance and dy <= tolerance and dz <= tolerance

# def main(args=None):

#     rclpy.init(args=args)
#     robot_skills = ARM2_Skills()
#     # target_position = [0.154, -0.309, -0.064]  # Define target coordinates
#     json_file = "target_coords.json"
#     with open(json_file, 'r') as file:
#         coordinates = json.load(file)
#     target_position = coordinates[0]
#     x, y, z = target_position['x'], target_position['y'], target_position['z']
#     tolerance = 0.0001  # Define allowable tolerance

#     robot_skills.home()
#     robot_skills.move_to(x, y, z+0.248, 177.54, 2.48, 0.94, velocity=3.5)
#     robot_skills.move_to(x, y, z+0.284, 177.97, -2.65, -90.63, velocity=3.5)
#     robot_skills.move_to(x, y, z+0.144, 178.97, -2.65, -90.63, velocity=3.5)
#     robot_skills.move_to(x, y, z, -179.08, -2.14, -87.03, velocity=0.5)

#     robot_skills.get_logger().info("Waiting for tool_pose to reach target position...")
#     while rclpy.ok():
#         rclpy.spin_once(robot_skills)  # Process callback queue
#         if robot_skills.tool_pose and is_pose_ready(robot_skills.tool_pose, [x, y, z], tolerance):
#             robot_skills.get_logger().info("Tool pose reached target position. Executing lock_screw.")
#             if call_lock_screw():  # Run the screwdriver task and wait for it to complete
#                 time.sleep(3)
#                 robot_skills.get_logger().info("Screwdriver task done. Moving to the next position.")
#                 robot_skills.home()
#             else:
#                 robot_skills.get_logger().error("Screwdriver task failed. Halting further operations.")
#             break

#     robot_skills.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()