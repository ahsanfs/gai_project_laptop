import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions, SetIO, SendScript
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import threading
from geometry_msgs.msg import PoseStamped
import time
import json
import cv2
import requests
import base64
url_update_screw_driver_capture = "http://localhost:6000/screw_driver_capture"


class ARM2_Skills(Node):

    def __init__(self):
        super().__init__('ARM2_Skills')
        self.set_positions_client = self.create_client(SetPositions, '/set_positions')
        self.set_io_client = self.create_client(SetIO, '/set_io')
        self.send_script_client = self.create_client(SendScript, '/send_script')

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

        self.image_frame = None
        self.subscription = self.create_subscription(
            Image,
            '/techman_image',  # Replace with your image topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.image_frame = self.bridge.imgmsg_to_cv2(msg)

    #     self.scalled_frame = cv2.resize(self.image_frame, (480, 320))
    #     # Encode the frame into bytes (JPEG format)
    #     _, self.img_encoded = cv2.imencode('.jpg', self.scalled_frame)

    #     # Convert to bytes for sending
    #     self.base64_image = base64.b64encode(self.img_encoded).decode('utf-8')  # <-- decode here
    #     self.response = requests.post(url_update_screw_driver_capture, json={"image": self.base64_image})
    #     print(f"Server response: {self.response.text}")
    
    # def generate_frames(self,image_frame):
    #     print("xhxh")

    #     _, buffer = cv2.imencode('.jpg', image_frame)
    #     frame_data = buffer.tobytes()
    #     yield (b'--frame\r\n'
    #            b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

    def pose_callback(self, msg):
        """Callback to update tool_pose."""
        self.tool_pose = msg.pose  # Extract the pose

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

    def call_send_script(self, id: str, script: str):
        # Create a request for the SendScript service
        request = SendScript.Request()
        request.id = id
        request.script = script

        # Send the request and wait for the response
        future = self.send_script_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info(f"Script '{id}' sent successfully.")
            else:
                self.get_logger().error(f"Failed to send script '{id}'. Script correctness: {future.result().ok}")
        else:
            self.get_logger().error(f"Service call failed for script '{id}'.")

    def arm2_home(self, velocity=1.0):
        """Move the robot to the home position."""
        self.call_set_positions(
            x=226.55, y=-144.01, z=334.04,
            rx=158.44, ry=-5.25, rz=9.98,
            velocity=velocity
        )
        # print(f"ARM 2 moving to {131.62, -48.95, 571.27}")
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

    def find(self):
        id_ = "L"
        script = "ScriptExit()"
        self.call_send_script(id_, script)

        self.get_logger().info("Waiting for image frame...")
        timeout = time.time() + 1000 
        while self.image_frame is None and time.time() < timeout:
            rclpy.spin_once(self)

        if self.image_frame is not None:
            self.get_logger().info("Image frame successfully received.")
            self.image_frame = None
            return True
        else:
            self.get_logger().error("Timeout: No image frame received.")
            return False



    def call_lock_screw(self):
        """Trigger the screwdriver script and wait for it to complete."""
        print("Locking")
        screwdriver_process = subprocess.Popen(['python3', '/home/gai-hucenrotia/gai_ws/src/scripts/Visual_Programming_Flowchart_v3.1/vola/src/Python_Backend/screwdriver_connect.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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