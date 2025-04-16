import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions, SetIO, SendScript
import math
from geometry_msgs.msg import PoseStamped
import struct
import socket


class ARM1_Skills(Node):

    def __init__(self):
        super().__init__('ARM1_Skills')
        self.set_positions_client = self.create_client(SetPositions, '/arm1/set_positions')
        self.set_io_client = self.create_client(SetIO, '/arm1/set_io')
        self.send_script_client = self.create_client(SendScript, '/arm1/send_script')

        while not self.set_positions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arm1/set_positions service...')
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arm1/set_io service...')
        while not self.send_script_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arm1/sends_script service...')

        self.tool_pose = None
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/arm1/tool_pose',
            self.pose_callback,
            10  # QoS profile for reliability
        )

    def pose_callback(self, msg):
        """Callback to update tool_pose."""
        self.tool_pose = msg.pose  # Extract the pose
        # self.get_logger().info(f"Updated tool_pose: {self.tool_pose}")

    def degrees_to_radians(self, degrees):
        """Convert degrees to radians."""
        return degrees * (math.pi / 180)

    def milimeters_to_meters(self, units):
        return units / 1000

    def call_set_positions(self, x, y, z, rx, ry, rz, velocity=0.5, acc_time=0.5, blend_percentage=0, fine_goal=True):
        """Send a command to move the robot to a position with specified parameters."""
        req = SetPositions.Request()
        req.motion_type = 4  # Always use motion type 2
        req.positions = [
            self.milimeters_to_meters(x),
            self.milimeters_to_meters(y),
            self.milimeters_to_meters(z),
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
            self.get_logger().error('Failed to call /arm1/set_positions')

    def call_set_io(self, module, type_, pin, state):
        """Send a command to control the robot IO (e.g., gripper)."""
        req = SetIO.Request()
        req.module = module
        req.type = type_
        req.pin = pin
        req.state = float(state)

        future = self.set_io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Set IO result: {future.result()}')
        else:
            self.get_logger().error('Failed to call /arm1/set_io')

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



    def pick_up(self, x, y, z, rx, ry, rz, velocity=1.0):
        """Perform the pick-up operation with specified position and speed."""
        self.call_set_positions(x, y, z, rx, ry, rz, velocity)
        self.call_set_io(module=1, type_=1, pin=0, state=1.0)  # Lock gripper
        self.get_logger().info(f'Picking up object at [{x}, {y}, {z}] with rotation [{rx}, {ry}, {rz}] and velocity {velocity}.')

    def put_down(self, x, y, z, rx, ry, rz, velocity=1.0):
        """Perform the put-down operation with specified position and speed."""
        self.call_set_positions(x, y, z, rx, ry, rz, velocity)
        self.call_set_io(module=1, type_=1, pin=0, state=0.0)  # Unlock gripper
        self.get_logger().info(f'Putting down object at [{x}, {y}, {z}] with rotation [{rx}, {ry}, {rz}] and velocity {velocity}.')

    def move_to(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot to a specified position."""
        self.call_set_positions(x, y, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving to [{x}, {y}, {z}] with rotation [{rx}, {ry}, {rz}] and velocity {velocity}.')

    def arm1_home(self, velocity=1.0):
        """Move the robot to the home position."""
        self.call_set_positions(
            x=126.93, y=204.37, z=305.12,
            rx=161.02, ry=-0.43, rz=175.62,
            velocity=velocity
        )
        print(f"ARM 1 moving to {328, 57, 171}")
        self.get_logger().info('Moving to home position.')

    def insert(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Perform the insert operation."""
        pos_1_y = y - 0.08 # Adjust y position slightly away from target (to manipulate insertion trajectory)
        self.call_set_positions(x, pos_1_y, z, rx, ry, rz, velocity=2.0)
        self.call_set_positions(x, y, z, rx, ry, rz, velocity=0.5)
        self.get_logger().info(f'Inserting object at [{x}, {y}, {z}] with rotation [{rx}, {ry}, {rz}] and velocity {velocity}.')

    def move_forward(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot forward."""
        self.call_set_positions(x, y + 50, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving forward from [{x}, {y}, {z}] with velocity {velocity}.')

    def move_back(self, x, y, z, rx, ry, rz, velocity=0.5):
        """Move the robot backward."""
        self.call_set_positions(x, y - 50, z, rx, ry, rz, velocity)
        self.get_logger().info(f'Moving back from [{x}, {y}, {z}] with velocity {velocity}.')

    def find(self):
        """Find an object (dummy implementation)."""
        self.get_logger().info('Finding object...')

    def gripper(self):
        id_ = "Hucenrotia"
        script = "ScriptExit()"
        self.call_send_script(id_, script)

        return True

    def is_pose_ready(self, pose, target_position, tolerance=0.01):
        """Check if the tool pose is within tolerance of the target position."""
        dx = abs(pose.position.x - target_position[0])
        dy = abs(pose.position.y - target_position[1])
        dz = abs(pose.position.z - target_position[2])

        return dx <= tolerance and dy <= tolerance and dz <= tolerance




# def main(args=None):
#     rclpy.init(args=args)
#     robot_skills = ARM1_Skills()

#     # Predefined function calls with default or specified velocities
#     robot_skills.home()
#     robot_skills.move_to(0.405, 0.045, -0.002, 173.33, 0.16, 93.13, velocity=1.0) # Move to grasping position
#     ## PICKUP FUNTION ##
#     robot_skills.move_to(0.405, 0.045, 0.009, 173.33, 0.16, 93.13, velocity=1.0) # Lift from buffer
#     robot_skills.insert(0.141,  0.339, -0.008, 166.37, 2.67, 177.51, velocity=0.5)



#     robot_skills.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


