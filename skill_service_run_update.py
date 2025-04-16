import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetIO, SetPosition
from std_srvs.srv import Trigger
from example_interfaces.srv import SetFloat64Array, SetBool


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        # Services for basic skills
        self.set_io_srv = self.create_service(
            SetIO, 'set_io', self.set_io_callback)
        self.set_position_srv = self.create_service(
            SetPosition, 'set_position', self.set_position_callback)
        self.control_gripper_srv = self.create_service(
            SetBool, 'control_gripper', self.control_gripper_callback)
        self.move_home_srv = self.create_service(
            Trigger, 'move_home', self.move_home_callback)

        self.get_logger().info('RobotControl Node Initialized.')

    # Service callback to set IO
    def set_io_callback(self, request, response):
        self.get_logger().info(f"Setting IO - Module: {request.module}, Type: {request.type}, Pin: {request.pin}, State: {request.state}")
        # Add logic to interact with the robot hardware or simulator here
        response.ok = True  # Assume success
        return response

    # Service callback to move to a specific position
    def set_position_callback(self, request, response):
        self.get_logger().info(
            f"Moving to Position - Motion Type: {request.motion_type}, Positions: {request.pose}, "
            f"Velocity: {request.velocity}, Acc Time: {request.acc_time}, Blend Percentage: {request.blend_percentage}, Fine Goal: {request.fine_goal}"
        )
        # Add logic to interact with the robot hardware or simulator here
        response.ok = True  # Assume success
        return response

    # Service callback to control the gripper
    def control_gripper_callback(self, request, response):
        state = "Closed" if request.data else "Opened"
        self.get_logger().info(f"Gripper State: {state}")
        # Add logic to open/close the gripper
        response.success = True
        return response

    # Service callback to move the robot to the home position
    def move_home_callback(self, request, response):
        home_position = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]  # Example home position
        self.get_logger().info(f"Moving to Home Position: {home_position}")
        # Add logic to move the robot to home
        response.success = True
        response.message = "Moved to Home Position."
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()

    try:
        rclpy.spin(robot_control)
    except KeyboardInterrupt:
        robot_control.get_logger().info('Shutting down RobotControl Node.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
