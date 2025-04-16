import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetIO, SetPosition
#ros2 run tm_driver tm_driver robot_ip:=192.168.10.2
class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.cli_io = self.create_client(SetIO, 'tm_driver/set_io')
        self.cli_pos = self.create_client(SetPosition, 'tm_driver/set_position')
        
        while not self.cli_io.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IO service...')
        while not self.cli_pos.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Position service...')

    def move_to_position(self, position, motion_type=1, velocity=1.0, acc_time=0.2, blend_percentage=0, fine_goal=True):
        req = SetPosition.Request()
        req.motion_type = motion_type
        req.pose = position
        req.velocity = velocity
        req.acc_time = acc_time
        req.blend_percentage = blend_percentage
        req.fine_goal = fine_goal
        self.cli_pos.call_async(req)

    def control_gripper(self, state):
        req = SetIO.Request()
        req.module = 1  # End-effector
        req.type = 1    # Digital output
        req.pin = 0     # Gripper control pin
        req.state = state
        self.cli_io.call_async(req)

    def pick_up(self, position):
        self.move_to_position(position)
        self.control_gripper(1.0)  # Close gripper

    def put_down(self, position):
        self.move_to_position(position)
        self.control_gripper(0.0)  # Open gripper

    def lock(self):
        req = SetIO.Request()
        req.module = 1
        req.type = 1
        req.pin = 1     # Lock control pin
        req.state = 1.0 # Engage lock
        self.cli_io.call_async(req)

    def insert(self, insertion_position):
        req = SetPosition.Request()
        req.motion_type = 2  # Linear movement
        req.pose = insertion_position
        req.velocity = 0.1
        req.acc_time = 0.2
        req.blend_percentage = 0
        req.fine_goal = True
        self.cli_pos.call_async(req)

    def move_forward(self, distance, current_position):
        new_position = list(current_position)
        new_position[0] += distance
        self.move_to_position(new_position)

    def move_backward(self, distance, current_position):
        new_position = list(current_position)
        new_position[0] -= distance
        self.move_to_position(new_position)

    def go_home(self):
        home_position = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]
        self.move_to_position(home_position)

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()

    # Define positions for each task
    pick_up_position = [0.5, 0.0, 0.2, 0.0, 0.0, 0.0]
    put_down_position = [0.6, 0.0, 0.2, 0.0, 0.0, 0.0]
    insertion_position = [0.5, 0.1, 0.2, 0.0, 0.0, 0.0]
    current_position = pick_up_position  # Example initial position

    # Perform each task
    robot_control.pick_up(pick_up_position)
    robot_control.put_down(put_down_position)
    robot_control.lock()
    robot_control.insert(insertion_position)
    robot_control.move_forward(0.1, current_position)
    robot_control.move_backward(0.1, current_position)
    robot_control.go_home()

    rclpy.spin(robot_control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

