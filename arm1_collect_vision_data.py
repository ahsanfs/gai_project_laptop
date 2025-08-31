import time
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from arm1_skills import ARM1_Skills


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/techman_image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            image_frame = self.bridge.imgmsg_to_cv2(msg)
            filename = f"captured_frame_{int(time.time())}_A.jpg"
            cv2.imwrite(filename, image_frame)
            self.get_logger().info(f"Image saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(subtasks, coords):
    """Get initial image frame of work space"""
    rclpy.init()

    # Start image listener node in a background thread
    image_saver_node = ImageSaver()
    executor = MultiThreadedExecutor()
    executor.add_node(image_saver_node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Run robot logic
    arm1 = ARM1_Skills()
    arm1.arm1_home()
    
    for t, task in enumerate(subtasks):
        x, y, z, rx, ry, rz = coords[t]
        if task == 'vision':
            arm1.move_to(x, y, z, rx, ry, rz, velocity=0.5)
            arm1.find()
            time.sleep(1)

    arm1.arm1_home()
    arm1.destroy_node()
    image_saver_node.destroy_node()
    rclpy.shutdown()

#adjust the coords_calibration

# coords_calibration=[
#     [126.93, 204.37, 305.12, 161.02, -0.43, 175.62],
#     [383.16, -241.98, 302.94, 173.11, 29.08, 82.94],
#     [421.98, -229.93, 230.69, -176.64, 14.72, 58.64],
#     [234.93, -439.42, 366.36, 162.64, -10.72, 16.28],
#     [248.77, -372.06, 259.42, 179.21, -19.52, 41.73],
#     [207.66, -193.33, 378.21, 176.78, 13.12, 67.43],
#     [221.81, -151.81, 407.77, 176.47, 29.64, 92.19],
#     [276.14, -372.26, 388.77, 162.71, 16.49, 60.98],
#     [318.52, -356.87, 260.13, 173.21, -3.35, 62.77],
#     [416.79, -271.16, 236.83, -175.46, 30.75, 79.03]
# ]

coords_calibration=[
    [126.93, 204.37, 305.12, 161.02, -0.43, 175.62]
]

if __name__ == '__main__':
    # main(subtasks=['vision', 'vision', 'vision', 'vision', 'vision', 'vision', 'vision', 'vision', 'vision', 'vision'], coords=coords_calibration)
    main(subtasks=['vision'], coords=coords_calibration)


    # x=126.93, y=204.37, z=305.12,
    # rx=161.02, ry=-0.43, rz=175.62,