import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/techman_image',  # Replace with your image topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_saved = False

    def image_callback(self, msg):
        if not self.image_saved:  # Save only one image
            try:
                # Convert ROS2 Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg)
                
                # Save the OpenCV image as a JPG file
                file_path = 'saved_image.jpg'
                cv2.imwrite(file_path, cv_image)
                self.get_logger().info(f"Image saved to {file_path}")
                self.image_saved = True  # Prevent saving multiple images
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
