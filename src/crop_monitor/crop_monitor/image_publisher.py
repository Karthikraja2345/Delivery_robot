import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()
        img_path = '/home/karthik/early.jpeg'  # Adjust if needed
        self.cv_img = cv2.imread(img_path)
        if self.cv_img is None:
            self.get_logger().error(f'Cannot load image at {img_path}')
            exit(1)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = self.br.cv2_to_imgmsg(self.cv_img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published static image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
