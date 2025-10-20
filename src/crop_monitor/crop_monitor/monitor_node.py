import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from visualization_msgs.msg import Marker, MarkerArray

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/monitor/markers', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (20, 50, 50), (40, 255, 255))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        markers = MarkerArray()
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.id = i
            marker.type = Marker.CUBE
            marker.pose.position.x = (x + w / 2) * 0.001
            marker.pose.position.y = (y + h / 2) * 0.001
            marker.pose.position.z = 0.1
            marker.scale.x = w * 0.001
            marker.scale.y = h * 0.001
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.a = 0.6
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
