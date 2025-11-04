#!/home/arash/my_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path', 'model/best.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribers and Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detect_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.vis_pub = self.create_publisher(Image, '/yolo/image_marked', 10)

        self.get_logger().info('YOLO Detector Node started...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(source=frame, verbose=False)
        det_msg = Detection2DArray()

        # Fill ROS2 message
        for r in results:
            for box in r.boxes:
                detection = Detection2D()
                detection.bbox.center.x = float(box.xywh[0][0])
                detection.bbox.center.y = float(box.xywh[0][1])
                detection.bbox.size_x = float(box.xywh[0][2])
                detection.bbox.size_y = float(box.xywh[0][3])

                obj = ObjectHypothesisWithPose()
                obj.id = int(box.cls[0])
                obj.score = float(box.conf[0])
                detection.results.append(obj)
                det_msg.detections.append(detection)

        # Publish detections
        self.detect_pub.publish(det_msg)

        # Optional visualization
        annotated = results[0].plot()
        vis_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.vis_pub.publish(vis_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
