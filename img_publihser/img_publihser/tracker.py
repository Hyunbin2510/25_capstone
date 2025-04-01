import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

CONFIDENCE_THRESHOLD = 0.7
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

CLASS_FILE = '~/capstone_ws/yolo_class.txt' # 이거 왜 만듬? 안쓸거면면
if os.path.exists(CLASS_FILE):
    with open(CLASS_FILE, 'r') as f:
        class_list = f.read().splitlines()
else:
    class_list = []

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('tracker')
        self.subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Point, 'person_tracking', 10)
        self.opencvpub = self.create_publisher(Image, 'opencv_image', 10)
        self.bridge = CvBridge()
        
        model_path = '~/capstone_ws/yolo11m.pt'
        self.yolo_model = YOLO(model_path)

        self.tracker = DeepSort(max_age=20)

    def image_callback(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'이미지 변환 오류: {e}')
                return

            image_height, image_width = frame.shape[:2]

            results = self.yolo_model.predict(source=[frame], save=False)[0]
            detection_results = []

            # 탐지 결과 처리: detection.boxes.data.tolist()의 각 data는
            # [xmin, ymin, xmax, ymax, confidence, class_id] 형식입니다.
            for data in results.boxes.data.tolist():
                confidence = float(data[4])
                if confidence < CONFIDENCE_THRESHOLD:
                    continue

                xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
                label = int(data[5])
                bbox = [xmin, ymin, xmax - xmin, ymax - ymin]
                detection_results.append([bbox, confidence, label])
                
            tracks = self.tracker.update_tracks(detection_results, frame=frame)
            for track in tracks:
                print(track.track_id)
                if not track.is_confirmed():
                    continue
                if track.track_id != 1:
                    continue

                ltrb = track.to_ltrb()
                xmin, ymin, xmax, ymax = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])
                center_x = (xmin + xmax) / 2.0
                bbox_width = xmax - xmin
                norm_center_x = center_x / image_width
                norm_bbox_width = bbox_width / image_width

                tracking_msg = Point()
                tracking_msg.x = norm_center_x   
                tracking_msg.y = 0.0             
                tracking_msg.z = norm_bbox_width 

                self.publisher.publish(tracking_msg)
                self.get_logger().info(f'Publish: TrackID=1, norm_center_x={norm_center_x:.2f}, norm_bbox_width={norm_bbox_width:.2f}')

                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.putText(frame, f'ID: {track.track_id}', (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
                print(xmin, ymin, xmax, ymax)
            processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.opencvpub.publish(processed_msg)
            self.get_logger().info("Processed image published.")


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

