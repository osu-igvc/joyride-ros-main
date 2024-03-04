# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from vision_msgs.msg import Detection2DArray

# import cv2
# from cv_bridge import CvBridge
# import torch, requests

# # Object classifier that uses YOLO to detect traffic signs and other objects
# # Runs as fast as new images are received, as YOLO is faster than our cameras. Runs on GPU.

# # - Outputs an image showing classifications
# # - Outputs a 2D detection array with bounding boxes, classification IDs, confidence, etc.

# # Much of the code is sourced from:
# # https://github.com/open-shade/yolov5/blob/main/yolov5/interface.py

# class TrafficSignDetector(Node):
#     def __init__(self):
#         super().__init__('traffic_sign_detector')

#         # ROS Parameters
#         self.image_source_topic = self.declare_parameter('image_source', '/sensors/cameras/lane/image_raw').get_parameter_value().string_value
#         self.image_output_topic = self.declare_parameter('image_output', '/perception/signs_detected').get_parameter_value().string_value
#         self.detections_output_topic = self.declare_parameter('detection_output','/perception/sign_predictions').get_parameter_value().string_value


#         self.weights = self.declare_parameter('weights_path', 'best.pt')
        
#         # YOLO Model using PyTorch Hub

#         # - absolute path to yolov5
#         #self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/yolov5').get_parameter_value().string_value
#         self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/third_party/yolov5').get_parameter_value().string_value

#         # - absolute path to best.pt
#         self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt').get_parameter_value().string_value
#         self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False)

#         # ROS pubs
#         self.detection_pub = self.create_publisher(Detection2DArray, self.detections_output_topic, 10)
#         self.classification_pub = self.create_publisher(Image, self.image_output_topic, 10)

#         self.bridge = CvBridge()
#         # ROS subs
#         self.image_sub = self.create_subscription(Image, self.image_source_topic, self.new_raw_image_cb, 1)
#         print("before load")
#         self.get_logger().info('Traffic Detector loaded')
#         print("after loaded")
    
#     def new_raw_image_cb(self, msg:Image):
#         print("begin")
#         cvImg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#         results = self.model(cvImg)
#         print("before results")
#         print(results)
#         print("running 1")
#         results.render()
#         processed_img = self.bridge.cv2_to_imgmsg(results.ims[0], encoding='rgb8')
#         # print(processed_img)
#         self.classification_pub.publish(processed_img)
#         print("Running 2")

#         #results.print()
#         # Run classifier
#         # Annotate image
#         # publish annotated
#         # Build detection array
#         # publish detection array


# def main():
#     rclpy.init()
#     tDetector = TrafficSignDetector()
#     rclpy.spin(tDetector)

#     tDetector.destroy_node()
#     rclpy.shutdown()



# if __name__ == '__main__':
#     main()
import torch
import requests
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis, BoundingBox2D, Point2D, Pose2D
from geometry_msgs.msg import PoseWithCovariance, Point

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        self.declare_parameter('pub_image', True)
        self.declare_parameter('pub_json', False)
        self.declare_parameter('pub_boxes', True)
        self.declare_parameter('pub_proc_image', True) # ADDED
        self.declare_parameter('weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt')
        self.subscription = self.create_subscription(
            Image,
            # '/sensors/cameras/lane/image_raw',
            '/sensors/cameras/center/image',
            self.listener_callback,
            100 
        ) # may want to mess with the queue size of 1

        self.image_publisher = self.create_publisher(Image, 'yolov5/image', 10)
        self.json_publisher = self.create_publisher(String, 'yolov5/json', 10)
        self.detection_publisher = self.create_publisher(Detection2DArray, 'yolov5/detection_boxes', 10)
        self.proc_image_publisher = self.create_publisher(Image, 'yolov5/proc_image',10) # ADDED

        self.counter = 0
        self.br = CvBridge()

        #response = requests.get(self.get_parameter('weights_path').value)
        #open("data.pt", "wb").write(response.content)
        #self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./data.pt')

        # self.model = torch.hub.load('/home/joyride-obc/joyride-ros-main/src/yolov5', 'custom', path='/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt')
        self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/third_party/yolov5').get_parameter_value().string_value

       # - absolute path to best.pt
        self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt').get_parameter_value().string_value
        self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False)
        self.get_logger().info("YOLOv5 Initialized")

    def inference(self, image: Image):
        return self.model(image)

    def getDetectionArray(self, df):
        dda = Detection2DArray()

        detections = []
        self.counter += 1

        for row in df.itertuples():
            
            self.get_logger().info(f"Detected {row.name}")

            detection = Detection2D()

            detection.header.stamp = self.get_clock().now().to_msg()
            detection.header.frame_id = str(self.counter)

            hypothesises = []
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(row[6])
            hypothesis.hypothesis.score = float(row[5])

            pwc = PoseWithCovariance()
            pwc.pose.position = Point()
            pwc.pose.position.x = (int(row.xmin) + int(row.xmax)) / 2
            pwc.pose.position.y = (int(row.ymin) + int(row.ymax)) / 2

            hypothesis.pose = pwc

            hypothesises.append(hypothesis)
            detection.results = hypothesises

            bbox = BoundingBox2D()
            bbox.size_x = (int(row.xmax) - int(row.xmin)) / 2
            bbox.size_y = (int(row.ymax) - int(row.ymin)) / 2

            point = Point2D()
            point.x = (int(row.xmin) + int(row.xmax)) / 2
            point.y = (int(row.ymin) + int(row.ymax)) / 2

            center = Pose2D()
            center.position = point
            center.theta = 0.0

            detection.bbox = bbox

            detections.append(detection)



        dda.detections = detections
        dda.header.stamp = self.get_clock().now().to_msg()
        dda.header.frame_id = str(self.counter)
        return dda
    
    def add_bounding_box(self, image, results):
        for result in results.xyxy[0]:
            label = int(result[5])
            confidence = result[4]
            xmin, ymin, xmax, ymax = map(int, result[:4])

            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(image, f"{label} {confidence:.2f}", (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image

    def listener_callback(self, data):
        self.get_logger().info('Got Image')
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.inference(current_frame)

        if self.get_parameter('pub_image').value:
            #processed_image = self.br.cv2_to_imgmsg(results.ims[0])
            for result in results.xyxy[0]:
                label = int(result[5])
                confidence = result[4]
                xmin, ymin, xmax, ymax = map(int, result[:4])

                cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(current_frame, f"{label} {confidence:.2f}", (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            current_frame = self.br.cv2_to_imgmsg(current_frame)
            self.image_publisher.publish(current_frame)
            #self.image_publisher.publish(processed_image)

        if self.get_parameter('pub_json').value:
            json = String()
            json.data = results.pandas().xyxy[0].to_json(orient="records")
            self.json_publisher.publish(json)

        if self.get_parameter('pub_boxes').value:
            detections = self.getDetectionArray(results.pandas().xyxy[0])
            self.detection_publisher.publish(detections)

        # if self.get_parameter('pub_proc_image').value:
        #     image_with_bboxes = self.add_bounding_box(current_frame, results)
        #     image_with_bboxes_msg = self.br.cv2_to_imgmsg(image_with_bboxes)
        #     self.proc_image_publisher(image_with_bboxes_msg)
            #ros2 run rqt_image_view rqt_image_view



def main(args=None):
    rclpy.init(args=args)
    TDetector = TrafficSignDetector()
    rclpy.spin(TDetector)

    TDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
