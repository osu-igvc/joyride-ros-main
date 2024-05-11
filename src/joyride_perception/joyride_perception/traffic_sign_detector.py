# # Much of the code is sourced from:
# # https://github.com/open-shade/yolov5/blob/main/yolov5/interface.py

import torch
import requests
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis, BoundingBox2D, Point2D, Pose2D
from geometry_msgs.msg import PoseWithCovariance, Point
import easyocr
import numpy as np
# from PIL import Image

from dataclasses import dataclass

@dataclass
class DetectedItem():
    range: float = 0
    confidence: float = 0
    red_detected: bool = False

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('yolov5_node')

        self.declare_parameter('pub_image', True) # Keep True
        self.declare_parameter('pub_json', False)
        self.declare_parameter('pub_boxes', False)
        self.declare_parameter('pub_proc_image', True) # ADDED
        self.declare_parameter('weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_bestdataset_YOLOV5.pt')
        # self.declare_parameter('weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_end_semester.pt')

        self.subscription = self.create_subscription(
            Image,
            # '/sensors/cameras/lane/image_raw',
            # '/sensors/cameras/center/image',
            '/sensors/cameras/right/image',
            self.listener_callback,
            1
        ) # may want to mess with the queue size of 1
        self.reader = easyocr.Reader(['en'], gpu=True)

        self.image_publisher = self.create_publisher(Image, 'yolov5/image', 10)
        self.json_publisher = self.create_publisher(String, 'yolov5/json', 10)

        #self.imageProcessTimer = self.create_timer(0.05, self.processImageCB)
        self.detection_publisher = self.create_publisher(Detection2DArray, 'yolov5/detection_boxes', 10)
        self.proc_image_publisher = self.create_publisher(Image, 'yolov5/proc_image',10) # ADDED
        # self.stop_sign_range_publisher = self.create_publisher(Float32, 'StopSign_Detection/Range',10)
        #self.sign_det_range_publisher = self.create_publisher(Int32MultiArray, 'yolov5/det_range_info',

        self.counter = 0
        self.br = CvBridge()

        #response = requests.get(self.get_parameter('weights_path').value)
        #open("data.pt", "wb").write(response.content)
        #self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./data.pt')

        # self.model = torch.hub.load('/home/joyride-obc/joyride-ros-main/src/yolov5', 'custom', path='/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt')
        # absolute path to cloned yolov5 model
        self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/third_party/yolov5').get_parameter_value().string_value

       # - absolute path to best.pt
        self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_bestdataset_YOLOV5.pt').get_parameter_value().string_value
        # self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_end_semester.pt').get_parameter_value().string_value
        # self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False, device='cpu')
        self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False)
        # Used as an identifier that YOLOv5 is sucessfully running
        self.get_logger().info("YOLOv5 Initialized")

    def inference(self, image: Image):
        return self.model(image)

       
    # def add_bounding_box(self, image, results):
    #     for result in results.xyxy[0]:
    #         label = int(result[5])
    #         confidence = result[4]
    #         xmin, ymin, xmax, ymax = map(int, result[:4])
    #         # print(results.xyxy[0])

    #         cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
    #         cv2.putText(image, f"{label} {confidence:.2f}", (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
       
    #     return image

    # def rangePrediction(self, width: int) -> float:
    #     raise NotImplemented

    # def processImageCB(self):
    #     detectedItems = self.stopSignDetection(self.image)
    #     self.publish_data(detectedItems)

    # def publish_data(self, detectedItems: list[DetectedItem]):
    #     detections = Detection2D()
    #     detections.header.frame_id = "blfy_right"
    #     detections.header.stamp = self.get_clock().now().to_msg()


    #     for  i, item in enumerate(detectedItems):
    #         d = ObjectHypothesisWithPose()

    #         d.id = i
    #         d.score = item.confidence
    #         d.pose.pose.position.x      = item.range
    #         d.pose.pose.position.y      = 0
    #         d.pose.pose.position.z      = 0
    #         d.pose.pose.orientation.x   = 0
    #         d.pose.pose.orientation.y   = 0
    #         d.pose.pose.orientation.z   = 0
    #         d.pose.pose.orientation.w   = 1

    #         detections.results.append(d)

    #     #publish detections
    
    # def stopSignDetection(self, img: np.ndarray) -> list[DetectedItem]:
    #     raise NotImplemented

    # Color verification utlizing HSV values with opencv
    def verify_color_red(self, xmin, ymin, xmax, ymax, img): # img is current_frame in callback
        red_sign = False
        img_crop = img[ymin:ymax, xmin:xmax]
        hsv_img = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        img_height_hsv, img_width_hsv, channels_hsv = hsv_img.shape
        num_pix_hsv_img = img_height_hsv * img_width_hsv
        bad_hue_range_min = 70
        bad_hue_range_max = 153
        mask_not_red = cv2.inRange(hsv_img[:, :, 0], bad_hue_range_min, bad_hue_range_max)
        num_pix_not_red = np.count_nonzero(mask_not_red)
        num_pix_red = num_pix_hsv_img - num_pix_not_red
        tune_red_ver = 1.35
        if (num_pix_red > (tune_red_ver * num_pix_not_red)):
            red_sign = True
            # print('RED')
            # cv2.putText(img, 'RED', (10, 600), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
        else:
            red_sign = False
            # print("Not RED")
            # cv2.putText(img, 'NOT RED', (10, 650), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
        return red_sign, img_crop
    # Checks to see if detected text ioncludes different alterations of the word stop
    def check_read_stop(self, ocr_text_up):
        read_stop = False
        possible_stop = ['STOP', 'STO', 'TOP', 'ST', 'TO', 'OP']
        if any(word in ocr_text_up for word in possible_stop):
            read_stop = True
        else:
            read_stop = False
        return read_stop
    # Predicted distance calculation
    def pred_dist(self, xmin, ymin, xmax, ymax):
        height = ymax - ymin 
        width = xmax - xmin
        d_pred_in = 0.0
        d_pred_val = width # Default with width measurement
        if height > 1.1*width: #monitor for stop sign rotation (edge case)
            d_pred_val = height
        tune_d_pred = 10
        d_pred_in = (d_pred_val / 8045) ** (1/-0.754) + tune_d_pred # distance in INCHES
        if width > 270: #tuning for closer signs
            d_pred_in = d_pred_in + tune_d_pred - 10
        return d_pred_in

    # Callback for all of the required opperations for yolov5 prediction, color detection, text recoginition, and distance estimation
    def listener_callback(self, data): 
        self.image = self.br.imgmsg_to_cv2(data)
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.inference(current_frame)

        if self.get_parameter('pub_image').value:
            # Initializing values
            last_detected_texts = {}
            red_sign = False
            stop_sign_detected = False
            d_pred_meter = Float32()

            # Iterate through all bounding boxes from YOLOv5
            for result in results.xyxy[0]:
                # Extract info of each bounding box
                label = int(result[5])
                confidence = result[4]
                if (label == 1):
                    label = 'Stop'
                xmin, ymin, xmax, ymax = map(int, result[:4])

                # Added for screenshots
                #cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                #cv2.putText(current_frame,f'Conf: {confidence*100:.1f}%', (xmax, ymax - 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

                # Check sign color
                red_sign, img_cropped = self.verify_color_red(xmin, ymin, xmax, ymax, current_frame)
                if red_sign:
                    cv2.putText(current_frame, 'RED', (10, 600), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                else:
                    cv2.putText(current_frame, 'NOT RED', (10, 650), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

                # Check word in sign
                ocr_results = self.reader.readtext(np.array(img_cropped)) 
                detected_text = " ".join(res[1] for res in ocr_results)
                last_detected_texts[label] = detected_text
                #cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                filtered_text = ''.join(char for char in detected_text if char.isalnum())
                filtered_text_up = filtered_text.upper()
                sign_read_stop = self.check_read_stop(filtered_text_up)
                # Added for screenshot below
                # cv2.putText(current_frame, f"{detected_text}", (xmax, ymax - 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                cv2.putText(current_frame, 'Read Stop', (600, 600,), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                if sign_read_stop:
                    cv2.putText(current_frame, f'{sign_read_stop}', (600, 650), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                else:
                    cv2.putText(current_frame, f'{sign_read_stop}', (600, 700), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                
                # Only process for distance if: sign is red, sign says stop, YOLOv5 confidence > threshold
                yolo_pos_det_threshold = 0.55
                if (confidence > yolo_pos_det_threshold) and red_sign and sign_read_stop: 
                    stop_sign_detected = True
                    d_pred_in = self.pred_dist(xmin, ymin, xmax, ymax)
                    d_pred_ft = d_pred_in / 12.0
                    d_pred_meter = float(d_pred_in)*0.0254
                    d_pred_ft_str = '{:.2f}'.format(d_pred_ft)
                    # Draw bounding box around ranged stop sign
                    cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    # Display detected text (not needed)
                    cv2.putText(current_frame, f"{detected_text}", (xmax, ymax - 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    # Display confidence of YOLOv5 (not needed)
                    cv2.putText(current_frame,f'Conf: {confidence*100:.1f}%', (xmax, ymax - 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    # Display predicted distance in feet (not needed)
                    cv2.putText(current_frame, f"{d_pred_ft_str} ft.", (xmax, ymax - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            cv2.imshow('Processed Image', current_frame)
            cv2.waitKey(1)
            current_frame = self.br.cv2_to_imgmsg(current_frame)
            self.image_publisher.publish(current_frame)

        if self.get_parameter('pub_json').value:
            json = String()
            json.data = results.pandas().xyxy[0].to_json(orient="records")
            self.json_publisher.publish(json)

        if self.get_parameter('pub_boxes').value:
            detections = self.getDetectionArray(results.pandas().xyxy[0])
            self.detection_publisher.publish(detections)



def main(args=None):
    rclpy.init(args=args)
    TDetector = TrafficSignDetector()
    rclpy.spin(TDetector)

    TDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
