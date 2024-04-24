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
from std_msgs.msg import String, Int32MultiArray
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
        self.declare_parameter('pub_boxes', True)
        self.declare_parameter('pub_proc_image', True) # ADDED
        # self.declare_parameter('weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt')
        self.declare_parameter('weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_bestdataset_YOLOV5.pt')
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
        self.detection_publisher = self.create_publisher(Detection2DArray, 'yolov5/detection_boxes', 10)
        self.proc_image_publisher = self.create_publisher(Image, 'yolov5/proc_image',10) # ADDED
        #self.sign_det_range_publisher = self.create_publisher(Int32MultiArray, 'yolov5/det_range_info', 10)

        #self.imageProcessTimer = self.create_timer(0.05, self.processImageCB)

        self.counter = 0
        self.br = CvBridge()

        #response = requests.get(self.get_parameter('weights_path').value)
        #open("data.pt", "wb").write(response.content)
        #self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='./data.pt')

        # self.model = torch.hub.load('/home/joyride-obc/joyride-ros-main/src/yolov5', 'custom', path='/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best.pt')
        self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/third_party/yolov5').get_parameter_value().string_value

       # - absolute path to best.pt
        self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/best_bestdataset_YOLOV5.pt').get_parameter_value().string_value
        self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False)
        self.get_logger().info("YOLOv5 Initialized")

    def inference(self, image: Image):
        return self.model(image)

    def getDetectionArray(self, df):
        dda = Detection2DArray()

        detections = []
        self.counter += 1

        for row in df.itertuples():
           
            # self.get_logger().info(f"Detected {row.name}")

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

        # print("row")
        # print(row)

        return dda
       
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

    def listener_callback(self, data): 
        self.image = self.br.imgmsg_to_cv2(data)

        #deleted row
        #self.get_logger().info('Got Image')
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.inference(current_frame)
        # reader = easyocr.Reader(['en'], gpu=True)

        # reader = easyocr.Reader(['en'])

        if self.get_parameter('pub_image').value:
            #processed_image = self.br.cv2_to_imgmsg(results.ims[0])
            #print(results.xyxy[0])
            # reader = easyocr.Reader(['en'])


            frame_skip_counter = 0
            frame_skip_threshold = 100000
            last_detected_texts = {}
            red_sign = False

            for result in results.xyxy[0]:
                # reader = easyocr.Reader(['en'], gpu=True)
                frame_skip_counter += 1
                label = int(result[5])
                confidence = result[4]
                xmin, ymin, xmax, ymax = map(int, result[:4])
                pos_det_threshold = 0.01
                img_verify_color = current_frame[ymin:ymax, xmin:xmax]
                hsv_img = cv2.cvtColor(img_verify_color, cv2.COLOR_BGR2HSV)
                img_height_hsv, img_width_hsv, channels_hsv = hsv_img.shape
                num_pix_hsv_img = img_height_hsv * img_width_hsv
                bad_hue_range_min = 40
                bad_hue_range_max = 153
                mask_not_red = cv2.inRange(hsv_img[:, :, 0], bad_hue_range_min, bad_hue_range_max)
                num_pix_not_red = np.count_nonzero(mask_not_red)
                num_pix_red = num_pix_hsv_img - num_pix_not_red
                tune_red_ver = 1
                if (num_pix_red > (tune_red_ver * num_pix_not_red)):
                    red_sign = True
                    # print('RED')
                    cv2.putText(current_frame, 'RED', (10, 600), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                else:
                    red_sign = False
                    # print("Not RED")
                    cv2.putText(current_frame, 'NOT RED', (10, 750), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

                if (label == 1):
                    label = 'Stop'
                frame_skip_counter = 0
                ocr_results = self.reader.readtext(np.array(img_verify_color)) # img_verify_color is cropped img
                detected_text = " ".join(res[1] for res in ocr_results)
                last_detected_texts[label] = detected_text
                print('OCR Running', detected_text)
                cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                # Only process for distance if sign is red and confidence > threshold
                if (confidence > pos_det_threshold) and red_sign:
                    height = ymax - ymin 
                    width = xmax - xmin
                    centr_bbox = xmin + (width/2)
                    d_pred_val = width # Default with width measurement
                    if height > 1.1*width: #monitor for stop sign rotation (edge case)
                        d_pred_val = height
                    tune_d_pred = 10
                    d_pred = (d_pred_val / 8045) ** (1/-0.754) + tune_d_pred
                    if width > 270: #tuning for closer signs
                        d_pred = d_pred + tune_d_pred - 10
                    d_pred_ft = float(d_pred) / 12.0
                    d_pred_ft_str = '{:.2f}'.format(d_pred_ft)
                #     ocr_results = reader.readtext(np.array(cropped_img))
                #     detected_text = " ".join(res[1] for res in ocr_results)
                #     last_detected_texts[label] = detected_text
                # else:
                #     detected_text = last_detected_texts.get(label, "")
                    print(d_pred_ft,'ft.', d_pred, 'in')
                    cv2.rectangle(current_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    #cv2.putText(current_frame, f"{label} {confidence:.2f}", (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    #cv2.putText(current_frame, f"{d_pred_ft_str}", (xmax, ymax + 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    # disp_img_height, disp_img_width, _ = current_frame.shape
                    # xmin_flip = disp_img_width - xmin
                    # xmax_flip = disp_img_width - xmax
                    # ymin_flip = disp_img_height -  ymin
                    # ymax_flip = disp_img_height - ymax
                    cv2.putText(current_frame, f"{label}, Conf: {confidence:.2f}", (xmax, ymax - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    cv2.putText(current_frame, f"{d_pred_ft_str}, {detected_text}", (xmax, ymax - 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    
                
                    # Trying this - if it doesn't work switch to above
                    # cv2.rectangle(current_frame, (xmax, ymax), (xmin, ymin), (0, 255, 0), 2)
                    # cv2.putText(current_frame, f"{label} {confidence:.2f}", (xmax, ymax - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    # cv2.putText(current_frame, f"{d_pred_ft_str}", (xmin, ymin + 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    # Structure: Predicted Distance in Inches, Predicted Distance in Feet, Confidence, Color is Red (True for Red, False for Not Red)
                    
                    #output_info =np.array([d_pred, d_pred_ft, confidence, red_sign])




                # Crop the detected stop sign
                # cropped_img = current_frame[ymin:ymax, xmin:xmax]
                # cropped_img = current_frame.crop()
                # flipped_img = cv2.flip(cropped_img,0)
                # flipped_img = cv2.flip(flipped_img,1)
                # conver_image = cv2.cvtColor(flipped_img,cv2.COLOR_BGR2RGB)
                # pil_image = Image.fromarray(conver_image)

                # if frame_skip_counter % frame_skip_threshold == 0:
                #     reader = easyocr.Reader(['en'])


                #     ocr_results = reader.readtext(np.array(cropped_img))
                #     detected_text = " ".join(res[1] for res in ocr_results)
                #     last_detected_texts[label] = detected_text
                # else:
                #     detected_text = last_detected_texts.get(label, "")


                # imgGray = cv2.cvtColor(flipped_img, cv2.COLOR_BGR2GRAY)
                # cv2.imshow('flipped image',flipped_img)
                # ocr_results = reader.readtext(flipped_img)
                # ocr_results = reader.readtext(np.array(flipped_img))
                # detected_text = " ".join(res[1] for res in ocr_results)
                # print("printed ocr", detected_text)
                #print(ymin)
                #print(height)
                #print('W:',width)
                #print('U:', centr_bbox)
                #print('H:', height)

                #d_text = "Distance: {:.2f}".format(d_pred)
                #cv2.putText(current_frame, d_pred, (xmin, ymax-5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
               
                #cv2.putText(current_frame, f"{result.name} {confidence:.2f}", (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                #self.get_logger().info(f"Detected {result.name}")
                # Font size changed to 2 above
            #printing array for troubleshooting
            # frame_skip_counter += 1
            cv2.imshow('Processed Image', current_frame)
            cv2.waitKey(1)
            #cv2.imwrite('/home/joyride-obc/joyride-ros-main/src/joyride_perception/first_frame4meter.jpg', current_frame)

            current_frame = self.br.cv2_to_imgmsg(current_frame)
            self.image_publisher.publish(current_frame)
            #self.sign_det_range_publisher.publish(output_info)
            
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
