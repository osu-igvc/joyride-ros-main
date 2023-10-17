



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os.path

from joyride_perception.submodules.lane import Lane

# Subscribe to camera topic
# Send image to lane


class LaneDetector(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')

        

        self.output_topic = self.declare_parameter('image_output_topic', '/cameras/front_lane_markings').get_parameter_value().string_value
        self.topic_name = self.declare_parameter('image_source_topic', '/sensors/cameras/center/image/compressed').get_parameter_value().string_value



        self.video_subscriber = self.create_subscription(Image, self.topic_name, self.imageCallback, 10)
        #self.video_subscriber # prevent unused variable

        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        
        self.bridge = CvBridge()

        self.scale_ratio = 1
        

    def imageCallback(self, msg):
        self.get_logger().info('Received image from: ' + self.topic_name)
        
        new_frame = self.bridge.imgmsg_to_cv2(msg)
        self.detectLanes(new_frame)
        


    def detectLanes(self, frame):
        # lane_obj = Lane(frame)

        # lane_line_markings = lane_obj.get_line_markings()
        # lane_obj.plot_roi(plot=False)
        # warped_frame = lane_obj.perspective_transform(plot=False)
        # histogram = lane_obj.calculate_histogram(plot=True)  

        
        # left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)
        # lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
        # frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
        # lane_obj.calculate_curvature(print_to_terminal=False)
        # frame_with_lane_lines2 = lane_obj.display_curvature_offset(frame=frame_with_lane_lines, plot=True)

        width = int(frame.shape[1] * self.scale_ratio)
        height = int(frame.shape[0] * self.scale_ratio)
        frame = cv2.resize(frame, (width, height))
			
      # Store the original frame
        original_frame = frame.copy()

      # Create a Lane object
        lane_obj = Lane(orig_frame=original_frame)

      # Perform thresholding to isolate lane lines
        lane_line_markings = lane_obj.get_line_markings()

      # Plot the region of interest on the image
        lane_obj.plot_roi(plot=False)

      # Perform the perspective transform to generate a bird's eye view
      # If Plot == True, show image with new region of interest
        warped_frame = lane_obj.perspective_transform(plot=False)

      # Generate the image histogram to serve as a starting point
      # for finding lane line pixels
        histogram = lane_obj.calculate_histogram(plot=False)	
	
      # Find lane line pixels using the sliding window method 
        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
        plot=False)

      # Fill in the lane line
        lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
	
      # Overlay lines on the original frame
        frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)

      # Calculate lane line curvature (left and right lane lines)
        lane_obj.calculate_curvature(print_to_terminal=False)

      # Calculate center offset  																
        lane_obj.calculate_car_position(print_to_terminal=False)
	
      # Display curvature and center offset on image
        frame_with_lane_lines2 = lane_obj.display_curvature_offset(
        frame=frame_with_lane_lines, plot=False)
        

        self.publisher.publish(self.bridge.cv2_to_imgmsg(frame_with_lane_lines2, 'bgr8'))
        self.get_logger().info('Publishing markings on' + self.output_topic)



def main(args=None):
    rclpy.init(args=args)

    laneDetector = LaneDetector()

    rclpy.spin(laneDetector)

    laneDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
