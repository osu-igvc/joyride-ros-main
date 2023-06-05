#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include <string>
#include <condition_variable>

#include "lane_mainFunctions.h"

using namespace cv;
using namespace std;

// Uses compressed image, switch commented code out for testing with non-compressed data
string topicmsg = "/sensors/cameras/center/image/compressed"; 
// string topicmsg = "/sensors/cameras/center/image";

class WhiteDetection : public rclcpp::Node
{
    public:
        WhiteDetection() : Node("white_detection_node"){

            // sub_camera = this->create_subscription<sensor_msgs::msg::Image>(topicmsg, 5, bind(&WhiteDetection::callback, this, placeholders::_1)); // Non-compressed image
            sub_camera = this->create_subscription<sensor_msgs::msg::CompressedImage>(topicmsg,5, bind(&WhiteDetection::callback, this, placeholders::_1)); // Compressed

            sub_threshhold_values = this->create_subscription<std_msgs::msg::String>("perception/lane/slider_values", 5,
                bind(&WhiteDetection::adjustThresholds, this, placeholders::_1));

            pub_lane_overlay = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/overlay",10);
            pub_lane_point_msg = this->create_publisher<sensor_msgs::msg::Image>("perception/object_to_point_cloud",10);
            pub_lane_pixel_distance = this->create_publisher<std_msgs::msg::Int32>("perception/lane/pixel_distance_from_center", 10);

            pub_white = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/white",10);
            pub_edge = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/edge",10);
        }

    private:

        void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){ //const sensor_msgs::msg::Image::SharedPtr msg //non-compressed  const sensor_msgs::msg::CompressedImage::SharedPtr // Compressed
            // Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; // Non-compressed
            Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image; // Compressed
            handleImage(image);
        }

        void handleImage(Mat image){
            if (image.empty()){
                cout << "Recieved Empty MSG" << endl;
                exit(0);
            } else if (!isHeightWidthSet()){
                setHeightWidth(image);
                cout << "lane: " << WIDTH << " " << HEIGHT << endl;
            }

            // Apply ROI
            Mat roiImg = image(ROI);
            if (roiImg.empty()){
                cout << "Recieved Empty ROI" << endl;
                exit(0);
            }

            // White detection
            Mat white = getWhiteInFrame(roiImg);

            // Edge detection
            Mat edge = getLaneEdgesInFrame(white);

            // Lanes
            handleLanes(image, edge);

            // Test Publish msgs
            sensor_msgs::msg::Image::SharedPtr white_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", white).toImageMsg();
            pub_white->publish(*white_msg);
            sensor_msgs::msg::Image::SharedPtr edge_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", edge).toImageMsg();
            pub_edge->publish(*edge_msg); 
        }

        void adjustThresholds(const std_msgs::msg::String::SharedPtr msg) {
            // turn msg into string
            string msg_string = msg->data;

            // Get individual numbers from string
            stringstream ss(msg_string);
            vector<int> values;
            int num;
            while (ss >> num) {
                values.push_back(num);
            }

            // assign values :: see lane_slider_node.cpp -> onTrackbar for order of values
            /*
                WHITE_THRESHOLD
                MIN_WHITE_REGION_SIZE
                6 hsv values
                HOUGH_THRESHOLD
                HOUGH_MIN_LINE_LENGTH
                HOUGH_MAX_LINE_GAP
                SEP_DISTANCE_BETWEEN_SEGMENTS
                SEP_DISTANCE_BETWEEN_LANES_Y
            */
            WHITE_THRESHOLD = values[0];
            MIN_WHITE_REGION_SIZE = values[1];
            HUE_LOW = values[2];
            SAT_LOW = values[3];
            VAL_LOW = values[4];
            HUE_HIGH = values[5];
            SAT_HIGH = values[6];
            VAL_HIGH = values[7];
            HOUGH_THRESHOLD = values[8];
            HOUGH_MIN_LINE_LENGTH = values[9];
            HOUGH_MAX_LINE_GAP = values[10];
            SEP_DISTANCE_BETWEEN_SEGMENTS = values[11];
            SEP_DISTANCE_BETWEEN_LANES_Y = values[12];
        }        

        void handleLanes(Mat frame, Mat edges) {
            vector<Vec4i> houghLines = HoughTransformOnEdges(edges);
            if (houghLines.size() > 0) {

                vector<Lane> laneLines = LaneLinesFromSegments(houghLines);


                // Apply Lanes to frame
                Mat overlay = overlayLanes(frame, laneLines);

                // Display lanes on binary image to convert to pointCloud2
                // Non-separated
                Mat lanes_binary_img = overlayLanes_binary(frame, laneLines);
                // Separated
                pair<Mat, Mat> binary_lanes = overlayLanes_binary_separated(frame, laneLines);
                Mat binary_solid_lanes = binary_lanes.first;
                Mat binary_dash_lanes = binary_lanes.second;

                // For using velocity controller gain to stay in center of line
                int distance_from_center = calculateDistanceFromCenter(laneLines);

                // Publish Messages
                sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay).toImageMsg();
                pub_lane_overlay->publish(*overlay_msg);  

                sensor_msgs::msg::Image::SharedPtr pointcloud_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", lanes_binary_img).toImageMsg();
                pub_lane_point_msg->publish(*pointcloud_msg); 
                pointcloud_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", binary_solid_lanes).toImageMsg();
                pub_lane_point_msg->publish(*pointcloud_msg); 
                pointcloud_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", binary_dash_lanes).toImageMsg();
                pub_lane_point_msg->publish(*pointcloud_msg); 

                auto distance_center_msg = std_msgs::msg::Int32();
                distance_center_msg.data = distance_from_center;
                pub_lane_pixel_distance->publish(distance_center_msg);
                
                // Deallocate Memory
                frame.release();
                edges.release();
                overlay.release();
                lanes_binary_img.release();
                binary_solid_lanes.release();
                binary_dash_lanes.release();
                houghLines.clear();
                laneLines.clear();
            }
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_threshhold_values;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera; // non-compressed version
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_camera; // compressed version

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_lane_overlay;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_lane_point_msg;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_lane_pixel_distance;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_white;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edge;


};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = make_shared<WhiteDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}