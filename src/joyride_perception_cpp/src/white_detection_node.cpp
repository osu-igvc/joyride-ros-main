#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include <string>
#include <condition_variable>

#include "lane_mainFunctions.h"
#include "lane_helperFunctions.h"
#include "slider.h"

using namespace cv;
using namespace std;

bool isCompressed = false;

class WhiteDetection : public rclcpp::Node
{
    public:
        WhiteDetection() : Node("white_detection_node"){

            if (isCompressed){
                // TODO
            } else{
                sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>("image", 5,
                    bind(&WhiteDetection::callback_normal, this, placeholders::_1));
            }

            sub_threshhold_values_ = this->create_subscription<std_msgs::msg::String>("perception/lane/slider_values", 5,
                bind(&WhiteDetection::adjustThresholds, this, placeholders::_1));

            pub_lane_overlay_ = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/overlay",10);
            pub_lane_point_msg = this->create_publisher<sensor_msgs::msg::Image>("perception/object_to_point_cloud",10);

            pub_white_ = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/white",10);
            pub_white_two = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/white_two",10);
            pub_edge_ = this->create_publisher<sensor_msgs::msg::Image>("perception/lane/edge",10);
        }

    private:

        void callback_compressed(){ // add msg here
        //     // TODO
        }

        void callback_normal(const sensor_msgs::msg::Image::SharedPtr msg){
            Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
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
            Mat whiteTwo = whiteAttemptTwo(roiImg);

            // Edge detection
            Mat edge = getLaneEdgesInFrame(white);

            // Lanes
            handleLanes(image, edge);

            // Potholes
            handlePotholes(image, edge);

            // Test Publish msgs
            sensor_msgs::msg::Image::SharedPtr white_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", white).toImageMsg();
            pub_white_->publish(*white_msg);  
            sensor_msgs::msg::Image::SharedPtr white_msg_two = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", whiteTwo).toImageMsg();
            pub_white_two->publish(*white_msg_two); 
            sensor_msgs::msg::Image::SharedPtr edge_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", edge).toImageMsg();
            pub_edge_->publish(*edge_msg); 
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
                6hsv values
                HOUGH_THRESHOLD
                HOUGH_MIN_LINE_LENGTH
                HOUGH_MAX_LINE_GAP
                SEP_DISTANCE_BETWEEN_SEGMENTS
                SEP_DISTANCE_BETWEEN_LANES_Y
                SEP_DISTANCE_BETWEEN_DASH_X
            */
            WHITE_THRESHOLD = values[0];
            MIN_WHITE_REGION_SIZE = values[1];
            hlb = values[2];
            slb = values[3];
            vlb = values[4];
            hup = values[5];
            sup = values[6];
            vup = values[7];
            HOUGH_THRESHOLD = values[8];
            HOUGH_MIN_LINE_LENGTH = values[9];
            HOUGH_MAX_LINE_GAP_SOLID = values[10];
            SEP_DISTANCE_BETWEEN_SEGMENTS = values[11];
            SEP_DISTANCE_BETWEEN_LANES_Y = values[12];
            SEP_DISTANCE_BETWEEN_DASH_X = values[13];
        }        

        void handleLanes(Mat frame, Mat edges) {
            vector<Vec4i> houghLines = HoughTransformOnEdges(edges, true);
            vector<vector<Point>> laneLines = LaneLinesFromSegments(houghLines);

            // Apply Lanes to frame
            Mat overlay = overlayLanes(frame, laneLines);

            // Display lanes on binary image to convert to pointCloud2
            Mat lanes_binary_img = overlayLanes_binary(frame, laneLines);

            // Publish Messages
            sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay).toImageMsg();
            pub_lane_overlay_->publish(*overlay_msg);  

            sensor_msgs::msg::Image::SharedPtr pointcloud_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", lanes_binary_img).toImageMsg();
            pub_lane_point_msg->publish(*pointcloud_msg); 
            
            // Deallocate Memory
            frame.release();
            edges.release();
            overlay.release();
            lanes_binary_img.release();
            houghLines.clear();
            laneLines.clear();
        }

        void handlePotholes(Mat frame, Mat edges) {
            // To-do
            // Publish Messages
            // sensor_msgs::msg::Image::SharedPtr overlay_msg 
        }

        void updateROI(const std_msgs::msg::String::SharedPtr msg) {
            // turn msg into string
            string msg_string = msg->data;

            // Get individual numbers from string
            stringstream ss(msg_string);
            vector<int> values;
            int num;
            while (ss >> num) {
                values.push_back(num);
            }
            
            // Set values
            /*
                roi.x
                roi.y
                roi.width
                roi.height
            */
            if ( 
                (values[0] >= 0 && values[1] >= 0) 
                &&                
                (
                    ((WIDTH == 0 || HEIGHT == 0) && values[3] >= values[1] && values[2] >= values[0]) 
                    || 
                    (values[0] + values[2] <= WIDTH && values[1] + values[3] <= HEIGHT)
                )
            ){
                ROI.x = values[0];
                ROI.y = values[1];
                ROI.width = values[2];
                ROI.height = values[3];
                cout << "ROI Updated to " << msg->data.c_str() << endl;
            } else {
                cout << "Invalid ROI: " << msg->data.c_str() << endl;
            }
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_threshhold_values_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_lane_overlay_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_lane_point_msg;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_white_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_white_two;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edge_;


};

int main(int argc, char **argv){

    initGlobalVariablesForLab();

    rclcpp::init(argc,argv);
    auto node = make_shared<WhiteDetection>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}