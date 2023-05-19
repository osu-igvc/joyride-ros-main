#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

vector<Point> roi_vertices;
String window_roi = "Roi Select";
bool done = false;
Mat img;

void handleMsg(const sensor_msgs::msg::Image::SharedPtr msg){
    // Convert ROS image message to OpenCV format
    img = cv_bridge::toCvShare(msg, "bgr8")->image;

    namedWindow(window_roi, WINDOW_NORMAL);
    imshow(window_roi, img);
    Rect roi = selectROI(window_roi, img);

    cout << "ROI Selected: X: " << to_string(roi.x) << ", Y: " << to_string(roi.y) << ", WIDTH: " << to_string(roi.width) 
        << ", HEIGHT: " << to_string(roi.height) << endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("lane_roi_node_sub");
    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>("image", 1, handleMsg);

    
    while (!img.data) {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
