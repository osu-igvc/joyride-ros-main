#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "slider.h"

using namespace cv;
using namespace std;

void onTrackbar(int, void* publisher){
    std_msgs::msg::String msg;
    
    // create msg
    /*
        WHITE_THRESHOLD
        MIN_WHITE_REGION_SIZE
        HOUGH_THRESHOLD
        HOUGH_MIN_LINE_LENGTH
        HOUGH_MAX_LINE_GAP
        SEP_DISTANCE_BETWEEN_SEGMENTS
        SEP_DISTANCE_BETWEEN_LANES_Y
        SEP_DISTANCE_BETWEEN_DASH_X
    */
    msg.data = to_string(sliderVal_White_Threshold)
        + " " + to_string(sliderVal_Min_White_Region_Size)
        + " " + to_string(sliderVal_hsv_hlb)
        + " " + to_string(sliderVal_hsv_slb)
        + " " + to_string(sliderVal_hsv_vlb)
        + " " + to_string(sliderVal_hsv_hup)
        + " " + to_string(sliderVal_hsv_sup)
        + " " + to_string(sliderVal_hsv_vup)
        + " " + to_string(sliderVal_Hough_Threshold)
        + " " + to_string(sliderVal_Hough_Min_Line_Length)
        + " " + to_string(sliderVal_Hough_Max_Line_Gap)
        + " " + to_string(sliderVal_Sep_Distance_Between_Segments)
        + " " + to_string(sliderVal_Sep_Distance_Between_Lanes_Y)
        + " " + to_string(sliderVal_Sep_Distance_Between_Dash_X);
    // msg.data = to_string(sliderVal_White_Threshold)
    //     + " " + to_string(sliderVal_Min_White_Region_Size)
    //     + " " + to_string(sliderVal_Hough_Threshold)
    //     + " " + to_string(sliderVal_Hough_Min_Line_Length)
    //     + " " + to_string(sliderVal_Hough_Max_Line_Gap)
    //     + " " + to_string(sliderVal_Sep_Distance_Between_Segments)
    //     + " " + to_string(sliderVal_Sep_Distance_Between_Lanes_Y)
    //     + " " + to_string(sliderVal_Sep_Distance_Between_Dash_X);
    
    // publish msg
    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(*(static_cast<std::shared_ptr<void>*>(publisher)))->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);

    // Create empty image
    int imgWidth = 640;
    int imgHeight = 1;
    Mat img = Mat::zeros(imgHeight, imgWidth, CV_8UC3);

    // Create Slider
    namedWindow(window_Slider, WINDOW_NORMAL); 
    imshow(window_Slider, img);
    
    // Create Node 
    auto node = rclcpp::Node::make_shared("slider_node");
    // Create publisher
    auto publisher = node->create_publisher<std_msgs::msg::String>("perception/lane/slider_values", 10);

    initSliderVariables();

    // Create Trackbars
    createTrackbar( track_WHITE_THRESHOLD, window_Slider, &sliderVal_White_Threshold, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Min_White_Region_Size, window_Slider, &sliderVal_Min_White_Region_Size, alpha_slider_max, onTrackbar, static_cast<void*>(&publisher));

    createTrackbar( track_hsv_hlb, window_Slider, &sliderVal_hsv_hlb, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_hsv_slb, window_Slider, &sliderVal_hsv_slb, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_hsv_vlb, window_Slider, &sliderVal_hsv_vlb, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_hsv_hup, window_Slider, &sliderVal_hsv_hup, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_hsv_sup, window_Slider, &sliderVal_hsv_sup, 255, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_hsv_vup, window_Slider, &sliderVal_hsv_vup, 255, onTrackbar, static_cast<void*>(&publisher));

    createTrackbar( track_Hough_Threshold, window_Slider, &sliderVal_Hough_Threshold, 65, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Hough_Min_Line_Length, window_Slider, &sliderVal_Hough_Min_Line_Length, 85, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Hough_Max_Line_Gap, window_Slider, &sliderVal_Hough_Max_Line_Gap, alpha_slider_max, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Sep_Distance_Between_Segments, window_Slider, &sliderVal_Sep_Distance_Between_Segments, alpha_slider_max, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Sep_Distance_Between_Lanes_Y, window_Slider, &sliderVal_Sep_Distance_Between_Lanes_Y, alpha_slider_max, onTrackbar, static_cast<void*>(&publisher));
    createTrackbar( track_Sep_Distance_Between_Dash_X, window_Slider, &sliderVal_Sep_Distance_Between_Dash_X, alpha_slider_max, onTrackbar, static_cast<void*>(&publisher));

    // Set size of window
    // resizeWindow(window_Slider, 640, 200);

    // Keep sliders open until exit on window is pushed
    while (true) {
        if (getWindowProperty(window_Slider, WND_PROP_AUTOSIZE) == -1){
            exit(0);
        }
        waitKey();
    }

    rclcpp::spin(node);
    // Destroy all windows
    destroyAllWindows();
    return 0;
}