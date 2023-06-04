#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "lane_mainFunctions.h"

using namespace cv;
using namespace std;

const int alpha_slider_max = 255;
char window_Slider[] = "Adjust Lane Detection Thresholds";

// Slider values
int sliderVal_White_Threshold;
int sliderVal_Min_White_Region_Size;
int sliderVal_hsv_hlb;
int sliderVal_hsv_slb;
int sliderVal_hsv_vlb;
int sliderVal_hsv_hup;
int sliderVal_hsv_sup;
int sliderVal_hsv_vup;
int sliderVal_Hough_Threshold;
int sliderVal_Hough_Min_Line_Length;
int sliderVal_Hough_Max_Line_Gap;
int sliderVal_Sep_Distance_Between_Segments;
int sliderVal_Sep_Distance_Between_Lanes_Y;

// initialize name of trackbars
char track_WHITE_THRESHOLD[] =                  "White Threshold";
char track_Min_White_Region_Size[] =            "Minimum White Regions";
char track_hsv_hlb[] =                          "H Low Bound";
char track_hsv_slb[] =                          "S Low Bound";
char track_hsv_vlb[] =                          "V Low Bound";
char track_hsv_hup[] =                          "H High Bound";
char track_hsv_sup[] =                          "S High Bound";
char track_hsv_vup[] =                          "V High Bound";
char track_Hough_Threshold[] =                  "Hough: Threshold";
char track_Hough_Min_Line_Length[] =            "Hough: Min Length of Line";
char track_Hough_Max_Line_Gap[] =               "Hough: Max Line Gap";
char track_Sep_Distance_Between_Segments[] =    "Lane Division: Gap considered Same Lane";
char track_Sep_Distance_Between_Lanes_Y[] =     "Lane Division: Y Gap for different Lanes";


void initSliderVariables () {
    sliderVal_White_Threshold = WHITE_THRESHOLD;
    sliderVal_Min_White_Region_Size = MIN_WHITE_REGION_SIZE;
    sliderVal_hsv_hlb = HUE_LOW;
    sliderVal_hsv_slb = SAT_LOW;
    sliderVal_hsv_vlb = VAL_LOW;
    sliderVal_hsv_hup = HUE_HIGH;
    sliderVal_hsv_sup = SAT_HIGH;
    sliderVal_hsv_vup = VAL_HIGH;
    sliderVal_Hough_Threshold = HOUGH_THRESHOLD;
    sliderVal_Hough_Min_Line_Length = HOUGH_MIN_LINE_LENGTH;
    sliderVal_Hough_Max_Line_Gap = HOUGH_MAX_LINE_GAP;
    sliderVal_Sep_Distance_Between_Segments = SEP_DISTANCE_BETWEEN_SEGMENTS;
    sliderVal_Sep_Distance_Between_Lanes_Y = SEP_DISTANCE_BETWEEN_LANES_Y;
}

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
        + " " + to_string(sliderVal_Sep_Distance_Between_Lanes_Y);
    
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