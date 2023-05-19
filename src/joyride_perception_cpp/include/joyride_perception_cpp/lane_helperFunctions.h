#include <iostream>
#include <opencv2/opencv.hpp> 

using namespace cv;
using namespace std;

#ifndef helperFunctions
#define helperFunctions

/**
 * Compare the 2nd index of two Vec4i's
 * @return: true if a's 2nd index is greater than b's
*/
bool compareSecondIndex(Vec4i& a, Vec4i& b) {
    return a[1] > b[1];
}

/**
 * Compare the y indecies of within a vector<point>
 * @return: true if a's y coordinate is greater than b's
*/
bool comparePointY(Point& a, Point& b) {
    return a.y > b.y;
}

/**
 * Compare Lane's X coordinates
 * @return: true if vector<vector<Points> first x coordinate is smaller
*/
bool compareXCoordinatesOfLanes(vector<Point>& a, vector<Point>& b) {
    return a[0].x < b[0].x;
}

/**
 * Compare Lane's X coordinates
 * @return: true if vector<vector<Points> first x coordinate is smaller
*/
bool compareYCoordinatesOfLanes(vector<Point>& a, vector<Point>& b) {
    return a[0].y < b[0].y;
}

float slope(Point a, Point b){
    return (b.y - a.y) / (b.x - a.x);
}

/**
 * @param frame of original camera
 * @param laneLines 
 * @return Mat showing laneLines on the given frame
*/
Mat overlayLaneSegments(Mat frame, vector<vector<Point>> laneLines) {
    // Overlay dashed Lines onto frame
    Mat overlayCombined = frame.clone();
    for (size_t i = 1; i < laneLines.size()-1; i++) {
        for (size_t j = 1; j < laneLines[i].size(); j++) {
            line(overlayCombined, laneLines[1][j-1], laneLines[1][j], Scalar(0,0,255), 3, LINE_AA);
        }
    }
    // Overlay left and right lanes
    for (size_t j = 1; j < laneLines[0].size(); j++) {
        line(overlayCombined, laneLines[0][j-1], laneLines[0][j], Scalar(0,255,0), 3, LINE_AA);
    }
    int sizeOfLaneLines = laneLines.size() - 1;

    for (size_t j = 1; j < laneLines[sizeOfLaneLines].size(); j++) {
        // Point start
        line(overlayCombined, laneLines[sizeOfLaneLines][j-1], laneLines[sizeOfLaneLines][j], Scalar(255,0,0), 3, LINE_AA);
    }

    // Deallocate Memory
    laneLines.clear();

    return overlayCombined;
}

/**
 * @param frame of camera image
 * @param lines detected from the houghTransform
 * @return Mat showing the lines on the frame
*/
Mat overlayHoughLines(Mat frame, vector<Vec4i> lines) {
    // ------------Overlay------------------------------------------------
    // Create a clone of frame to draw the lane lines on
    Mat overlay = frame.clone();
    // Draw lines onto image
    for (size_t i = 0; i < lines.size(); i++) {
        // Point start
        line(overlay, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, LINE_AA);
    }

    // Deallocate Memory
    lines.clear();
    frame.release();

    return overlay;
}

/**
 * @param frame of camera image
 * @param lanes sorted line segments that represent the lane lines
 * @param color of lines to draw - default green
 * @return Mat showing the liven lanes on the camera frame
*/
Mat overlayLanes(Mat frame, vector<vector<Point>> lanes, const Scalar& color = Scalar(0,255,0)) {
    Mat overlay = frame.clone();
    for (size_t i = 0; i < lanes.size(); i++) {
        for (size_t j = 1; j < lanes[i].size(); j++) {
            line(overlay, lanes[i][j-1], lanes[i][j], color, 3, LINE_AA);
        }
    }
    // Deallocate Memory
    lanes.clear();
    frame.release();

    return overlay;
}

/**
 * @param frame of camera image
 * @param lanes sorted line segments that represent the lane lines
 * @return black and white image of the lines
*/
Mat overlayLanes_binary(Mat frame, vector<vector<Point>> lanes){
    Mat black_white_lanes(frame.size(), CV_8UC3, Scalar(0,0,0));
    black_white_lanes = overlayLanes(black_white_lanes, lanes, Scalar(255,255,255));
    cvtColor(black_white_lanes, black_white_lanes, COLOR_BGR2GRAY);

    // Deallocate Memory
    lanes.clear();

    return black_white_lanes;
}


#endif