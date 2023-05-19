#include <iostream>
#include <opencv2/opencv.hpp> 

#include "lane_helperFunctions.h"

using namespace cv;
using namespace std;

#ifndef laneDetection
#define laneDetection

// ------------Global Variables---------------------------------------------
// Roi
int HEIGHT = 0;
int WIDTH = 0;
Rect ROI; // Defualt to whole frame
// Lane Color Detection
int WHITE_THRESHOLD = 230; // MAX: 243 // [0-255] // 8-> 200 // 3 -> 180 // 11 -> 230 // Circle -> 225
int MIN_WHITE_REGION_SIZE = 25;  // minimum size in pixels 10<x<25 // Circle -> 20
// Mat SE = getStructuringElement(MORPH_RECT, Size(3,3));
// HSV White
int hlb = 0;
int slb = 0;
int vlb = 220;
int hup = 180;
int sup = 30;
int vup = 255;

// Hough Transfrom
int HOUGH_THRESHOLD = 25; //50 // 8->75 // Circle -> 25
static double HOUGH_THETA = CV_PI/180;
int HOUGH_MIN_LINE_LENGTH = 15;
int HOUGH_MAX_LINE_GAP_SOLID = 75;
int HOUGH_MAX_LINE_GAP_DASH = 100;
// Lane Separation
int SEP_DISTANCE_BETWEEN_SEGMENTS = 45; // Is segment within the same Lane // 8->30 // InLab (circle)->50
int SEP_DISTANCE_BETWEEN_LANES_Y = 100; // Minimum distance a lane can be to be considered Solid
int SEP_DISTANCE_BETWEEN_DASH_X = 40;

void initGlobalVariablesForLab() {
    WHITE_THRESHOLD = 225;
    MIN_WHITE_REGION_SIZE = 50;
    HOUGH_THRESHOLD = 35;
    HOUGH_MIN_LINE_LENGTH = 15;
    SEP_DISTANCE_BETWEEN_SEGMENTS = 38;
    
}

void initGlobalVariablesForRoadway() {
    WHITE_THRESHOLD = 200;
    MIN_WHITE_REGION_SIZE = 15;
    HOUGH_THRESHOLD = 75;
    HOUGH_MIN_LINE_LENGTH = 15;
    SEP_DISTANCE_BETWEEN_SEGMENTS = 30;
    
}

void initGlobalVariablesForVideo4() {
    WHITE_THRESHOLD = 180;
    MIN_WHITE_REGION_SIZE = 40;
    HOUGH_THRESHOLD = 75;
    HOUGH_MIN_LINE_LENGTH = 15;
    SEP_DISTANCE_BETWEEN_SEGMENTS = 30;
    
}

void setHeightWidth(Mat frame) {
    HEIGHT = frame.rows;
    WIDTH = frame.cols;
    ROI.x = 0;
    ROI.y = 0;
    ROI.width = WIDTH;
    ROI.height = HEIGHT;
}

bool isHeightWidthSet() {
    return (WIDTH != 0 && HEIGHT != 0);
}

Mat applyRoi(Mat frame){
    Rect roi(0, HEIGHT/8, WIDTH, 7*HEIGHT/8);
    // Set ROI
    // roi for 7,8 (0, HEIGHT/8, WIDTH, 7*HEIGHT/8)
    // roi for 3 (0, 5*HEIGHT/8, WIDTH, 3*HEIGHT/8)
    return frame(roi);
}


Mat getWhiteInFrame(Mat frame) {
    // ------------RGB to LAB---------------------------------------------
    // RBG to Lab color space
    Mat labFrame;
    cvtColor(frame, labFrame, COLOR_RGB2Lab);
    // Split into L, a, and b channels
    vector<Mat> lab_channels;
    split(labFrame, lab_channels);
    // extract L channel
    labFrame = lab_channels[0];
    // Threshold for white pixels
    labFrame = labFrame > WHITE_THRESHOLD;

    // ------------Remove Small Regions-----------------------------------
    // Perform connected component labeling
    Mat labels, stats, centroids;
    int numLabels = connectedComponentsWithStats(labFrame, labels, stats, centroids);
    // Filter out small connected components (specks)
    Mat big_white_pixels = Mat::zeros(labFrame.size(), CV_8UC1);
    for (int i = 1; i < numLabels; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        if (area >= MIN_WHITE_REGION_SIZE) {
            big_white_pixels.setTo(255, labels == i);
        }
    }

    // Deallocate Memory
    lab_channels.clear();
    frame.release();
    labFrame.release();
    labels.release();
    stats.release();
    centroids.release();

    return big_white_pixels;
}

Mat whiteAttemptTwo(Mat frame){
    Mat hsvImg;
    cvtColor(frame, hsvImg, COLOR_BGR2HSV);

    Scalar lowerWhite = Scalar(hlb, slb, vlb);
    Scalar upperWhite = Scalar(hup, sup, vup);

    Mat whiteMask;
    inRange(hsvImg, lowerWhite, upperWhite, whiteMask);

    return whiteMask;
}


/**
 * @param frame from camera
 * @return Mat of the edges from the isolated lane lines
*/
Mat getLaneEdgesInFrame(Mat frame){
    // ------------Dilate-------------------------------------------------
    // Dilate image
    // Mat SE = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    // cout << "SE" << endl;
    // dilate(big_white_pixels, big_white_pixels, SE);
    
    // cout << "Image Dilated" << endl;

    // ------------Edges--------------------------------------------------
    
    Canny(frame, frame, 100, 200); // 50,150?? no difference to 100,200

    return frame;
}

/**
 * @param Mat frame of edges
 * @return vector<Vec4i> of hough line segemnts in y_1 descending order
*/
vector<Vec4i> HoughTransformOnEdges(Mat frame, bool isSolid) {
    // ------------HoughLinesP--------------------------------------------
    std::vector<Vec4i> lines; // (x_1, y_1, x_2, y_2) Where 2 is the end point
    if (isSolid) {
        HoughLinesP(frame, lines, 1.75, HOUGH_THETA, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP_SOLID);
    } else {
        HoughLinesP(frame, lines, 1.75, HOUGH_THETA, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP_DASH);
    }
    
    // ------------Sort HoughLines----------------------------------------
    // y_1 is sorted in descending order
    if (!lines.empty()){
        sort(lines.begin(), lines.end(), compareSecondIndex);
    }

    // Deallocate Memory
    frame.release();

    return lines;
}


/**
 *
 *  separatedLanes[i] -> Separation of Lane Lines into 'i' lanes
 *  separatedLanes[i][j] -> 'j'th point that makes up 'i'th Lane Line
 *  separatedLanes[i][j][Point] -> .x and .y
 * 
 * @param lines from the houghTransform
 * @return vector of vector points that make up the laneLines 
 *          sorted from left to right and the points from bottom of the image up
 * 
*/
vector<vector<Point>> LaneLinesFromSegments(vector<Vec4i> lines) {
    vector<vector<Point>> laneLines;

    // Add first houghSegment into laneLines
    laneLines.push_back( {Point(lines[0][0], lines[0][1]),
                Point(lines[0][2], lines[0][3])} );

    // Traverse through the rest of houghSegments to add them to laneLines
    for (size_t indexOfHough = 1; indexOfHough < lines.size(); indexOfHough++) {
        vector<Point> currentSegment = { Point(lines[indexOfHough][0], lines[indexOfHough][1]),
                Point(lines[indexOfHough][2], lines[indexOfHough][3]) };
        
        bool segmentAdded = false; // Check if a new lane needs added
        
        // Traverse through the lanes in laneLines
        for (size_t laneNumber = 0; laneNumber < laneLines.size(); laneNumber++) {
            
            // Traverse through all points in lane
            for (size_t lanePoint = 0; lanePoint < laneLines[laneNumber].size(); lanePoint++){ 
                Point pointInLane = laneLines[laneNumber][lanePoint];
                // Check if currentSegment belongs in this lane
                if ( ( (pointInLane.x + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[0].x && pointInLane.x - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[0].x)
                    && (pointInLane.y + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[0].y && pointInLane.y - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[0].y) )
                    || ( (pointInLane.x + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[1].x && pointInLane.x - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[1].x)
                    && (pointInLane.y + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[1].y && pointInLane.y - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[1].y) )
                ){
                    // Add currentSegment points to this lane
                    laneLines[laneNumber].push_back(currentSegment[0]);
                    laneLines[laneNumber].push_back(currentSegment[1]);
                    segmentAdded = true;
                    break; // out of laneLines/LaneNumber loop back to hough loop
                }
            }

            if (segmentAdded){
                break;
            }
            
        }

        if (!segmentAdded) {
            laneLines.push_back( {Point(lines[indexOfHough][0], lines[indexOfHough][1]),
                Point(lines[indexOfHough][2], lines[indexOfHough][3])} );
        }

    }

    // sort each lane's points by their y coordinate descending
    for (size_t i = 0; i < laneLines.size(); i ++) {
        sort(laneLines[i].begin(), laneLines[i].end(), comparePointY);
    }

    // sort each lane by their first point's x coordinate ascending to separate left, right, and center lanes
    sort(laneLines.begin(), laneLines.end(), compareXCoordinatesOfLanes);

    // Deallocate Memory
    lines.clear();

    return laneLines;
}

/**
 * @param laneLines from LaneLinesFromSegments()
 * @return
*/
vector<vector<Point>> getSolidLanes(vector<vector<Point>> laneLines) {
    vector<vector<Point>> solidlanes;
    for (size_t i = 0; i < laneLines.size(); i++) {
        if (laneLines[i].front().y - laneLines[i].back().y > SEP_DISTANCE_BETWEEN_LANES_Y){
            solidlanes.push_back({laneLines[i]});
        }
    }

    // Deallocate Memory
    laneLines.clear();

    return solidlanes;
}

/**
 * could proabbly combine this with getSolidLanes
*/
vector<vector<Point>> getDashedLanes(vector<vector<Point>> laneLines, vector<vector<Point>> solidLanes) {
    // add empty vector to end of laneLines to use as a check if solid was found later
    laneLines.push_back({});
    for (size_t i = 0; i < solidLanes.size(); i++){
        auto found = find(laneLines.begin(), laneLines.end(), solidLanes[i]);
        if (found != laneLines.end()) {
            // remove solid line from laneLine
            laneLines.erase(found);
        }
    }

    // Deallocate Memory
    solidLanes.clear();

    return laneLines;
}

/**
 * TODO: update to 
 * 
 * Option: Use higher inclusive gapHoughTransform to compare with lower inclusive hough to find the full contents of a dashLine
 *      Using this hough normally adds unwanted lines
 * 
 * Option: Consider that there is one dashed line available
 *      XXX NOPE makes straight cut lines
 * 
 * Option: Use curvature of solid lanes to approximate where the dashed lane should be and if dashes are
 *      close to that approximated line then combine them into one line.
 * 
 * Option: Find slope betwwen dashes and if points fall along line created by slope combine them...
 *      XXX SLope idea will not work on curves
 * 
*/
vector<vector<Point>> combineDashedLanes(vector<vector<Point>> dashSegments, vector<vector<Point>> fullLines) {
    vector<vector<Point>> dashLines;
    
    if (!dashSegments.empty()) {
        // find vector within fullLines that contains all points in vector of dashSegments

        
    }

    // Deallocate Memory
    dashSegments.clear();
    fullLines.clear();

    return dashLines;
}



#endif