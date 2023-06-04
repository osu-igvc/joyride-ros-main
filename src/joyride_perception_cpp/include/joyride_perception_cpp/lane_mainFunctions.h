#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#ifndef laneDetection
#define laneDetection

// -------------------- GLOBAL VARIABLES --------------------
/* Some variables are connected to the lane_slider_node for dynamic adjustments */

int HEIGHT = 0;
int WIDTH = 0;
/* Defualt to whole frame -> Can use lane_roi_node to get ROI you want and change hardcoded data */
Rect ROI; 

// Lane Color Detection
int WHITE_THRESHOLD = 230;
/* Min area of white pixels to be detected*/
int MIN_WHITE_REGION_SIZE = 50;

// HSV White
int HUE_LOW = 0;
int SAT_LOW = 0;
int VAL_LOW = 220;
int HUE_HIGH = 180;
int SAT_HIGH = 30;
int VAL_HIGH = 255;

// Hough Transfrom
int HOUGH_THRESHOLD = 35;
static double HOUGH_THETA = CV_PI/180;
int HOUGH_MIN_LINE_LENGTH = 15;
int HOUGH_MAX_LINE_GAP = 75;
// Lane Separation
/* Distance used to determine what segments are of the same line */
int SEP_DISTANCE_BETWEEN_SEGMENTS = 38;
/* Minimum y length a lane can be to be considered Solid */
int SEP_DISTANCE_BETWEEN_LANES_Y = 100;

struct Lane {
    vector<Point> points;
    bool isSolid;
};

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



// -------------------- COMPARE FUNCTIONS --------------------
/**
 * Compare the first x coordinates of Lane() structs
 * @return: true if a's 2nd index is greater than b's
*/
bool compareByFirstX(const Lane& lane1, const Lane& lane2){
    // Check if both containers have at least one point
    if (!lane1.points.empty() && !lane2.points.empty()) {
        // Compare the first x-coordinate of the points
        return lane1.points[0].x < lane2.points[0].x;
    }
    // If one of the containers is empty, put it at the end
    return lane1.points.empty();
}

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



// -------------------- LANE DETECTION --------------------

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

Mat getWhiteInFrameHSV(Mat frame){
    Mat hsvImg;
    cvtColor(frame, hsvImg, COLOR_BGR2HSV);

    Scalar lowerWhite = Scalar(HUE_LOW, SAT_LOW, VAL_LOW);
    Scalar upperWhite = Scalar(HUE_HIGH, SAT_HIGH, VAL_HIGH);

    Mat whiteMask;
    inRange(hsvImg, lowerWhite, upperWhite, whiteMask);

    return whiteMask;
}

/**
 * @param frame from camera
 * @return Mat of the edges from the isolated lane lines
*/
Mat getLaneEdgesInFrame(Mat frame){
    
    Canny(frame, frame, 100, 200); // 50,150?? no difference to 100,200

    return frame;
}

/**
 * @param Mat frame of edges
 * @return vector<Vec4i> of hough line segemnts in y_1 descending order
*/
vector<Vec4i> HoughTransformOnEdges(Mat frame) {
    // ------------HoughLinesP--------------------------------------------
    std::vector<Vec4i> lines; // (x_1, y_1, x_2, y_2) Where '_2' is the end point
    HoughLinesP(frame, lines, 1.75, HOUGH_THETA, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

    // ------------Remove out of bounds Y---------------------------------
    /* Distance from top of screen we want to stop detecting lanes */
    int HORIZON_MAX = 200;
    lines.erase(remove_if(lines.begin(), lines.end(), [HORIZON_MAX](const Vec4i& vec){
        return vec[1] < HORIZON_MAX || vec[3] < HORIZON_MAX; }), lines.end()); 

    // ------------Sort HoughLines----------------------------------------
    // y_1 is sorted in descending order
    if (!lines.empty()){
        sort(lines.begin(), lines.end(), compareSecondIndex);
    }

    // Deallocate Memory
    frame.release();

    return lines;
}



// -------------------- LANE PROCESSING --------------------

bool determine_lane_type(Lane lane) {
    return (lane.points[lane.points.size()-1].y - lane.points[0].y > SEP_DISTANCE_BETWEEN_LANES_Y) ? true : false;
}

vector<Lane> LaneLinesFromSegments(vector<Vec4i> lines) {
    vector<Lane> laneLines;

    // Add first houghSegment into laneLines
    laneLines.push_back( {{Point(lines[0][0], lines[0][1]),
                Point(lines[0][2], lines[0][3])}, false} );

    // Traverse through the rest of houghSegments to add them to laneLines
    for (size_t indexOfHough = 1; indexOfHough < lines.size(); indexOfHough++) {
        vector<Point> currentSegment = { Point(lines[indexOfHough][0], lines[indexOfHough][1]),
                Point(lines[indexOfHough][2], lines[indexOfHough][3]) };
        
        bool segmentAdded = false; // Check if a new lane needs added
        
        // Traverse through the lanes in laneLines
        for (size_t laneNumber = 0; laneNumber < laneLines.size(); laneNumber++) {
            
            // Traverse through all points in lane
            for (size_t lanePoint = 0; lanePoint < laneLines[laneNumber].points.size(); lanePoint++){ 
                Point pointInLane = laneLines[laneNumber].points[lanePoint];
                // Check if currentSegment belongs in this lane
                if ( ( (pointInLane.x + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[0].x && pointInLane.x - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[0].x)
                    && (pointInLane.y + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[0].y && pointInLane.y - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[0].y) )
                    || ( (pointInLane.x + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[1].x && pointInLane.x - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[1].x)
                    && (pointInLane.y + SEP_DISTANCE_BETWEEN_SEGMENTS > currentSegment[1].y && pointInLane.y - SEP_DISTANCE_BETWEEN_SEGMENTS < currentSegment[1].y) )
                ){
                    // Add currentSegment points to this lane
                    laneLines[laneNumber].points.push_back(currentSegment[0]);
                    laneLines[laneNumber].points.push_back(currentSegment[1]);
                    segmentAdded = true;
                    break; // break out of laneLines/LaneNumber loop back to hough loop
                }
            }
            if (segmentAdded){
                break;
            }
            
        }

        if (!segmentAdded) {
            laneLines.push_back( {{Point(lines[indexOfHough][0], lines[indexOfHough][1]),
                Point(lines[indexOfHough][2], lines[indexOfHough][3])}, false} );
        }

    }

    // Traverse through each line
    for (size_t i = 0; i < laneLines.size(); i ++) {
        // Sort each Lane's points by their y coordinate descending
        sort(laneLines[i].points.begin(), laneLines[i].points.end(), comparePointY);
        // Determine whether each lane is solid or dashed
        laneLines[i].isSolid = determine_lane_type(laneLines[i]);
    }

    // sort each lane by their first point's x coordinate ascending to separate left, right, and center lanes
    sort(laneLines.begin(), laneLines.end(), compareByFirstX);

    // Deallocate Memory
    lines.clear();

    return laneLines;
}

/**
 * For using velocity controller gain to stay in center of line
*/
int calculateDistanceFromCenter(vector<Lane> laneLines) {
    /* Distance from bottom of screen we want to measure lanes at */
    int target_y = HEIGHT - 100; // TODO: measure
    /* how far away from lane we want to be when we only see one */
    int distance_from_lane = 100; // TODO: calculate/measure 
    int center_img = WIDTH/2;

    /* Return value: Pos-> car too far right    Neg-> car too far left */
    int distance_from_center = 0;
    // Get points along target_y that are lanes
    vector<int> xs_at_target_y;
    for (const auto& lane : laneLines){
        for (const auto& point : lane.points) {
            if (point.y == target_y){
                xs_at_target_y.push_back(point.x);
            }
        }
    }

    // Calculate Distance car is from Lanes
    if (xs_at_target_y.size() > 1)  { // At least two lanes
        // get two points closes to WIDTH/2 (center of image) (lane lines)
        int close_low = 0;
        int close_high = WIDTH;
        for (const auto&x : xs_at_target_y) {
            if (x <= center_img && x > close_low) {
                close_low = x;
            } else if (x >= center_img && x < close_high) {
                close_high = x;
            }
        }
        // compare lane_center to frame_center: pos if to the right, neg if to left
        distance_from_center = ((close_low + close_high) / 2) - center_img;
    } else if (xs_at_target_y.size() == 1) { // one lane available
        if (xs_at_target_y[0] > WIDTH/2){ // We see the right lane
            distance_from_center = xs_at_target_y[0] - distance_from_lane - center_img;
        } else { // We see left lane
            distance_from_center = xs_at_target_y[0] + distance_from_lane - center_img;
        }  
    }

    return distance_from_center;
}

// -------------------- OVERLAYS --------------------

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
Mat overlayLanes(Mat frame, vector<Lane> lanes, const Scalar& color = Scalar(0,255,0)) {
    Mat overlay = frame.clone();
    for (size_t i = 0; i < lanes.size(); i++) {
        for (size_t j = 1; j < lanes[i].points.size(); j++) {
            line(overlay, lanes[i].points[j-1], lanes[i].points[j], color, 3, LINE_AA);
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
Mat overlayLanes_binary(Mat frame, vector<Lane> lanes){
    Mat black_white_lanes(frame.size(), CV_8UC3, Scalar(0,0,0));
    black_white_lanes = overlayLanes(black_white_lanes, lanes, Scalar(255,255,255));
    cvtColor(black_white_lanes, black_white_lanes, COLOR_BGR2GRAY);

    // Deallocate Memory
    lanes.clear();

    return black_white_lanes;
}

/**
 * Separates solid and dashed lines and puts them on separate black and white mat's
 * @param frame of camera image
 * @param lanes sorted line segments that represent the lane lines
 * @return black and white image of the lines [0]: Solid [1]: Dashed
*/
pair<Mat, Mat> overlayLanes_binary_separated(Mat frame, vector<Lane> lanes){
    Mat solid_BW(frame.size(), CV_8UC3, Scalar(0,0,0));
    Mat dash_BW(frame.size(), CV_8UC3, Scalar(0,0,0));

    for (size_t i = 0; i < lanes.size(); i++) {
        for (size_t j = 1; j < lanes[i].points.size(); j++) {
            if (lanes[i].isSolid){
                line(solid_BW, lanes[i].points[j-1], lanes[i].points[j], Scalar(255,255,255), 3, LINE_AA);
            } else{
                line(dash_BW, lanes[i].points[j-1], lanes[i].points[j], Scalar(255,255,255), 3, LINE_AA);
            }

        }
    }
    cvtColor(solid_BW, solid_BW, COLOR_BGR2GRAY);
    cvtColor(dash_BW, dash_BW, COLOR_BGR2GRAY);

    // Deallocate Memory
    lanes.clear();

    return make_pair(solid_BW, dash_BW);
}

#endif