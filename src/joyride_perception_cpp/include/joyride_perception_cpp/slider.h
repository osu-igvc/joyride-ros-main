// g++ -I /usr/local/include/opencv4 slider.cpp submodules/lane_helperFunctions.o submodules/lane_mainFunctions.o -o slider `pkg-config --libs opencv4`

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>

#include "lane_mainFunctions.h"
#include "lane_helperFunctions.h"

using namespace cv;
using namespace std;


const int alpha_slider_max = 300;
char window_Slider[] = "Adjust Lane Detection Thresholds";
string subfolder = "presavedVariables/";

int key = 0;

// Slider values
int sliderVal_White_Threshold;
int sliderVal_Min_White_Region_Size;
int sliderVal_Hough_Threshold;
int sliderVal_Hough_Min_Line_Length;
int sliderVal_Hough_Max_Line_Gap;
int sliderVal_Sep_Distance_Between_Segments;
int sliderVal_Sep_Distance_Between_Lanes_Y;
int sliderVal_Sep_Distance_Between_Dash_X;

int sliderVal_hsv_hlb;
int sliderVal_hsv_slb;
int sliderVal_hsv_vlb;
int sliderVal_hsv_hup;
int sliderVal_hsv_sup;
int sliderVal_hsv_vup;

// initialize name of trackbars
char track_WHITE_THRESHOLD[] =                  "White Threshold";
char track_Min_White_Region_Size[] =            "Minimum White Regions";
char track_Hough_Threshold[] =                  "Hough: Threshold";
char track_Hough_Min_Line_Length[] =            "Hough: Min Length of Line";
char track_Hough_Max_Line_Gap[] =               "Hough: Max Line Gap";
char track_Sep_Distance_Between_Segments[] =    "Lane Division: Gap considered Same Lane";
char track_Sep_Distance_Between_Lanes_Y[] =     "Lane Division: Y Gap for different Lanes";
char track_Sep_Distance_Between_Dash_X[] =      "Lane Division: X Gap for Dash Lines";

char track_END_TRACKER[] =                      "Apply To Frame";

char track_hsv_hlb[] =                          "H Low Bound";
char track_hsv_slb[] =                          "S Low Bound";
char track_hsv_vlb[] =                          "V Low Bound";
char track_hsv_hup[] =                          "H High Bound";
char track_hsv_sup[] =                          "S High Bound";
char track_hsv_vup[] =                          "V High Bound";

void initSliderVariables () {
    sliderVal_White_Threshold = WHITE_THRESHOLD;
    sliderVal_Min_White_Region_Size = MIN_WHITE_REGION_SIZE;
    sliderVal_Hough_Threshold = HOUGH_THRESHOLD;
    sliderVal_Hough_Min_Line_Length = HOUGH_MIN_LINE_LENGTH;
    sliderVal_Hough_Max_Line_Gap = HOUGH_MAX_LINE_GAP_SOLID;
    sliderVal_Sep_Distance_Between_Segments = SEP_DISTANCE_BETWEEN_SEGMENTS;
    sliderVal_Sep_Distance_Between_Lanes_Y = SEP_DISTANCE_BETWEEN_LANES_Y;
    sliderVal_Sep_Distance_Between_Dash_X = SEP_DISTANCE_BETWEEN_DASH_X;

    sliderVal_hsv_hlb = hlb;
    sliderVal_hsv_slb = slb;
    sliderVal_hsv_vlb = vlb;
    sliderVal_hsv_hup = hup;
    sliderVal_hsv_sup = sup;
    sliderVal_hsv_vup = vup;

}

void saveVariablesToFile(string filenamecontext){
    string filename = subfolder + filenamecontext + ".txt";

    ifstream infile(filename);
    if (infile.good()){
        // File already exists
        infile.close();
        cout << "FileName already taken please try again: ";
        string filenamecontext;
        getline(cin, filenamecontext);
        saveVariablesToFile(filenamecontext);

    } else {
        // Save variables to a file
        ofstream file_out(filename);
        if (file_out.is_open()){
            cout << "Saving Variables ..." << endl;
            // file_out << sliderVal_White_Threshold << "\n" << sliderVal_Min_White_Region_Size << "\n" 
            //         << sliderVal_Hough_Threshold << "\n" << sliderVal_Hough_Min_Line_Length << "\n" 
            //         << sliderVal_Hough_Max_Line_Gap << "\n" << sliderVal_Sep_Distance_Between_Segments << "\n"
            //         << sliderVal_Sep_Distance_Between_Lanes_Y << "\n" << sliderVal_Sep_Distance_Between_Dash_X << "\n";
            file_out << sliderVal_White_Threshold << "\n" << sliderVal_Min_White_Region_Size << "\n" 
                    << sliderVal_hsv_hlb << "\n" << sliderVal_hsv_slb << "\n" << sliderVal_hsv_vlb << "\n"
                    << sliderVal_hsv_hup << "\n" << sliderVal_hsv_sup << "\n" << sliderVal_hsv_vup << "\n"
                    << sliderVal_Hough_Threshold << "\n" << sliderVal_Hough_Min_Line_Length << "\n" 
                    << sliderVal_Hough_Max_Line_Gap << "\n" << sliderVal_Sep_Distance_Between_Segments << "\n"
                    << sliderVal_Sep_Distance_Between_Lanes_Y << "\n" << sliderVal_Sep_Distance_Between_Dash_X << "\n";
            file_out.close();
            cout << "Variables successfully saved" << endl;

        } else {
            cout << "Error saving Variables" << endl;
        }
    }

    
}

