/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/

// system includes
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>


// library includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define CVVISUAL_DEBUGMODE
//#include <opencv2/cvv/debug_mode.hpp>
//#include <opencv2/cvv/show_image.hpp>
//#include <opencv2/cvv/filter.hpp>
//#include <opencv2/cvv/dmatch.hpp>
//#include <opencv2/cvv/final_show.hpp>


#define _AUTODRIVE_SHOWCANNY
#define _AUTODRIVE_SHOWHOUGH

#undef _DEBUG
// application inlcudes
#include "Autodrive.h"
#include "imageprocessor/ImageProcessor.h"

using namespace cv;
using namespace std;

//! Visualise the effect of each opencv image manipulation method.
//! i.e. this test program aims to improve understanding of how each
//! ImageConfig parameter will effect Autodrive.
//!
//! Steps are:
//! 1) Fast forward video to frame number X, where X is provided on command line
//! 2) display output image from:
//!   a) (DONE) cv::canny
//!   b) cv::houghp
//!   c) cv: .... color???
//!   d) Overlay of lines
//!   e) coordinates, e.g. (0,0) point.
//!   f) etc.
//!   g) print on screen param values while cycling through the same image transformed with different param values.
//!   h) display lines, lanes, roadline etc. to understand those different objects.
//! 3) Create another program which is a GUI for Autodrive where I can change the params using GUI sliders and see the effect on the image and on Autodrive functionality.

using std::vector;
using std::to_string;
template<class T> std::string toString(const T& p_arg)
{
  std::stringstream ss;

  ss << p_arg;

  return ss.str();
}

//Globals
cv::Mat frame, frameGray;  //input frame image, and its greyscale
cv::Mat manip, manipEdges; //output image after manipulation, e.g. cv::canny
int lowThresh;
int max_lowThresh = 300;
int highThresh;
int max_highThresh = 300;
int hi_lo_ratio = 3;  //recommended ratio used to convert Canny lowThresh to highThresh
int kernel_size = 3;
string window_name = "AutodriveTest";

void usage() {
  printf("usage: test_image_manip [-f frame_number] [-r WxH]\n");
  printf("-h                print this help\n");
  printf("-f frame_number   fast forward video to given frame number\n");
  printf("-r WxH            change resolution to width W and height H\n");
}


void displayWindows(vector<Mat>& frames) {
    // iterate through the frames
    for(int index = 0; index < frames.size(); ++index) {
        // index allows us to assign a unique name to each window
        // windows will be titled 'Window 0', 'Window 1', 'Window 2'... until frames.size()-1
        String windowTitle = "Window " + to_string(index);

        imshow(windowTitle, frames[index]);
    }
}

static void CannyThreshold(int, void*)
{
    cv::blur(frameGray, manipEdges, Size(3,3));
    cv::Canny(manipEdges, manipEdges, lowThresh, lowThresh*hi_lo_ratio, kernel_size);
    //cv::Canny(manipEdges, manipEdges, lowThresh, highThresh, kernel_size);
    manip = Scalar::all(0);
    frame.copyTo( manip, manipEdges);
    //TODO: print frame number onto screen
    imshow( window_name, manip ); //Autodrive::show_image makes the image 3 times bigger
}




int main(int argc, char** argv) {
    cv::Size* resolution = nullptr;
    int frame_number = 1;  // can override using command line param f
    
    // parse options
    const char* optstring = "hf:r:";
    int opt;
    while ((opt = getopt(argc, argv, optstring)) != -1) {
      switch (opt) {
      case 'h':
        usage();
        return 0;
        break;
      case 'f':
        frame_number = atoi(optarg);
        break;
      case 'r':
        {
          char dummych;
          resolution = new cv::Size();
          if (sscanf(optarg, "%d%c%d", &resolution->width, &dummych, &resolution->height) != 3) {
            printf("%s not a valid resolution\n", optarg);
            return 1;
          }
        }
        break;
      default: /* '?' */
        usage();
        return 2;
      }
    }
    
    
    std::cout<<"main";
    string filename = "testreal_small.mp4";
    cv::VideoCapture capture(filename);
    if (!capture.isOpened()) {
        throw "Error when opening test4.avi";
    }
    
    
    
    namedWindow(window_name, WINDOW_AUTOSIZE);

    for (int i = 0; i < frame_number; i++){
        capture >> frame;  //fast forward to frame of interest
    }

    //**************** UP TO HERE *******************
    //TODO: does Autodrive do any image manipulation prior to Canny?
    // e.g. is the image size set, or greyscale etc??
    //  This code does greyscale and blur.  Is that done in Autodrive???
    createTrackbar( "Min Threshold:", window_name, &lowThresh, max_lowThresh, CannyThreshold );
    //Next line no longer required, since the highThresh is set automatically using hi_lo_ratio
    //createTrackbar( "Max Threshold:", window_name, &highThresh, max_highThresh, CannyThreshold );
    manip.create( frame.size(), frame.type() );

    while (!frame.empty())
    {
        vector<Mat> images;  //empty vector
        string frameString{"frame"};
        frameString += toString(frame_number);
            //cvv::showImage(frame, CVVISUAL_LOCATION, frameString.c_str());
    
        // convert to grayscale
        cv::cvtColor(frame, frameGray, CV_BGR2GRAY);
            //cvv::debugFilter(frame, frameGray, CVVISUAL_LOCATION, "to gray");
        
        // Show the output of the cv::canny function using the input frame 
        CannyThreshold(0,0);
        //show_image(frame, 3, "w");  // display the raw video frame
        
        //Might be needed on track
        //cv::erode(matCopy, matCopy, cv::Mat(), cv::Point(-1, -1), 1);


        //Autodrive::car.img_proc()->continue_processing(frame);

        
        waitKey(0); // waits to display frame
        capture >> frame;
        frame_number++;
    }
    return 0;
}
