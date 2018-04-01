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
#include <string>
#include <sstream>
#ifdef __linux__ 
#include <getopt.h>
#else
#include "getopt.h"
#endif

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
#include "imageprocessor/Line.h"

using namespace std;
using namespace cv;
using namespace Autodrive;
//using linef = Autodrive::Line<float>;

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
//! Assumptions:
//!  - input to Autodrive C++ code is provided by the Android App.
//!    - interaction with the phone camera is via CameraActivity.java.  On each camera frame, that calls driver.process_image(inputFrame.rgba()) in AutomaticCarDriver.java.  Here is what process_image() does:
//!        Mat resized = new Mat();
//!        Size prevSize = image.size();
//!        Size size = new Size(240,135);
//!        Imgproc.resize(image, resized, size,0,0,Imgproc.INTER_NEAREST);
//!        Autodrive.setImage(resized.getNativeObjAddr());
//!        Autodrive.drive();
//!  - then AutodriveJavaFacade.cpp implements setImage() as "Autodrive::car.image_ = (cv::Mat*)newMat;"
//!  - i.e. the input to the C++ code is a 240x135 colour image
//!
		
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
cv::Mat imCanny, imCannyColor; //output image after cv::canny
cv::Mat imHough; // output of Hough transform
cv::Mat imLane; // output of Lane processing
cv::Mat imBirdseye; // image pre-Birdseye warping
cv::Mat imBirdseyeWarped; // output of Birdseye warping
cv::Mat birdseye_matrix;
cv::Mat imFrameWarped; //input frame, warped
cv::Mat imTest; // just for basic drawing tests

int lowThresh = 80;  // can be changed in GUI slider
int max_lowThresh = 300;
int highThresh;
int max_highThresh = 300;
int hi_lo_ratio = 3;  //recommended ratio used to convert Canny lowThresh to highThresh
int kernel_size = 3;
string window_name_frame = "A frame from the Camera";
string window_name_canny = "Canny Edge Detection Test";
string window_name_hough = "Hough Line Detection Test";
string window_name_lane = "Lane Detection Test";
string window_name_birdseye = "Birdseye Test";
string window_name_birdseye_warped = "Birdseye Warped Test";
string window_name_frame_warped = "Warped Input Frame Normalise Test";
string window_name_road_follower = "Road Follower Test";
string window_name_test = "Drawing Test";
int houghThresh = 20;  // can be changed in GUI slider
int max_houghThresh = 300;
float center_diff_;
std::unique_ptr<RoadFollower> road_follower_;
bool birdseye_done = false;  //only calculate the birdseye matrix once, then stop doing it
long frame_number = 1;  //default go to first frame in video.  Can be overridden on command line using -f
bool init_road_follower_done = false;
linef left_image_border_;
linef right_image_border_;

Autodrive::lanes lane_lines;

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


static void CannyThreshold() {
    //cv::blur(frame, imCanny, Size(3,3));  //Autodrive does not use blur.  Test with/without.
    cv::Canny(frame, imCanny, lowThresh, lowThresh*hi_lo_ratio, kernel_size);
    //cv::Canny(imCanny, imCanny, lowThresh, highThresh, kernel_size);
    imCannyColor = Scalar::all(0); // make the image all black
    frame.copyTo( imCannyColor, imCanny);  // copy frame to "imCannyColor" using mask "imCanny".  i.e. the lines will now be coloured!
    //TODO: print frame number onto screen
    imshow( window_name_canny, imCannyColor ); //Autodrive::show_image makes the image 3 times bigger
}

static void HoughThreshold() {
    vector<cv::Vec4i> lines;
    
    imCannyColor.copyTo(imHough);  //start off with the edge detection color image
    // This transform detects straight "lines" as extremes (x0,y0, x1,y1) in image "imCannyColor"
    // rho=1 pixel; theta=1 degree; threshold=20, minLinLength=10, maxLineGap=50
    //source image must be 8-bit, single-channel 
    cv::HoughLinesP(imCanny, lines, 1, CV_PI / 180, 1+houghThresh, 10, 50);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      // params are: output image, start point, end point, colour(BGR), thickness, linetype
      line( imHough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 2, CV_AA);
    }
    //if (lines.size() > 0) {
      imshow( window_name_hough, imHough ); 
    //}
}

// Based on BirdseyeTransformer::get_lane_markings
// input is imCanny
// output is lanes drawn on the image
static void LaneThreshold() {
    std::vector<cv::Vec4i> lines;
    linef leftMostLine;
    linef rightMostLine;
    
    imCannyColor.copyTo(imLane);  //start off with the edge detection color image
    cv::HoughLinesP(imCanny, lines, 1, CV_PI / 180, 1+houghThresh, 10, 50);  //but only process the GRAYSCALE image
    bool foundLeft = false;
    bool foundRight = false;
    int center = imCanny.size().width / 2;
    
    for(cv::Vec4i one_line : lines) {
        int startx = one_line[0];
        int starty = one_line[1];
        int endx = one_line[2];
        int endy = one_line[3];
        linef one_hough_line(one_line);  //call the Line constructor to convert to a linef
		//! Each Hough Line starts from min x val and ends at max x val (left lane line slopes forward so start is at bottom, right line is opposite)
		// Direction_fixed_half will take the positive direction of a line (range 0 to PI)
        float dirr = one_hough_line.direction_fixed_half();
        float dir_diff = dirr - Autodrive::Direction::FORWARD;
    
        //! Ignore line if it differs from Direction::FORWARD by 1 radian (about 57 degrees)
        if (abs(dir_diff) > 1.f)
            continue;
        //! Draw all remaining candidate lines: image, colour(BGR), thickness
        one_hough_line.draw(imLane, cv::Scalar(0, 255, 255), 1);
        //! Work out whether it is the left or right line
        if ( startx > center + 5) {
                if (endx > startx && endy > starty && one_hough_line.length() > 50) {  // line starting on RHS, and sloping down further right
					if (!foundRight) {
						rightMostLine = one_hough_line;
						foundRight = true;
					}
					else {
						if (one_hough_line.length() > rightMostLine.length()) {
							rightMostLine = one_hough_line; //this line is longer than the one previous found, so keep this one.
						}
					}
                }
        }
        if ( endx < center - 5) { // line ends LHS of center (left line slopes forward with start at bottom, end at top)
			if (startx < endx && starty > endy && one_hough_line.length() > 50) {  //ensure line slopes forward, and is 50 pixels long
				if (!foundLeft) {
					leftMostLine = one_hough_line;
					foundLeft = true;
				}
				else {
					if (one_hough_line.length() > leftMostLine.length()) {
						leftMostLine = one_hough_line; //this line is longer than the one previous found, so keep this one.
					}
				}
			}
        }
    }
    
    if (foundRight && foundLeft) {
    // Draw the likely the lane markers, params=Image,colour(BGR),thckness
        leftMostLine.draw(imLane,cv::Scalar(0,0,255),2);
        rightMostLine.draw(imLane,cv::Scalar(0,255,0),2);
		//! As lines get more vertical, slope becomes a large number.  Hence getting two lines with the same slope +-1 is unlikely.
		//! Instead, use radians.  Compare the absolute direction of the two lines versus FORWARD.  They should be similar.  Threshold is PI/6, approx 50 degrees.
		float dir_diff_left = leftMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. PI/4 - PI/2 = -PI/4
		float dir_diff_right = rightMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. 3PI/4 - PI/2 = PI/4
		cout << "rightMostLine radian direction from FORWARD = " << dir_diff_left << endl;
		cout << "leftMostLine radian direction from FORWARD = " << dir_diff_right << ", requires difference < " << CV_PI / 6 << endl;
		if (abs((dir_diff_left + dir_diff_right)) < (CV_PI / 6)) {
			rightMostLine.stretchY(0.f, (float) imCanny.size().height);
            leftMostLine.stretchY(0.f, (float) imCanny.size().height );
            //! Draw the final chosen lane lines (BGR)
            leftMostLine.draw(imLane, cv::Scalar(0, 0, 255), 5);
            rightMostLine.draw(imLane,cv::Scalar(0, 255, 0), 5);
            lane_lines.left = leftMostLine; 
            lane_lines.right = rightMostLine;
            lane_lines.found = true;
        }
    }
    if (lane_lines.found == false) {
        cv::putText(imLane, "SEARCHING FOR STRAIGHT LANES...", Autodrive::POINT(50.f, imLane.size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
    }
    std::ostringstream oss;
    oss << "Frame" << frame_number;
    //std::string var = oss.str();
    cv::putText(imLane, oss.str(), Autodrive::POINT(30,30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);
    imshow( window_name_lane, imLane ); //Autodrive::show_image makes the image 3 times bigger
}

// Based on BirdseyeTransformer::find_perspective()
// Input = imCanny
void BirdseyeThreshold() {
    imCannyColor.copyTo(imBirdseye);   //start off with the Canny color image, so we can draw colour lines on it
    if (!lane_lines.found) {
            return;
    }
    //take a copy of the current lanes, so we can stretch them without affecting the global variable.
    Autodrive::lanes b_lines = lane_lines;
    
    float icrop = 0.f;
    float xdiff;
    float im_height = (float) imBirdseye.size().height;
    float im_width = (float) imBirdseye.size().width;
    //! Stretch the lines, but if they converge at the top of the image,
    //  keep cropping the top of the lines until they are at least X pixels apart
    do
    {
        xdiff = b_lines.right.leftMost_x() - b_lines.left.rightMost_x();
        b_lines.right.stretchY(icrop, im_height);  //params=bottom(lowest value of y) to "top" (highest value of y)
        b_lines.left.stretchY(icrop, im_height);
        icrop+=3.f;
    } while (xdiff < im_width/3.0f);  //was div by 3, but can increase this to see further into distance

    b_lines.right.draw(imBirdseye, cv::Scalar(255,0,0),3);  //blue lane lines
    b_lines.left.draw(imBirdseye, cv::Scalar(255,0,0),3);
    
    //stretchY ensures the lines start at lowY and end at highY
    //Hence the center point will be half way between either the start point
    //or the end point.  The difference from the image center is then:
    center_diff_ = (abs(b_lines.left.begin.x + b_lines.right.begin.x) / 2.f - im_width /2.f);

    //getPerspectiveTransform requires 4 vertices of a quadrangle in the source image, and their corresponding points in the dest image
    //  - source image: choose the start and end of the lane lines (tried allowing 10 pixels on the outside of the lines, but then the warped lanes are still not straight)
    //  - dest image: choose the size of the image (but give a bit of room to left and right to allow lane line detection)
    cout << "left lane begin" << b_lines.left.begin << endl;
    cout << "left lane end" << b_lines.left.end << endl;
    cout << "right lane begin" << b_lines.right.begin << endl;
    cout << "right lane end" << b_lines.right.end << endl;
    //! In case the lane lines go outside the image (e.g. off the screen to the right), keep shortening the lines until
    //! they all fit inside the image.
//    icrop = 0.f;
//    while ((b_lines.left.end.x < 0) || (b_lines.right.end.x > im_width)) {
//        icrop+=0.1f;
//        b_lines.right.stretchY(b_lines.right.begin.y, im_height-icrop); //params=bottom(lowest value of y) to "top" (highest value of y)
//        b_lines.left.stretchY(b_lines.left.begin.y, im_height-icrop);
//    }
    cout << "new left lane begin" << b_lines.left.begin << endl;
    cout << "new left lane end" << b_lines.left.end << endl;
    cout << "new right lane begin" << b_lines.right.begin << endl;
    cout << "new right lane end" << b_lines.right.end << endl;
    
    //Autodrive::POINT pts1[] = { b_lines.left.begin, b_lines.right.begin, Autodrive::POINT(b_lines.left.end.x, im_height), Autodrive::POINT(b_lines.right.end.x, im_height) };
    Autodrive::POINT pts1[] = { b_lines.left.begin, b_lines.right.begin, b_lines.left.end, b_lines.right.end };
	//Autodrive::POINT pts1[] = { Autodrive::POINT(b_lines.left.begin.x -10, b_lines.left.begin.y), Autodrive::POINT(b_lines.right.begin.x +10, b_lines.right.begin.y), Autodrive::POINT(b_lines.left.end.x -10, b_lines.left.end.y), Autodrive::POINT(b_lines.right.end.x +10, b_lines.right.end.y) };
    // Next two lines tried to stretch the single lane out to the whole screen, however it distorted the image too much, and made the lane lines too fuzzy
	//Autodrive::POINT pts2[] = { Autodrive::POINT(b_lines.left.end.x, 0), Autodrive::POINT(b_lines.right.end.x, 0), b_lines.left.end, b_lines.right.end };
	//Autodrive::POINT pts2[] = { Autodrive::POINT(20,0), Autodrive::POINT(im_width-50,0), Autodrive::POINT(20,im_height), Autodrive::POINT(im_width-50,im_height) };
	// pts2 just ensures the lanes are now made straight, by going from the start of the detected lines (near top of image), then straight down to bottom of image.
	Autodrive::POINT pts2[] = { b_lines.left.begin, b_lines.right.begin, Autodrive::POINT(b_lines.left.begin.x, im_height), Autodrive::POINT(b_lines.right.begin.x, im_height) };

    birdseye_matrix = cv::getPerspectiveTransform(pts1, pts2);
    // This border is the edge of the original image in the warped space (should only plot it in the warped image)
    // The border is always detected by Canny as a line, so we need to know where it is so we can blank the Cannied lines there.
	// The calculation here isnt perfect for some reason, but it is good enough. Have fudged +5 to each x value.  FIX.
	left_image_border_ = linef(Autodrive::POINT(b_lines.left.begin.x - b_lines.left.end.x / 2 +5, b_lines.left.end.y), Autodrive::POINT(0, b_lines.left.begin.y));
    right_image_border_ = linef(Autodrive::POINT(b_lines.right.begin.x - (b_lines.right.end.x - im_width) / 2 +5, b_lines.right.end.y), Autodrive::POINT(im_width, b_lines.right.begin.y));

	cv::warpPerspective(imBirdseye, imBirdseyeWarped, birdseye_matrix, imBirdseye.size(), cv::INTER_LINEAR);
    
    left_image_border_.draw(imBirdseyeWarped, cv::Scalar(0,0,255),1);  //red image border
    right_image_border_.draw(imBirdseyeWarped, cv::Scalar(0,0,255),1);
    
    imshow( window_name_birdseye, imBirdseye );
    imshow( window_name_birdseye_warped, imBirdseyeWarped ); 
    birdseye_done = true;
}

void init_road_follower() {
	// birdseyeTransform
	cv::warpPerspective(frame, imFrameWarped, birdseye_matrix, frame.size(), cv::INTER_LINEAR);
	car.img_proc()->normalize_lighting(&imFrameWarped);  //blur and intensity no longer ysed
	cv::Mat imFrameWarpedCannied;
	cv::Canny(imFrameWarped, imFrameWarpedCannied, lowThresh, lowThresh*hi_lo_ratio, kernel_size);
	int the_center = static_cast<int>(frame.size().width / 2.f + center_diff_);
	road_follower_ = Autodrive::make_unique<Autodrive::RoadFollower>(imFrameWarpedCannied, the_center, car.img_conf());
	imshow(window_name_frame_warped, imFrameWarped);
	init_road_follower_done = true;
}

void continue_road_follower()
{
	imshow(window_name_frame, frame);
	cv::warpPerspective(frame, imFrameWarped, birdseye_matrix, frame.size(), cv::INTER_LINEAR);
	car.img_proc()->normalize_lighting(&imFrameWarped);  //blur and intensity no longer used
	//draw the border of the warped image (to check it is calculated correctly)
	left_image_border_.draw(imFrameWarped, cv::Scalar(0, 0, 255), 2);  //red image border
	right_image_border_.draw(imFrameWarped, cv::Scalar(0, 0, 255), 2);
	imshow(window_name_frame_warped, imFrameWarped);

	cv::Mat imFrameWarpedCannied;
	cv::Canny(imFrameWarped, imFrameWarpedCannied, lowThresh, lowThresh*hi_lo_ratio, kernel_size);
	// PAINT OVER BORDER ARTEFACTS FROM TRANSFORM in black (since canny always detects the border as a line)
	//cout << "center_diff_ is" << center_diff_ << endl;
	left_image_border_.draw(imFrameWarpedCannied, cv::Scalar(0, 0, 0), car.img_conf()->transform_line_removal_threshold_);
	right_image_border_.draw(imFrameWarpedCannied, cv::Scalar(0, 0, 0), car.img_conf()->transform_line_removal_threshold_);
	imshow(window_name_canny, imFrameWarpedCannied);

	//! Key step is to call update on the road_follower
	// ***************** UP TO HERE ************************  need to understand steps in this update method. Particularly does it do a Hough Transform???
	// This uses image "imFrameWarpedCannied" to follow the road, and draws results on the image "frame"
	Autodrive::CarCmd cmnd = road_follower_->update(imFrameWarpedCannied, frame);
	float angle = Autodrive::Direction::FORWARD;

	if (cmnd.changed_angle())
	{
		//TODO: *15 really needed.  Mathf was prepended by Autodrive::
		angle = static_cast<float>(((90.0 - cmnd.angle()*15.0)* Mathf::PI) / 180.f);
	}

	POINT center(frame.size().width / 2.f, (float)frame.size().height);
	//! Draw a short green line from center bottom in direction of the changed angle (line starts at car bonnet)
	linef(center, center + POINT(std::cos(angle) * 100, -sin(angle) * 100)).draw(frame, CV_RGB(0, 250, 0));

	//What are other lines:
	// - pink marks car_y
	// - green vertical parallel lines?
	// - aqua vertical line
	// - blue lane lines
	imshow(window_name_road_follower, frame);
	
}


static void TestDrawing() {
    vector<cv::Vec4i> lines;
    
    imCannyColor.copyTo(imTest);  //start off with the edge detection color image
    // This transform detects straight "lines" as extremes (x0,y0, x1,y1) in image "imCannyColor"
    // rho=1 pixel; theta=1 degree; threshold=20, minLinLength=10, maxLineGap=50
    //source image must be 8-bit, single-channel 
    cv::HoughLinesP(imCanny, lines, 1, CV_PI / 180, 1+houghThresh, 10, 50);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      // params are: output image, start point, end point, colour(BGR), thickness, linetype
      cv::line( imTest, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 2, CV_AA);
      cv::circle(imTest, Point(l[0], l[1]), 3, Scalar(255,0,0));  //mark start of line
    }
    
    int t_width = imTest.size().width;
    int t_height = imTest.size().height;
    cout << "Image dimensions = " << t_width << "x" << t_height << endl;
    //cv::line(imTest, Point(0,0), Point(t_width,t_height), Scalar(255, 0, 0), 5);
    //cv::circle(imTest, Point(5,50), 3, Scalar(255,0,0));
    //cv::line(imTest, Point(40,140), Point(120,40), Scalar(255, 0, 0), 3);
    //cv::line(imTest, Point(200,40), Point(280,140), Scalar(255, 0, 0), 3);
    linef leftLine(Point(40,140), Point(120,40));
    linef rightLine(Point(200,40), Point(280,140));
    rightLine.draw(imTest, cv::Scalar(255,0,0),3);
    leftLine.draw(imTest, cv::Scalar(255,0,0),3);
    
    // Test Line.stretchY().  Yep, it works.  Lines are stretched to top and bottom of image
    rightLine.stretchY(0.f, (float) imTest.size().height);
    leftLine.stretchY(0.f, (float) imTest.size().height );
    rightLine.draw(imTest, cv::Scalar(0,0,255),1);
    leftLine.draw(imTest, cv::Scalar(0,0,255),1);
    imshow( window_name_test, imTest ); 
}

static void updateThreshold(int, void*) {
	if (!init_road_follower_done) {
		CannyThreshold();
		HoughThreshold();
		if (!birdseye_done) {
			LaneThreshold();
			BirdseyeThreshold();
		}
		else {
			init_road_follower();
		}
	}
	else {
		//Problem with this is all the windows no longer update
		// Does this do a Hough transform????
		continue_road_follower();
	}
    //TestDrawing();
}

int main(int argc, char** argv) {
    cv::Size* resolution = nullptr;
    
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
        frame_number = atol(optarg);
        break;
      case 'r':
        {
		  char dummych[1];
          
          resolution = new cv::Size();
#ifdef __linux__
          if (sscanf(optarg, "%d%c%d", &resolution->width, &dummych, &resolution->height) != 3) {
#else
		  if (sscanf_s(optarg, "%d%1s%d", &resolution->width, dummych, (unsigned)sizeof(dummych), &resolution->height) != 3) {
#endif
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
    
    
    
    namedWindow(window_name_canny, WINDOW_AUTOSIZE);
    namedWindow(window_name_hough, WINDOW_AUTOSIZE);
	namedWindow(window_name_frame_warped, WINDOW_AUTOSIZE);
    
    for (long i = 0; i < frame_number; i++) {
        capture >> frame;  //fast forward to frame of interest
    }

    // Do same image manipulation prior to Canny as per Autodrive code
    // i.e. set image size, birdseye transform, normalize.
    createTrackbar( "Min Threshold:", window_name_canny, &lowThresh, max_lowThresh, updateThreshold );
    createTrackbar( "Hough Threshold:", window_name_hough, &houghThresh, max_houghThresh, updateThreshold );
	
    //Next line no longer required, since the highThresh is set automatically using hi_lo_ratio
    //createTrackbar( "Max Threshold:", window_name, &highThresh, max_highThresh, CannyThreshold );
    imCannyColor.create( frame.size(), frame.type() );  //create a new Mat image of same size as the input frame
    imHough.create( frame.size(), frame.type() ); 
    imLane.create( frame.size(), frame.type() );
    imBirdseye.create( frame.size(), frame.type() );
    imBirdseyeWarped.create( frame.size(), frame.type() ); 
	imFrameWarped.create(frame.size(), frame.type());
    imTest.create( frame.size(), frame.type() ); 
    
    while (!frame.empty())
    {
        // Show the output of the cv::canny and other functions on the input frame 
        updateThreshold(0,0);
        //show_image(frame, 3, "w");  // display the raw video frame
        waitKey(0); // waits until a keystroke
        capture >> frame;
        frame_number++;
    }
    return 0;
}
