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
#include <iostream>

#include "BirdseyeTransformer.h"
using namespace Autodrive;
using namespace std;

void BirdseyeTransformer::birds_eye_transform(cv::Mat* mat, cv::Mat birdseye_matrix)
{
	//birdseye_matrix comes from "find_perspective()"
	cv::warpPerspective(*mat, *mat, birdseye_matrix, mat->size(), cv::INTER_LINEAR);
}

//! Canny the input image
//! get_lane_markings for leftLine and rightLine
//! stretch the lines
//! Then get birdseye perspective
optional<cv::Mat> BirdseyeTransformer::find_perspective(cv::Mat* matIn, double thresh1, double thresh2) {
	optional<cv::Mat> birdseye_matrix;
	//Might be needed on track
	//cv::erode(matCopy, matCopy, cv::Mat(), cv::Point(-1, -1), 1);
	cv::Mat cannied;
	cv::Canny(*matIn, cannied, thresh1, thresh2, 3);
	calc_lane_markings(cannied, matIn);
	if (lane_markings_.found == false)
		return nullptr;
		
	//take a copy of the current lanes, so we can stretch them without affecting the class member
    Autodrive::lanes b_lines = lane_markings_;
    float icrop = 0.f;
    float xdiff;
    float im_height = (float) matIn->size().height;
    float im_width = (float) matIn->size().width;
    //! Stretch the lines, but if they converge at the top of the image,
    //  keep cropping the top of the lines until they are at least X pixels apart
    do
    {
        xdiff = b_lines.right.leftMost_x() - b_lines.left.rightMost_x();
        b_lines.right.stretchY(icrop, im_height);  //params=bottom(lowest value of y) to "top" (highest value of y)
        b_lines.left.stretchY(icrop, im_height);
        icrop+=3.f;
    } while (xdiff < im_width/3.0f);  //was div by 3, but can increase this to see further into distance

    b_lines.right.draw(*matIn, cv::Scalar(255,0,0),3);  //blue lane lines
    b_lines.left.draw(*matIn, cv::Scalar(255,0,0),3);
    
    //stretchY ensures the lines start at lowY and end at highY
    //Hence the center point will be half way between either the start points
    //or the end points.  Chosen difference from the image center is then:
    center_diff_ = (abs(b_lines.left.begin.x + b_lines.right.begin.x) / 2.f - im_width /2.f);

    //getPerspectiveTransform requires 4 vertices of a quadrangle in the source image, and their corresponding points in the dest image
    //  - source image: choose the start and end of the lane lines (tried allowing 10 pixels on the outside of the lines, but then the warped lanes are still not straight)
    //  - dest image: choose the size of the image (but give a bit of room to left and right to allow lane line detection)
    //cout << "left lane begin" << b_lines.left.begin << endl;
    //cout << "left lane end" << b_lines.left.end << endl;
    //cout << "right lane begin" << b_lines.right.begin << endl;
    //cout << "right lane end" << b_lines.right.end << endl;
    //! In case the lane lines go outside the image (e.g. off the screen to the right), keep shortening the lines until
    //! they all fit inside the image.
//    icrop = 0.f;
//    while ((b_lines.left.end.x < 0) || (b_lines.right.end.x > im_width)) {
//        icrop+=0.1f;
//        b_lines.right.stretchY(b_lines.right.begin.y, im_height-icrop); //params=bottom(lowest value of y) to "top" (highest value of y)
//        b_lines.left.stretchY(b_lines.left.begin.y, im_height-icrop);
//    }
    //cout << "new left lane begin" << b_lines.left.begin << endl;
    //cout << "new left lane end" << b_lines.left.end << endl;
    //cout << "new right lane begin" << b_lines.right.begin << endl;
    //cout << "new right lane end" << b_lines.right.end << endl;
    
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

	return birdseye_matrix;
}

void BirdseyeTransformer::calc_lane_markings(const cv::Mat& canniedMat,cv::Mat* drawMat) {
	std::vector<cv::Vec4i> lines;
	linef leftMostLine;
	linef rightMostLine;
	// This transform detects straight "lines" by their endpoints (x0,y0, x1,y1) in image "canniedMat"
	// rho=1 pixel; theta=1 degree; threshold=20, minLinLength=10, maxLineGap=50
	// source image must be 8-bit, single-channel 
	//! Each Hough Line starts from min x val and ends at max x val (left lane line slopes forward so start is at bottom, right line is opposite)
	cv::HoughLinesP(canniedMat, lines, 1, CV_PI / 180, 21, 10, 50);
	bool foundLeft = false;
	bool foundRight = false;
	int center = canniedMat.size().width / 2;
	cv::imshow("mytestcannied", canniedMat);

	for(cv::Vec4i one_line : lines) {
		int startx = one_line[0];
		int starty = one_line[1];
		int endx = one_line[2];
		int endy = one_line[3];
		linef one_hough_line(one_line);  //call the Line constructor to convert to a linef

		float dirr = one_hough_line.direction_fixed_half();
		float dir_diff = dirr - Autodrive::Direction::FORWARD;

        //! Ignore line if it differs from Direction::FORWARD by 1 radian (90 degrees about 1.6 radian)
        // Since this function is only used during initialisation, can be quite stringent
		if (abs(dir_diff) > 1.0f)
			continue;
        //! Draw all remaining candidate lines: image, colour(BGR), thickness
		one_hough_line.draw(*drawMat, cv::Scalar(0, 255, 255), 1);
		
		//! Work out whether it is the left or right line
		if (startx > center + 5) {
		    //!line starting on RHS, and sloping down further right
			//! check line starts on RHS of center, and sloping down further right, and is long enough
	        if (endx > startx && endy > starty && one_hough_line.length() > 50) {  		
				if (!foundRight) {
					rightMostLine = one_hough_line;
					foundRight = true;
				} else {
					if (one_hough_line.length() > rightMostLine.length()) {
						rightMostLine = one_hough_line; //this line is longer than the one previous found, so keep this one.
					}
				}
			}
		}
		if (endx < center - 5) { // line ends LHS of center (left line slopes forward with start at bottom, end at top)
			if (startx < endx && starty > endy && one_hough_line.length() > 50) {  //ensure line slopes forward, and is 50 pixels long
				if (!foundLeft) {
					leftMostLine = one_hough_line;
					foundLeft = true;
				} else {
					if (one_hough_line.length() > leftMostLine.length()) {
						leftMostLine = one_hough_line; //this line is longer than the one previous found, so keep this one.
					}
				}
			}
		}
	}
	if (foundRight && foundLeft) {
    // Draw the likely the lane markers, params=Image,colour(BGR),thckness
		leftMostLine.draw(*drawMat,cv::Scalar(0,0,255),2);
		rightMostLine.draw(*drawMat,cv::Scalar(0,255,0),2);
		//! As lines get more vertical, slope becomes a large number.  Hence getting two lines with the same slope +-1 is unlikely.
		//! Instead, use radians.  Compare the absolute direction of the two lines versus FORWARD.  They should be similar.  Threshold is PI/6, approx 50 degrees.
		float dir_diff_left = leftMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. PI/4 - PI/2 = -PI/4
		float dir_diff_right = rightMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. 3PI/4 - PI/2 = PI/4
		cout << "rightMostLine radian direction from FORWARD = " << dir_diff_left << endl;
		cout << "leftMostLine radian direction from FORWARD = " << dir_diff_right << ", requires difference < " << CV_PI / 6 << endl;
		if (abs((dir_diff_left + dir_diff_right)) < (CV_PI / 6)) {
			rightMostLine.stretchY(0.f, (float)(*drawMat).size().height);
			leftMostLine.stretchY(0.f, (float)(*drawMat).size().height);
			//! Draw the final chosen lane lines (BGR)
			leftMostLine.draw(*drawMat, cv::Scalar(0, 0, 255), 5);
			rightMostLine.draw(*drawMat, cv::Scalar(0, 255, 0), 5);
			lane_markings_.left = leftMostLine;
			lane_markings_.right = rightMostLine;
			lane_markings_.found = true;
			std::cout << "lane markings found" << std::endl;
		}
	}
	cv::imshow("mytest", *drawMat);
}
