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
 
#include "BirdseyeTransformer.h"
using namespace Autodrive;

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
	cv::Mat matCopy = matIn->clone();
	//Might be needed on track
	//cv::erode(matCopy, matCopy, cv::Mat(), cv::Point(-1, -1), 1);
	cv::Mat cannied;
	cv::Canny(matCopy, cannied, thresh1, thresh2, 3);
	matCopy = cannied;
	auto lines = get_lane_markings(matCopy,matIn);
	if (!lines.found)
		return nullptr;
		
	linef leftLine = lines.left;
	linef rightLine = lines.right;

	float icrop = 0.f;
	float xdiff;
	float im_height = (float) matCopy.size().height;
	float im_width = (float) matCopy.size().width;
	//! Stretch the lines, but if they converge at the top of the image,
    //  keep cropping the top of the lines until they are at least X pixels apart
	do
	{
		xdiff = rightLine.leftMost_x() - leftLine.rightMost_x();
		rightLine.stretchY(icrop, im_height); //params=bottom(lowest value of y) to "top" (highest value of y)
		leftLine.stretchY(icrop, im_height);
		icrop+=3.f;
	} while (xdiff < im_width/3.0f); //div by 3, but can increase this to see further into distance

	lines.right.draw(matCopy, cv::Scalar(255, 0, 0), 3);  //blue lane lines
	lines.left.draw(matCopy, cv::Scalar(255, 0, 0), 3);

	float bottom = im_height;
	float xleft = leftLine.begin.x;
	float xright = rightLine.begin.x;
	
	//stretchY ensures the lines start at lowY and end at highY
	//Hence the center point will be half way between either the start points
	//or the end points.  Chosen difference from the image center is then:
	center_diff_ = (abs(xleft + xright) / 2.f - im_width /2.f);

	// Choose the input quadrangle for warping
	//POINT pts1[] = { leftLine.begin, rightLine.begin, POINT(leftLine.end.x, bottom), POINT(rightLine.end.x, bottom) };
	POINT pts1[] = { leftLine.begin, rightLine.begin, leftLine.end, rightLine.end };
	// Choose the output quadrangle for warping
	//POINT pts2[] = { leftLine.begin, rightLine.begin, POINT(xleft, bottom), POINT(xright, bottom) };
	POINT pts2[] = { leftLine.begin, rightLine.begin, POINT(leftLine.begin.x, im_height), POINT(rightLine.begin.x, im_height) };

	birdseye_matrix = cv::getPerspectiveTransform(pts1, pts2);

	left_image_border_ = linef(POINT(xleft - leftLine.end.x / 2 + 5, leftLine.end.y), POINT(0, leftLine.begin.y));
	right_image_border_ = linef(POINT(xright - (rightLine.end.x - im_width)/2 + 5, rightLine.end.y), POINT(im_width, rightLine.begin.y));
	
	return birdseye_matrix;
}

lanes BirdseyeTransformer::get_lane_markings(const cv::Mat& canniedMat,cv::Mat* drawMat) {
	lanes lanes;
	std::vector<cv::Vec4i> lines;
	linef leftMostLine;
	linef rightMostLine;
	// This transform detects straight "lines" by their endpoints (x0,y0, x1,y1) in image "canniedMat"
	// rho=1 pixel; theta=1 degree; threshold=20, minLinLength=10, maxLineGap=50
	// source image must be 8-bit, single-channel 
	cv::HoughLinesP(canniedMat, lines, 1, CV_PI / 180, 20, 10, 50);
	bool foundLeft = false;
	bool foundRight = false;
	int center = canniedMat.size().width / 2;

	for(cv::Vec4i one_line : lines) {
		int startx = one_line[0];
		int starty = one_line[1];
		int endx = one_line[2];
		int endy = one_line[3];
		linef one_hough_line(one_line);  //call the Line constructor to conver to a linef

		float dirr = one_hough_line.direction_fixed_half();
		float dir_diff = dirr - Direction::FORWARD;

        //! Ignore line if it differs from Direction::FORWARD by 1 radian (90 degrees about 1.6 radian)
		if (abs(dir_diff) > 1.f)
			continue;
        //! Draw all remaining candidate lines: image, colour(BGR), thickness
		one_hough_line.draw(*drawMat, cv::Scalar(0, 255, 255), 1);
		//! Work out whether it is the left or right line
		if ( startx > center + 5) {
			if (!foundRight) {
				rightMostLine = one_hough_line;
				foundRight = true;
			} else {
				if (one_hough_line.length() > rightMostLine.length()) {
					rightMostLine = one_hough_line; //this line is longer than the one previous found, so keep this one.
				}
			}
		}
		if (endx < center - 5) { // line ends LHS of center (left line slopes forward with start at bottom, end at top)
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
		leftMostLine.draw(*drawMat,cv::Scalar(0,0,255),2);
		rightMostLine.draw(*drawMat,cv::Scalar(0,255,0),2);
		//! As lines get more vertical, slope becomes a large number.  Hence getting two lines with the same slope +-1 is unlikely.
		//! Instead, use radians.  Compare the absolute direction of the two lines versus FORWARD.  They should be similar.  Threshold is PI/6, approx 50 degrees.
		float dir_diff_left = leftMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. PI/4 - PI/2 = -PI/4
		float dir_diff_right = rightMostLine.direction_fixed_half() - Autodrive::Direction::FORWARD; //e.g. 3PI/4 - PI/2 = PI/4
		//cout << "rightMostLine radian direction from FORWARD = " << dir_diff_left << endl;
		//cout << "leftMostLine radian direction from FORWARD = " << dir_diff_right << ", requires difference < " << CV_PI / 6 << endl;
		if (abs((dir_diff_left + dir_diff_right)) < (CV_PI / 6)) {
			rightMostLine.stretchY(0.f, (float)(*drawMat).size().height);
			leftMostLine.stretchY(0.f, (float)(*drawMat).size().height);
			//! Draw the final chosen lane lines (BGR)
			leftMostLine.draw(*drawMat, cv::Scalar(0, 0, 255), 5);
			rightMostLine.draw(*drawMat, cv::Scalar(0, 255, 0), 5);
			lanes.left = leftMostLine;
			lanes.right = rightMostLine;
			lanes.found = true;
		}
	}
	return lanes;
}
