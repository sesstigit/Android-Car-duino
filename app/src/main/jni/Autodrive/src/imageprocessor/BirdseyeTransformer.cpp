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
	float height = (float) matCopy.size().height;
	float width = (float) matCopy.size().width;
	//! Stretch the lines, but if they converge at the top of the image,
    //  keep cropping the top of the lines until they are at least X pixels apart
	do
	{
		xdiff = rightLine.leftMost_x() - leftLine.rightMost_x();
		rightLine.stretchY(icrop, height);
		leftLine.stretchY(icrop, height);
		icrop+=3.f;
	} while (xdiff < width/3.0f);

	float bottom = height;
	float xleft = leftLine.end.x;
	float xright = rightLine.end.x;
//#define _VISUAL_WARP  //not used in final Autodrive
#ifdef _VISUAL_WARP
	while (warping){
		if (xleft < leftLine.begin.x || xright > rightLine.begin.x)
		{
			if (xleft < leftLine.begin.x){
				xleft++;
			}
			if (xright > rightLine.begin.x){
				xright--;
			}
		}
		else
#endif
		{
			xright = rightLine.begin.x;
			xleft = leftLine.begin.x;
		}
		
		center_diff_ = (abs(xleft + xright) / 2.f - width /2.f);

		//Crop moves the two upper cordinates farther appart, both from each other and from the lower cordinates (Outside the image)
		POINT pts1[] = { leftLine.begin, rightLine.begin, POINT(leftLine.end.x, bottom), POINT(rightLine.end.x, bottom) };

		//Warp compresses the bottom two cordinates together
		POINT pts2[] = { leftLine.begin, rightLine.begin, POINT(xleft, bottom), POINT(xright, bottom) };

		birdseye_matrix = cv::getPerspectiveTransform(pts1, pts2);

		left_image_border_ = linef(POINT(xleft - leftLine.end.x / 2, leftLine.end.y +2), POINT(0, leftLine.begin.y+2));
		right_image_border_ = linef(POINT(xright - (rightLine.end.x - width)/2, rightLine.end.y+2), POINT(width, rightLine.begin.y+2));
		
#ifdef _VISUAL_WARP
		cv::Mat warped_image;
		cv::warpPerspective(matCopy, warped_image, *birdseye_matrix, matCopy.size(), cv::INTER_LINEAR);

		cv::resize(warped_image, warped_image, warped_image.size() * 3);//resize image
		cv::imshow("mainwindow", warped_image);
		cv::waitKey(1); // waits to display frame
	}
#endif
	return birdseye_matrix;
}

lanes BirdseyeTransformer::get_lane_markings(const cv::Mat& canniedMat,cv::Mat* drawMat) {
	lanes lanes;
	std::vector<cv::Vec4i> lines;
	linef leftMostLine;
	linef rightMostLine;
	// This transform detects straight "lines" as extremes (x0,y0, x1,y1) in image "canniedMat"
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
		if (abs(dir_diff) < 0.f || abs(dir_diff) > 1.f)
			continue;
        //! Draw all remaining candidate lines: image, colour(BGR), thickness
		one_hough_line.draw(*drawMat, cv::Scalar(0, 0, 255), 5);
		//! Work out whether it is the left or right line
		if ( startx > center + 20) {
				if (endx > startx && endy > starty && one_hough_line.length() > 50) {  // line starting on RHS, and sloping down further right
					rightMostLine = one_hough_line;
					foundRight = true;
				}
		}
		if ( endx < center - 20) { // line on LHS of center
				leftMostLine = one_hough_line;
				foundLeft = true;
		}
	}
	if (foundRight && foundLeft) {
    // Draw the likely the lane markers, params=Image,colour(BGR),thckness
		leftMostLine.draw(*drawMat,cv::Scalar(255,0,0),2);
		rightMostLine.draw(*drawMat,cv::Scalar(255,0,0),2);
		if ( abs((-rightMostLine.k) - leftMostLine.k) < 0.9f)
		{   rightMostLine.stretchY(0.f, (float) canniedMat.size().height);
			leftMostLine.stretchY(0.f, (float) canniedMat.size().height );
			//TODO: Deprecated line
			//if ((leftMostLine.leftMost_x() >rightMostLine.rightMost_x()))
			{
			    //! Draw the final chosen lane lines (BGR)
				leftMostLine.draw(*drawMat, cv::Scalar(0, 0, 255), 5);
				rightMostLine.draw(*drawMat,cv::Scalar(0,0,255),5);
				lanes.left = leftMostLine; 
				lanes.right = rightMostLine;
				lanes.found = true;
			}
		}
	}
	return lanes;
}
