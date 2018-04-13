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
 
#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_
#define ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "Line.h"
#include "Util.h"

//using namespace std;
namespace Autodrive {

	struct lanes {
		linef left;
		linef right;
		bool found = false;
	};

	class BirdseyeTransformer {
	public:
	    BirdseyeTransformer() {};
	    ~BirdseyeTransformer() {};
	    //! Transform the input image using the supplied birdseye transform matrix (from find_perspective())
        //! The result should be a birdseye view of the road in which the lane lines are straight
		void birds_eye_transform(cv::Mat* mat, cv::Mat birdseye_matrix);
		//! find_perspective is only used during car initialisation.  It aims to locate the edges of the lane,
        //! and then calculate a perspective to make those edge lines straight, i.e. in preparation for 
        //! a birdseye transform.  It assumes the car is on a straight road. Basic steps:
        //! - Canny the input image
        //! - get_lane_markings for leftLine and rightLine
        //! - stretch the lines to make them a standard length
        //! - Then get birdseye perspective transform
		optional<cv::Mat> find_perspective(cv::Mat* matIn, double thresh1 = 80, double thresh2 = 240);
		float center_diff() { return center_diff_; };  //getter
		linef left_image_border() { return left_image_border_; }; //getter
		linef right_image_border() { return right_image_border_; }; //getter
	private:
	    //private method
	    //! Locate edges of the lane from input image.  Lanes must be marked by a line.  Steps are:
        //! - take the input image (which is grayscale edge detection produced by cv:Canny)
        //! - convert each edge into a line using the Hough transform
        //! - apply tests to all lines to see if they could be the right or left lane line
        //! - save the best lane lines (which passed all tests) to class member lane_markings_, and indicate success with lane_markings_.found=true
        //! - if no lines passed the test, then lane_markings_.found remains false
		void calc_lane_markings(const cv::Mat& canniedMat, cv::Mat* drawMat);
		//private members
		lanes lane_markings_;
		// The left and right image borders are the edge of the original image in the warped space
        // Unfortunately, the border is always detected by Canny as an edge, and hence confuses navigation.
        // Hence we store its location here, so it can be blanked with black pixels after cv::Canny.
		linef left_image_border_;
		linef right_image_border_;
		float center_diff_;
	};

}
#endif //ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_