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

#ifndef ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ImageConfig.h"
#include "Util.h"
#include "RoadFollower.h"
#include "BirdseyeTransformer.h"
#include "CarCmd.h"

//using namespace std;
namespace Autodrive {

	class RoadFollower; //forward declaraion.

	class ImageProcessor {
	public:
		ImageProcessor(const ImageConfig& img_conf);
		//! init_processing is the first function called by Autodrive to locate the lanes
        //! and apply a birdseye transform to the input camera image.
        //! If it cannot find the lanes, it prints a message to screen.
        bool init_processing(cv::Mat& mat);
        //! After successful initialisation, continue_processing is called for each frame from the camera.
        //! It tracks the lane lines in the input frame, and calculates driving commands
        //! to steer the car along the lane.
		CarCmd continue_processing(cv::Mat& mat);
		//! Normalize lighting in the input frame with the CLAHE algorithm
		void normalize_lighting(cv::Mat& bgr_image);
		//! Set the perspective externally, e.g. from one saved in a file
		void set_perspective(cv::Mat* p);
		//! Remove the perspective so we know to calculate a new one
		void delete_perspective();
		cv::Mat* get_perspective();
	private:
		bool left_line_found();
		bool right_line_found();
		//! Returns wether the car is on the left lane
        //! Currently only works if both roadlines are found by comparing their gaps
        //! because the line down the middle of the road is dashed
        bool is_left_lane();
		bool is_right_lane();
		int dashed_line_gaps();
		
        //! Keep a reference to the image processing configuration parameters
		const ImageConfig& img_conf_;
		std::unique_ptr<BirdseyeTransformer> birdseye_;
		std::unique_ptr<RoadFollower> road_follower_;
		//! The perspective was calculated during initialisation by BirdseyeTransform class.
		//! Each subsequent input frame is then warped according to the perspective transform
		//! Hence if it is wrong, then this program will not work well
		cv::Mat perspective_;

		POINT start_center_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_
