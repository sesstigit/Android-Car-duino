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

#ifndef ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ImageConfig.h"
#include "Util.h"
#include "Line.h"
#include "CarCmd.h"
#include "BirdseyeTransformer.h"
#include "Binarization.h"

//using namespace std;
namespace Autodrive {

	class RoadFollower; //forward declaraion.

	class AdvImageProcessor {
	public:
		AdvImageProcessor(const ImageConfig& img_conf);
		//! init_processing is the first function called by Autodrive to locate the lanes
        //! and apply a birdseye transform to the input camera image.
        //! If it cannot find the lanes, it prints a message to screen.
        bool init_processing(cv::Mat& mat);
        //! After successful initialisation, continue_processing is called for each frame from the camera.
        //! It tracks the lane lines in the input frame, and calculates driving commands
        //! to steer the car along the lane.
		CarCmd continue_processing(cv::Mat& mat);
		
	private:
        //! Keep a reference to the image processing configuration parameters
		const ImageConfig& img_conf_;
		std::unique_ptr<BirdseyeTransformer> birdseye_;
		//! The perspective is calculated during initialisation by BirdseyeTransform class.
		//! Each subsequent input frame is then warped according to the perspective transform
		//! Hence if it is wrong, this program will not work well
		cv::Mat perspective_;
		// Flag whether lane line state is conserved (this permits averaging results)
		bool keep_state_;
		LaneLine line_lt_;
		LaneLine line_rt_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_
