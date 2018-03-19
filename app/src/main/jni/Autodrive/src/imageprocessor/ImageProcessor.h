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

	//TODO: what was this used for?  It is also a variable used in opencv.
	//int intersection_protect = 0;

#define _AUTODRIVE_DILATE
#define _DEBUG

	class ImageProcessor {
	public:
		ImageProcessor(ImageConfig* img_conf);
		bool init_processing(cv::Mat* mat);
		CarCmd continue_processing(cv::Mat& mat);
	private:
		bool left_line_found();
		bool right_line_found();
		bool is_left_lane();
		bool is_right_lane();
		int dashed_line_gaps();
		void normalize_lighting(cv::Mat* bgr_image, int blur = 20, float intensity = 0.5f);

		ImageConfig* img_conf_;
		int thresh1_;   // used in cv::Canny function
		int thresh2_;   // used in cv::Canny function
		int intensity_; // used in normalize lighting.  High value (200) means very few lines detected.  Low value (10) means lots of noisy lines detected.
		int blur_i_;    // used in normalize lighting
		std::unique_ptr<RoadFollower> road_follower_;
		cv::Mat perspective_;

		POINT start_center_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_