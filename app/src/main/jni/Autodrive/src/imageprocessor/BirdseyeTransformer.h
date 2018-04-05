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
		void birds_eye_transform(cv::Mat* mat, cv::Mat birdseye_matrix);
		optional<cv::Mat> find_perspective(cv::Mat* matIn, double thresh1 = 80, double thresh2 = 240);
		float center_diff() { return center_diff_; };  //getter
		linef left_image_border() { return left_image_border_; }; //getter
		linef right_image_border() { return right_image_border_; }; //getter
	private:
		lanes get_lane_markings(const cv::Mat& canniedMat, cv::Mat* drawMat);
		linef left_image_border_;
		linef right_image_border_;
		float center_diff_;
	};

}
#endif //ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_