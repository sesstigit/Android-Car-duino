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

#ifndef ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_
#define ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_

#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "CarCmd.h"
#include "imageprocessor/ImageProcessor.h"

using namespace cv;
namespace Autodrive {

	class Car;  //forward declaration

	// Overtaking is a subclass of Autodrive because it is one mode of Autodrive.  
	class Overtaking {
	public:
		Overtaking(Car* ocar);
		CarCmd run(CarCmd lastCarCmd, Mat* mat);
		//int run();
	private:
		Car* ocar_;
		bool debug_mode_;
		int distance_travelled_;
		bool overtaking_;
		int turn_left_;
		int turn_left_calibration_;
		int oomph_turn_left_;
		int turn_right_;
		int turn_right_calibration_;
		int oomph_turn_right_;
		bool turn_left_calibration_finished_;
		int obstacle_distance_;
		bool obstacle_met_;
		bool obstacle_passed_;
		//bool line_LHS_sensor_.value();
		//bool line_RHS_sensor_.value();
		bool stop_;
	};
}
#endif // ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_