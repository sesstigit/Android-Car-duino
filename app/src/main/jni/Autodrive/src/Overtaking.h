#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_
#define ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_

#include <string.h>
#include <math.h>

#include "Autodrive.h"

#include <opencv3/opencv.hpp>
#include <opencv3/core/core.hpp>
//#include "imageprocessor/imageprocessor.hpp"

using namespace cv;

// Overtaking is a subclass of Autodrive because it is one mode of Autodrive.  
Class Overtaking : public Autodrive {
 public:
	int run(Mat* mat);
 private:
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
}

#endif // ANDROIDCARDUINO_AUTODRIVE_OVERTAKING_H_