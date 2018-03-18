#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ImageConfig.h"
#include "Util.h"
#include "RoadFollower.h"
#include "BirdseyeTransformer.h"
#include "../CarCmd.h"

using namespace std;

class RoadFollower; //forward declaraion.

int intersection_protect = 0;

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
	void normalize_lighting(cv::Mat* bgr_image, int blur, float intensity);

	ImageConfig* img_conf_;
	int thresh1_;   // used in normalize lighting
	int thresh2_;   // used in normalize lighting
	int intensity_; // used in normalize lighting
	int blur_i_;    // used in normalize lighting
	std::unique_ptr<RoadFollower> road_follower_;
	cv::Mat perspective_;

	POINT start_center_;
};

#endif //ANDROIDCARDUINO_AUTODRIVE_IMAGEPROCESSOR_H_