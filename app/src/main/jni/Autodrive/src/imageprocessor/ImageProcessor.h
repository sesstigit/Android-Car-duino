#pragma once
#include <opencv3/opencv.hpp>
#include <opencv3/core/core.hpp>

#include "../Autodrive.h"

#include "util.hpp"
#include "lightnormalizer.hpp"
#include "roadfollower.hpp"
#include "birdseyetransformer.hpp"

using namespace std;

int intersection_protect = 0;

#define _AUTODRIVE_DILATE
#define _DEBUG

Class ImageProcessor : Autodrive {
 public:
	bool init_processing(cv::Mat* mat);
	int continue_processing(cv::Mat& mat);
 private:
	left_line_found();
	right_line_found();
	is_left_lane();
	is_right_lane();
	dashed_line_gaps();
	void normalize_lighting(cv::Mat* bgr_image, int blur, float intensity)
	
	unique_ptr<roadfollower> road_follower_;
	cv::Mat perspective_;
	
	int thresh1_;
	int thresh2_;
	int intensity_;
	int blur_i_;

	POINT start_center_;
}

