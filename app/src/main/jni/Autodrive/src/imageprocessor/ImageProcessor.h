#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ImageConfig.h"
#include "Util.h"
#include "RoadFollower.h"
#include "BirdseyeTransformer.h"

using namespace std;

int intersection_protect = 0;

#define _AUTODRIVE_DILATE
#define _DEBUG

Class ImageProcessor {
 public:
	ImageProcessor(ImageConfig* img_conf);
	bool init_processing(cv::Mat* mat);
	int continue_processing(cv::Mat& mat);
 private:
	left_line_found();
	right_line_found();
	is_left_lane();
	is_right_lane();
	dashed_line_gaps();
	void normalize_lighting(cv::Mat* bgr_image, int blur, float intensity)

	ImageConfig* img_conf_;
	int thresh1_;   // used in normalize lighting
	int thresh2_;   // used in normalize lighting
	int intensity_; // used in normalize lighting
	int blur_i_;    // used in normalize lighting
	unique_ptr<roadfollower> road_follower_;
	cv::Mat perspective_;

	POINT start_center_;
}

