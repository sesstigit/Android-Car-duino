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
	
	// From settings.h
	bool normalize_lighting_ = true;
	int first_fragment_max_dist_ = 30; //15-60, Maximum vertical distance to the first pixel from carY
	int left_iteration_length_ = 5; //1-15, // How many pixels to iterate to the left, for each pixel
	int right_iteration_length_ = 6; //1-15, How many pixels to iterate to the right, for each pixel
	int transform_line_removal_threshold_ = 18; // How many pixels of the transform border to remove from the canny
	bool use_left_line_ = true; // If the middle line should be taken into consideration or not
	float iterate_reduce_on_start_ = -2.f; // How much less to iterate right and left when finding the first point
	float max_angle_diff_ = 0.7f; // 0.4 - 1.4, Every pixel in a line can not have an angle from the previous pixel that deviates more than this
	unsigned int smoothening_ = 0; // 0 - 8v, N Frames to take the mean value from

	// PID SETTINGS
	float kp_ = 0.5;
	float ki_ = 0.0;
	float kd_ = 0.0;
}

