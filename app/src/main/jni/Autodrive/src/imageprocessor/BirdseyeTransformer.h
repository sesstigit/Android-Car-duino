#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_
#define ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "Line.h"
#include "Util.h"

using namespace std;

struct lanes {
	linef left;
	linef right;
	bool found = false;
};

class BirdseyeTransformer {
 public:
	void birds_eye_transform(cv::Mat* mat, cv::Mat birdseye_matrix);
	optional<cv::Mat> find_perspective(cv::Mat* matIn, double thresh1 = 300, double thresh2 = 150);
	float center_diff() { return center_diff_; };  //getter
	linef left_image_border() { return left_image_border_; }; //getter
	linef right_image_border() { return right_image_border_; }; //getter
 private:
	lanes get_lane_markings(const cv::Mat& canniedMat,cv::Mat* drawMat);
    linef left_image_border_;
    linef right_image_border_;
    float center_diff_;
};


#endif //ANDROIDCARDUINO_AUTODRIVE_BIRDSEYETRANSFORMER_H_