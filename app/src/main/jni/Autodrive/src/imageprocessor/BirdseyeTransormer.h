#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_

#include <opencv3/opencv.hpp>
#include <opencv3/core/core.hpp>

#include "Line.h"
#include "Util.h"

using namespace std;

struct lanes{
	linef left;
	linef right;
	bool found = false;
};

Class BirdseyeTransformer {
 public:
	void birds_eye_transform(cv::Mat* mat, cv::Mat birdseye_matrix);
 private:
	lanes getLaneMarkings(const cv::Mat& canniedMat,cv::Mat* drawMat);
	optional<cv::Mat> find_perspective(cv::Mat* matIn, double thresh1 = 300, double thresh2 = 150);

    linef leftImageBorder;
    linef rightImageBorder;
    float centerDiff;
}


#endif //ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_