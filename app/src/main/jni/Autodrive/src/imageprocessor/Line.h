#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_LINE_H_
#define ANDROIDCARDUINO_AUTODRIVE_LINE_H_

#include <opencv2/core/core.hpp>
#include <limits>
#include "Util.h"

template <class numeric_t>
class Line {
 public:
	 using pointT = cv::Point_ < numeric_t >;
	 using vecT = cv::Vec < numeric_t, 4 >;
	 using limitsT = std::numeric_limits < numeric_t >;

	Line(vecT pointVector);
	Line(pointT pointBegin, pointT pointEnd);
	Line();

	void compute();
	numeric_t leftMost_x();
	numeric_t rightMost_x();
	void stretchY(numeric_t bottom, numeric_t top);
	float direction_fixed_half();
	void draw(cv::Mat& mat, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1);
	float length();
	pointT begin;
	pointT end;
	float k;

 private:
	float length2();
	float direction();
	bool differs_less_than_from(Line<numeric_t> line2, numeric_t m_diff, numeric_t k_diff);
	float m;
};

using linei = Line<int>;
using linef = Line<float>;

#endif //ANDROIDCARDUINO_AUTODRIVE_LINE_H_