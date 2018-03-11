#pragma once
#include <opencv2/core/core.hpp>
#include <limits>

using pointT = cv::Point_ < numeric_t >;
using vecT = cv::Vec < numeric_t, 4 >;
using limitsT = std::numeric_limits < numeric_t >;

template <typename numeric_t>
Class Line {
 public:
	void compute();
 private:
	Line(vecT pointVector);
	Line(pointT pointBegin, pointT pointEnd);
	Line();
	
	float length();
	float length2();
	void draw(cv::Mat& mat, cv::Scalar color, int thickness);
	float direction();
	float direction_fixed_half();
	bool differs_less_than_from(line<numeric_t> line2, numeric_t m_diff, numeric_t k_diff);
	void stretchY(numeric_t bottom, numeric_t top);
	
	pointT begin;
	pointT end;
	numeric_t leftMost_x() {return std::min(begin.x, end.x);}
	numeric_t rightMost_x()	{return std::max(begin.x, end.x);}

	float k;
	float m;
};

typedef line <int> linei;
typedef line <float> linef;
}
