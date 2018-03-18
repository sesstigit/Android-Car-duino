#include "Line.h"

template <class numeric_t>
Line<numeric_t>::Line(vecT pointVector) : begin(pointT(pointVector[0], pointVector[1])),
							 end(pointT(pointVector[2], pointVector[3])) {
	k = 1;
	m = 0;
	compute();
}

template <class numeric_t>
Line<numeric_t>::Line(pointT pointBegin, pointT pointEnd) : begin(pointBegin), end(pointEnd) {
	k = 1;
	m = 0;
	compute();
}

template <class numeric_t>
Line<numeric_t>::Line() {
	k = 1;
	m = 0;
}

template <class numeric_t>
void Line<numeric_t>::compute() {
	if (int(begin.x) == int(end.x))
	{
		k = limitsT::has_infinity ? limitsT::infinity() : limitsT::max();
	} else
	{
		k = (begin.y - end.y) / (begin.x - end.x);
	}
	m = begin.y - begin.x * k;
}

template <class numeric_t>
float Line<numeric_t>::length() {
	return std::sqrt(length2());
}

template <class numeric_t>
float Line<numeric_t>::length2() {
	auto d = begin - end;
	return d.x *d.x + d.y *d.y;
}

template <class numeric_t>
void Line<numeric_t>::draw(cv::Mat& mat, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1) {
	cv::line(mat, begin, end, color, thickness);
}

template <class numeric_t>
float Line<numeric_t>::direction() {
	POINT diff = end - begin;
	return atan2(-diff.y, diff.x);
}

// RETURNS A DIRECTION THAT IS ALWAYS BETWEEN 0 - PI , (0-180 in degrees)
template <class numeric_t>
float Line<numeric_t>::direction_fixed_half() {
	float dirr = direction();
	if (dirr < 0.f)
	{
		dirr = Mathf::PI + dirr;
	}
	if (dirr > Mathf::PI)
	{
		dirr = dirr - Mathf::PI;
	}
	return dirr;
}

template <class numeric_t>
bool Line<numeric_t>::differs_less_than_from(Line<numeric_t> line2, numeric_t m_diff, numeric_t k_diff) {
	return abs(line2.k - k) < k_diff && abs(line2.m - m < m_diff);
}

template <class numeric_t>
void Line<numeric_t>::stretchY(numeric_t bottom, numeric_t top) {
	numeric_t ychangeBegin = begin.y - bottom;
	begin = pointT(begin.x - ychangeBegin / k, bottom);

	numeric_t ychangeTop = end.y - top;
	end = pointT(end.x - ychangeTop / k, top);
}

template <class numeric_t>
numeric_t Line<numeric_t>::leftMost_x() {
	return std::min(begin.x, end.x); 
};

template <class numeric_t>
numeric_t Line<numeric_t>::rightMost_x() {
	return std::max(begin.x, end.x); 
};
