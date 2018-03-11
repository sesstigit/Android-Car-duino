#include "Line.h"

Line::Line(vecT pointVector) : begin(pointT(pointVector[0], pointVector[1])),
							 end(pointT(pointVector[2], pointVector[3])) {
	k = 1;
	m = 0;
	compute();
}

Line::Line(pointT pointBegin, pointT pointEnd) : begin(pointBegin), end(pointEnd)
{
	k = 1;
	m = 0;
	compute();
}

Line()::Line()
{
	k = 1;
	m = 0;
}

void Line::compute()
{
	if (int(begin.x) == int(end.x))
	{
		k = limitsT::has_infinity ? limitsT::infinity() : limitsT::max();
	} else
	{
		k = (begin.y - end.y) / (begin.x - end.x);
	}
	m = begin.y - begin.x * k;
}

float Line::length()
{
	return std::sqrt(length2());
}

float Line::length2()
{
	auto d = begin - end;
	return d.x *d.x + d.y *d.y;
}

void Line::draw(cv::Mat& mat, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1)
{
	cv::line(mat, begin, end, color, thickness);
}

float Line::direction()
{
	POINT diff = end - begin;
	return atan2(-diff.y, diff.x);
}

// RETURNS A DIRECTION THAT IS ALWAYS BETWEEN 0 - PI , (0-180 in degrees)
float Line::direction_fixed_half()
{
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

bool Line::differs_less_than_from(line<numeric_t> line2, numeric_t m_diff, numeric_t k_diff)
{
	return abs(line2.k - k) < k_diff && abs(line2.m - m < m_diff);
}

void Line::stretchY(numeric_t bottom, numeric_t top)
{
	numeric_t ychangeBegin = begin.y - bottom;
	begin = pointT(begin.x - ychangeBegin / k, bottom);

	numeric_t ychangeTop = end.y - top;
	end = pointT(end.x - ychangeTop / k, top);
}
