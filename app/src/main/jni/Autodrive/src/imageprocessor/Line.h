/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/
 
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

    Line(vecT pointVector) : begin(pointT(pointVector[0], pointVector[1])),
                             end(pointT(pointVector[2], pointVector[3])) {
        k = 1;
        m = 0;
        compute();
    }

    Line(pointT pointBegin, pointT pointEnd) : begin(pointBegin), end(pointEnd) {
        k = 1;
        m = 0;
        compute();
    }

    Line() {
        k = 1;
        m = 0;
    }

	void compute() {
         if (int(begin.x) == int(end.x))
         {
             k = limitsT::has_infinity ? limitsT::infinity() : limitsT::max();
         } else
         {
             k = (begin.y - end.y) / (begin.x - end.x);
         }
         m = begin.y - begin.x * k;
    }
         
	numeric_t leftMost_x() {
        return std::min(begin.x, end.x); 
    };
    
	numeric_t rightMost_x() {
        return std::max(begin.x, end.x); 
    };
	
	void stretchY(numeric_t bottom, numeric_t top) {
        numeric_t ychangeBegin = begin.y - bottom;
        begin = pointT(begin.x - ychangeBegin / k, bottom);
    
        numeric_t ychangeTop = end.y - top;
        end = pointT(end.x - ychangeTop / k, top);
    }
	
	float direction_fixed_half() {
        float dirr = direction();
        if (dirr < 0.f) {
            dirr = Mathf::PI + dirr;
        }
        if (dirr > Mathf::PI) {
            dirr = dirr - Mathf::PI;
        }
        return dirr;
    }
	
	void draw(cv::Mat& mat, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1) {
        cv::line(mat, begin, end, color, thickness);
    }
	
	float length() {
        return std::sqrt(length2());
    }
    
	float length2() {
        auto d = begin - end;
        return d.x *d.x + d.y *d.y;
    }
    
	pointT begin;
	pointT end;
	float k;

 private:
	float direction() {
        POINT diff = end - begin;
        return atan2(-diff.y, diff.x);
    }
    
	bool differs_less_than_from(Line<numeric_t> line2, numeric_t m_diff, numeric_t k_diff) {
        return abs(line2.k - k) < k_diff && abs(line2.m - m < m_diff);
    }
    
	float m;
};

using linei = Line<int>;
using linef = Line<float>;

#endif //ANDROIDCARDUINO_AUTODRIVE_LINE_H_