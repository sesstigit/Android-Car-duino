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

#ifndef ANDROIDCARDUINO_AUTODRIVE_UTIL_H_
#define ANDROIDCARDUINO_AUTODRIVE_UTIL_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <memory>

namespace Autodrive {

	using POINT = cv::Point2f;
	using UINT = unsigned int;

	struct Mathf
	{
		static const float PI;
		static const float PI_2;
		static int toDegrees(float radians) { return int(radians * 180.f / PI); }
	};

	struct Direction
	{
		static const float RIGHT;
		static const float LEFT;
		static const float FORWARD;
	};

#ifndef __ANDROID__
	void show_image(cv::Mat mat, int resize, std::string wName);
#endif

	template<typename T, typename... Args>
	std::unique_ptr<T> make_unique(Args&&... args)
	{
		return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
	}

	template<class TYPE>
	struct optional
	{
		optional(TYPE value) : valid(true), val(value)
		{
		}
		optional() : valid(false)
		{
		}
		optional(::std::nullptr_t) : valid(false)
		{
		}
		explicit operator bool() const
		{
			return valid;
		}
		TYPE* operator->()
		{
			return &val;
		}
		TYPE operator* () const
		{
			return val;
		}
		bool valid;
	private:
		TYPE val;
	};

	struct SearchResult
	{
		POINT point;
		int distance;
		bool found = false;
	};
    //! Make a line from a given point in a given direction.
    //! Iterate along the line until a non-zero point is found, i.e. white point
    //! This works becaue the image is thresholded to be black 0 or white 1 only,
	SearchResult firstnonzero_direction(const cv::Mat& mat, cv::Point_ < float > start, float direction, int maxDist);

	
	optional<cv::Point> firstnonzero_horizontal(const cv::Mat& mat, cv::Point iterator);


	template <class numeric_t>
	numeric_t weighted_average(numeric_t val1, numeric_t val2, numeric_t val1_multiplier)
	{
		return (val1*val1_multiplier + val2) / (val1_multiplier + 1);
	}
}

#endif //ANDROIDCARDUINO_AUTODRIVE_UTIL_H_