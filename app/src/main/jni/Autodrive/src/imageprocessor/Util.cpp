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
 
#include "Util.h"

namespace Autodrive {

	const float Mathf::PI = acosf(-1);
	const float Mathf::PI_2 = PI / 2.f;

	const float Direction::RIGHT = 0.f;
	const float Direction::LEFT = Mathf::PI;
	const float Direction::FORWARD = Mathf::PI_2;


#ifndef __ANDROID__
	void show_image(cv::Mat mat, int resize, std::string wName)
	{
		cv::resize(mat, mat, mat.size() * resize);//resize image
		cv::imshow(wName, mat);
	}
#endif

	SearchResult firstnonzero_direction(const cv::Mat& mat, cv::Point_ < float > start, float direction, int maxDist)
	{
		SearchResult res;
		POINT new_pos = start + POINT(::std::cos(direction) * maxDist, -::std::sin(direction) * maxDist);
		cv::LineIterator it(mat, start, new_pos);
		for (int i = 0; i < it.count; i++, it++)
		{
			if (mat.at<uchar>(it.pos()))
			{
				res.found = true;
				res.distance = i;
				res.point = it.pos();
				break;
			}
		}
		return res;
	}

	optional<cv::Point> firstnonzero_horizontal(const cv::Mat& mat, cv::Point iterator)
	{
		while (iterator.x < mat.size().width - 1)
		{
			if (mat.at<uchar>(iterator) != 0)
			{
				return iterator;
			}
			iterator.x++;
		}
		return nullptr;
	}

}