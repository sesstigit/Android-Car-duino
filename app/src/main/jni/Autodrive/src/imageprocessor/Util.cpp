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
	
	//! Optimise https://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html
	// the cv::undistort functions calls initUndistortRectifyMap and then remap.
	// initUndistortRectifyMap only needs to be done once, so we could improve
	// app efficiency that way.
	void camera_undistort(cv::Mat& inframe, cv::Mat& outframe, cv::Mat* intrinsic_matrix, cv::Mat* distortion_coeffs) {

		//if (img_conf.intrinsic_matrix_.is_empty() || img_conf.distortion_coeffs_.is_empty()){
		if (intrinsic_matrix == nullptr || distortion_coeffs == nullptr) {
			//std::cerr << "INFO: camera calibration info unavailable.  No undistortion applied" << std::endl;
		}
		else {
			//std::cerr << "INFO: undistorting frame" << std::endl;
			cv::undistort(inframe, outframe, *intrinsic_matrix, *distortion_coeffs);
		}
	}


	void normalize_lighting(cv::Mat& bgr_image)
	{
		// convert bgr_image to Lab
		cv::Mat lab_image;
		cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

		// Extract the L channel
		std::vector<cv::Mat> lab_planes(3);
		cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

										   // apply the CLAHE algorithm to the L channel
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
		clahe->setClipLimit(4);
		cv::Mat dst;
		clahe->apply(lab_planes[0], dst);

		// Merge the the color planes back into an Lab image
		dst.copyTo(lab_planes[0]);
		cv::merge(lab_planes, lab_image);

		// convert back to original number of color channels
		if (bgr_image.type() == CV_8UC4) {
			cv::Mat temp_bgr_image;
			cv::cvtColor(lab_image, temp_bgr_image, CV_Lab2RGB);
			cv::cvtColor(temp_bgr_image, bgr_image, CV_RGB2RGBA);  //android images appear to be RGBA
		}
		else {
			cv::cvtColor(lab_image, bgr_image, CV_Lab2BGR);
		}

	}

#ifdef _DEBUG
	std::string type2str(int type) {
	  std::string r;

	  uchar depth = type & CV_MAT_DEPTH_MASK;
	  uchar chans = 1 + (type >> CV_CN_SHIFT);

	  switch ( depth ) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
	  }

	  r += "C";
	  r += (chans+'0');

	  return r;
	}
#endif
}