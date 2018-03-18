#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_
#define ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_

#include "ImageProcessor.h"
#include "Line.h"
#include "RoadLineBuilder.h"

#include <chrono>

class LineFollower
{
 public:
	LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x, int carY, ImageConfig* img_conf);
	void update(cv::Mat& cannied);
	optional<int> get_prefered_angle();
 private:
    // params
	 ImageConfig* img_conf_;
	RoadLine road_line_;
	std::unique_ptr<RoadLineBuilder> road_builder_;
    int road_size_;
	bool is_found_;
    float target_road_distance_;
    
	// Private methods
	void draw(cv::Mat* colorCopy, int centerX);
    bool is_found();
    float distance_deviation();
    int total_gap();
};

#endif //ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_