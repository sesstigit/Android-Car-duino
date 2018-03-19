#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_

#include "Line.h"
#include "Util.h"
//#include "../settings.hpp"
#include "RoadLine.h"

class ImageConfig;  //forward declaration

class RoadLineBuilder
{
 public:
    RoadLineBuilder(POINT start_point, float center_x, int car_y, ImageConfig* img_conf);
    RoadLine build(const cv::Mat& cannied, size_t maxsize);
	POINT first_start() { return first_start_; };
	POINT last_start() { return last_start_; };

 private:
    const int point_dist_;
    const int max_dist_from_start_;
    const int max_upwards_iterations_;
    int total_gap_;
    int car_y_;
    POINT first_start_;
    POINT last_start_;
    float center_x_;
	ImageConfig* img_conf_;
    //was static on next line
    SearchResult find_point(const cv::Mat& cannied, POINT start, float left_angle, float fight_angle,float iteration_reduction = 0);
    POINT get_first_point(const cv::Mat& cannied);
    optional<POINT> get_next_point(const cv::Mat& cannied, float est_angle, const POINT& prev_point,int delta);
    
};

#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_