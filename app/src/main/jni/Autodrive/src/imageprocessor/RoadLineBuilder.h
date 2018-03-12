pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_

#include "Line.h"
#include "Util.h"
//#include "../settings.hpp"
#include "RoadLine.h"

Class RoadLineBuilder
{
 public:
    RoadLineBuilder(POINT start_point, float center_x,int car_y);
    RoadLine build(const cv::Mat& cannied, size_t maxsize);
    
 private:
    static const int point_dist_;
    static const int max_dist_from_start_;
    static const int max_upwards_iterations_;
    int total_gap_;
    int car_y_;
    POINT first_start_;
    POINT last_start_;
    float center_x_;

    static SearchResult find_point(const cv::Mat& cannied, POINT start, float left_angle, float fight_angle,float iteration_reduction = 0);
    POINT get_first_point(const cv::Mat& cannied);
    optional<POINT> get_next_point(const cv::Mat& cannied, float est_angle, const POINT& prev_point,int delta);
    
};

#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_