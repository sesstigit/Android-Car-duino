pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_

#include <vector>
#include "Line.h"
#include "Util.h"
#include <numeric>
//#include "../settings.hpp"

Class RoadLine
{
 public:
    RoadLine();
    RoadLine(int center_x, POINT start_point);
    void draw(cv::Mat* draw_mat);
    bool add_point(POINT p);
    float get_mean_angle(unsigned int last_size);
    float get_mean_angle_diffs(unsigned int last_size);
    float get_estimated_angle(int n);
    float get_mean_start_distance(unsigned int n_distances_from_begin);
    
 private:
    std::vector<POINT> points_;
    std::vector<int> distances_;
    std::vector<float> angles_;
    std::vector<float> angle_diffs_;
    int total_gap_ = 0;
    int center_x_;
};

#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_