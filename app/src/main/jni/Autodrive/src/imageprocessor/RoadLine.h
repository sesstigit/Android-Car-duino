#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_

#include <vector>
#include <numeric>

#include "Line.h"
#include "Util.h"
#include "ImageConfig.h"

//using namespace Autodrive;

class RoadLine
{
 public:
    RoadLine();
    RoadLine(int center_x, POINT start_point, ImageConfig* img_conf);
    void draw(cv::Mat* draw_mat);
    bool add_point(POINT p);
	int total_gap() { return total_gap_; };  //getter
	int num_points() { return static_cast<int>(points_.size()); };
	POINT back_points() { return points_.back(); }; 
    float get_mean_angle(unsigned int last_size = 0);
    float get_mean_angle_diffs(unsigned int last_size = 0);
    float get_estimated_angle(int n = 20);
    float get_mean_start_distance(unsigned int n_distances_from_begin);
	void set_total_gap(int calc_gap) { total_gap_ = calc_gap; };
 private:
    std::vector<POINT> points_;
    std::vector<int> distances_;
    std::vector<float> angles_;
    std::vector<float> angle_diffs_;
    int total_gap_ = 0;
    int center_x_;
	ImageConfig* img_conf_;
};

#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_