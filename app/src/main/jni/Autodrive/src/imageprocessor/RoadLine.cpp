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
 
#include "RoadLine.h"

using namespace Autodrive;

RoadLine::RoadLine(int center_x, POINT start_point, const ImageConfig& img_conf) :
	center_x_(center_x), img_conf_(img_conf) {
	angles_.push_back(Direction::FORWARD);
	points_.push_back(start_point);
	angle_diffs_.push_back(0);
}

RoadLine::RoadLine(const ImageConfig& img_conf) : img_conf_(img_conf) {
}

void RoadLine::draw(cv::Mat& draw_mat) {
    for (unsigned int i = 0; i < points_.size() - 1; i++)
    {
        linef(points_[i], points_[i + 1]).draw(draw_mat, cv::Scalar(255, 0, 0), 4);
    }
}

bool RoadLine::add_point(POINT p) {
    float prevAngle = angles_.back();
    float newAngle = linef(points_.back(), p).direction_fixed_half();

    float mean = get_mean_angle(0);
    //Almost never happens - ?
    if (fabs(newAngle - mean) > img_conf_.max_angle_diff_)//Mathf::PI_2 / 2.0f)
        return false;

    angles_.push_back(newAngle);
    angle_diffs_.push_back(newAngle - prevAngle);
    points_.push_back(p);
    distances_.push_back(p.x - center_x_);

    return true;
}

float RoadLine::get_mean_angle(unsigned int last_size) {
    unsigned int offset = 0;
    if (last_size > 0 && angles_.size() > last_size)
        offset = angles_.size() - last_size;
    return std::accumulate(angles_.begin() + offset, angles_.end(), 0.f) / (float) (angles_.size() - offset);
}

float RoadLine::get_mean_angle_diffs(unsigned int last_size) {
    unsigned int offset = 0;
    if (last_size > 0 && angles_.size() > last_size)
        offset = angles_.size() - last_size;
    return std::accumulate(angle_diffs_.begin() + offset, angle_diffs_.end(), 0.f) / ((float) angle_diffs_.size() - offset);
}

float RoadLine::get_estimated_angle(int n) {
    return angles_.back() + get_mean_angle_diffs(n);
}

float RoadLine::get_mean_start_distance(unsigned int n_distances_from_begin)
{
    if (distances_.size() == 0)
        return 0;
        
    if (n_distances_from_begin >= distances_.size())
        n_distances_from_begin = int(distances_.size());
    return std::accumulate(distances_.begin(), distances_.begin() + n_distances_from_begin, 0.f) / float(n_distances_from_begin);
}
