#include "RoadLine.h"

void RoadLine::draw(cv::Mat* draw_mat)
{
    for (unsigned int i = 0; i < points_.size() - 1; i++)
    {
        linef(points_[i], points_[i + 1]).draw(*draw_mat, cv::Scalar(255, 0, 0), 4);
    }
}

bool RoadLine::add_point(POINT p)
{
    float prevAngle = angles_.back();
    float newAngle = Autodrive::linef(points_.back(), p).direction_fixed_half();

    float mean = get_mean_angle(0);
    //Almost never happens - ?
    if (fabs(newAngle - mean) > Settings::maxAngleDiff)//Mathf::PI_2 / 2.0f)
        return false;

    angles_.push_back(newAngle);
    angle_diffs_.push_back(newAngle - prevAngle);
    points_.push_back(p);
    distances_.push_back(p.x - center_x_);

    return true;
}

RoadLine::RoadLine(int center_x, POINT start_point) : center_x_(center_x)
{
    angles_.push_back(Direction::FORWARD);
    points_.push_back(start_point);
    angle_diffs_.push_back(0);
}

RoadLine::RoadLine()
{}

float RoadLine::get_mean_angle(unsigned int last_size = 0)
{
    unsigned int offset = 0;
    if (last_size > 0 && angles_.size() > last_size)
        offset = angles_.size() - last_size;
    return std::accumulate(angles_.begin() + offset, angles_.end(), 0.f) / (float) (angles_.size() - offset);
}

float RoadLine::get_mean_angle_diffs(unsigned int last_size = 0)
{
    unsigned int offset = 0;
    if (last_size > 0 && angles_.size() > last_size)
        offset = angles_.size() - last_size;
    return std::accumulate(angle_diffs_.begin() + offset, angle_diffs_.end(), 0.f) / ((float) angle_diffs_.size() - offset);
}

float RoadLine::get_estimated_angle(int n = 20)
{
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
