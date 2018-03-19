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
 
#include "RoadLineBuilder.h"
#include "ImageConfig.h"

using namespace Autodrive;

RoadLineBuilder::RoadLineBuilder(POINT start_point, float center_x, int car_y, ImageConfig* img_conf) :
    first_start_(start_point), last_start_(start_point), center_x_(center_x),
	car_y_(car_y), img_conf_(img_conf), point_dist_(4), 
	max_dist_from_start_(22), max_upwards_iterations_(100), total_gap_(0) {
}

RoadLine RoadLineBuilder::build(const cv::Mat& cannied, size_t maxsize) {
    RoadLine road(center_x_, get_first_point(cannied), img_conf_);
    optional<POINT> new_point;
    total_gap_ = 0;
    while ((new_point = get_next_point(cannied, road.get_estimated_angle(), road.back_points(), point_dist_)).valid && road.num_points() < maxsize)
    {
        if (!road.add_point(*new_point))
            break;
    }
    road.set_total_gap(total_gap_);

    return road;
}

//TODO: was "static" on next line    
SearchResult RoadLineBuilder::find_point(const cv::Mat& cannied, POINT start, float left_angle, float fight_angle, float iteration_reduction) {
	SearchResult right_search = firstnonzero_direction(cannied, start, fight_angle, static_cast<int>(img_conf_->right_iteration_length_ - iteration_reduction));
    SearchResult left_search = firstnonzero_direction(cannied, start, left_angle, static_cast<int>(img_conf_->left_iteration_length_ - iteration_reduction));
    if (left_search.found && right_search.found)
    {
        if (left_search.distance <= right_search.distance + 15)
        {
            {
                return left_search;
            }
        } else
        {
            return right_search;
        }
    } else if (left_search.found)
    {
        return left_search;
    }

    return right_search;
}


POINT RoadLineBuilder::get_first_point(const cv::Mat& cannied)
{
    SearchResult searchRes;
    int unfound = 0;

    POINT start_point = last_start_;

    //SEARCH UPWARDS UNTIL HIT
    while (!searchRes.found && unfound++ < img_conf_->first_fragment_max_dist_)
    {
        searchRes = find_point(cannied, start_point, Direction::LEFT, Direction::RIGHT,img_conf_->iterate_reduce_on_start_);
        if (!searchRes.found)
            start_point.y--;
    }

    // SEARCH DOWNWARDS IF HIT ON FIRST Y AXIS
    SearchResult new_hit;
    new_hit = searchRes;
    int yStart = 0;
    while (new_hit.found && unfound == 1 && yStart++ < 5 && searchRes.point.y < car_y_)
    {
        new_hit.point.y++;
        new_hit = find_point(cannied, new_hit.point, Direction::LEFT, Direction::RIGHT);
        if (new_hit.found && int(new_hit.point.x) == int(searchRes.point.x))
        {
            searchRes = new_hit;
        } else
            break;
    }

    if (linef(first_start_, searchRes.point).length2() <= max_dist_from_start_*max_dist_from_start_)
        last_start_ = searchRes.point;

    return searchRes.point;
}


optional<POINT> RoadLineBuilder::get_next_point(const cv::Mat& cannied, float est_angle, const POINT& prev_point,int delta = 2)
{

    POINT it = prev_point;
    SearchResult searchResult;
    int unfound = 0;
    while (!searchResult.found && unfound < max_upwards_iterations_ && it.y > 0)
    {
        it.y-=delta;
        it.x += cosf(est_angle)*delta;
        searchResult = find_point(cannied, it, Direction::LEFT, Direction::RIGHT);
        total_gap_++;
        unfound++;
    }

    if (searchResult.found)
        return searchResult.point;
    else
        return nullptr;
}