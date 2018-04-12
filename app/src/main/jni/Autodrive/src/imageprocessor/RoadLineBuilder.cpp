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

//! These parameters should be configurable from the Android App.
//!max_dist_from_start_ was 22
//!step_distance was 4
//!max_upwards_iterations was 100
RoadLineBuilder::RoadLineBuilder(POINT start_point, float center_x, int car_y, const ImageConfig& img_conf) :
    first_start_(start_point), last_start_(start_point), center_x_(center_x),
	car_y_(car_y), img_conf_(img_conf), step_dist_(1), 
	max_dist_from_start_(50), max_upwards_iterations_(20), total_gap_(0) {
}

//! Called by LineFollower->update() on each line of the road.
std::unique_ptr<RoadLine> RoadLineBuilder::build(const cv::Mat& cannied, size_t maxsize) {
	// Create a RoadLine, specifying the center_x_ point, and a start_point (calculated as get_first_point())
	std::unique_ptr<RoadLine> road = make_unique<RoadLine>(center_x_, get_first_point(cannied), img_conf_);
    optional<POINT> new_point;
    total_gap_ = 0;
	// Keep adding points to the RoadLine up to maxsize (the road size)
    while ((new_point = get_next_point(cannied, road->get_estimated_angle(), road->back_points(), step_dist_)).valid && road->num_points() < maxsize)
    {
        if (!road->add_point(*new_point))
            break;
    }
    road->set_total_gap(total_gap_);

    return road;
}

//TODO: was "static" on next line    
SearchResult RoadLineBuilder::find_point(const cv::Mat& cannied, POINT start, float left_angle, float fight_angle, float iteration_reduction) {
	SearchResult right_search = firstnonzero_direction(cannied, start, fight_angle, static_cast<int>(img_conf_.right_iteration_length_ - iteration_reduction)); //!< find a white point in given direction
    SearchResult left_search = firstnonzero_direction(cannied, start, left_angle, static_cast<int>(img_conf_.left_iteration_length_ - iteration_reduction));
    //! OK to find either line, or both.
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

// Calculate the first point in a RoadLine.
POINT RoadLineBuilder::get_first_point(const cv::Mat& cannied)
{
    SearchResult searchRes;
    int unfound = 0;  //!< incremented each time a search fails
	// Begin search from previously found point.
    POINT start_point = last_start_;

    //SEARCH UPWARDS UNTIL HIT (end search when firstFragmentMaxDist reached).  HIT is a white point.
    while (!searchRes.found && unfound++ < img_conf_.first_fragment_max_dist_)
    {
		//! Search both to left and right of start_point for a Canny edge detection white line.
        searchRes = find_point(cannied, start_point, Direction::LEFT, Direction::RIGHT,img_conf_.iterate_reduce_on_start_);
        if (!searchRes.found)
            start_point.y--;
    }

    // SEARCH DOWNWARDS IF HIT ON FIRST Y AXIS
    SearchResult new_hit;
    new_hit = searchRes;  //!< new_hit is initialised to the point found above (which has found=true)
    int yStart = 0;
    //!< unfound==1 means the search upwards was successful first time
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

	//! Set last_start_ as state for the next search
	if (searchRes.found) {
		if (linef(first_start_, searchRes.point).length2() <= max_dist_from_start_*max_dist_from_start_)
			last_start_ = searchRes.point;
	} else {
		//cannot find the line, so move slowly back to first_start_ to avoid getting stuck searching a long way from the start.
		if (last_start_.x > first_start_.x) {
			last_start_.x--;
		} else if (last_start_.x < first_start_.x) {
			last_start_.x++;
		}
		if (last_start_.y > first_start_.y) {
			last_start_.y--;
		}
		else if (last_start_.y < first_start_.y) {
			last_start_.y++;
		}
	}
    return searchRes.point;
}

//! Get next point in a line based on the prev_point, an est_angle to search, and a step_distance to the next point.  Search left and right at each step.
optional<POINT> RoadLineBuilder::get_next_point(const cv::Mat& cannied, float est_angle, const POINT& prev_point,int step_dist = 2)
{

    POINT it = prev_point;
    SearchResult searchResult;
    int unfound = 0;
    while (!searchResult.found && unfound < max_upwards_iterations_ && it.y > 0)
    {
        it.y-=step_dist;
        it.x += cosf(est_angle)*step_dist;
        searchResult = find_point(cannied, it, Direction::LEFT, Direction::RIGHT);
        total_gap_++;  //increments on each search step
        unfound++;
    }

    if (searchResult.found)
        return searchResult.point;
    else
        return nullptr;
}