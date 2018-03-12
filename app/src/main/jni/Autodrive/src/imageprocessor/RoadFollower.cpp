#include "RoadFollower.h"

RoadFollower::RoadFollower(const cv::Mat& cannied, int center_x) : center_x_(center_x) {
	car_y_ = find_car_end(cannied);
	POINT right_line_start = find_line_start(cannied, Direction::RIGHT);
	POINT left_line_start = find_line_start(cannied, Direction::LEFT);

	left_line_follower_ = make_unique<LineFollower>(cannied, left_line_start, center_x_,car_y_);
	right_line_follower_ = make_unique<LineFollower>(cannied, right_line_start, center_x_,car_y_);
}

// TODO: main function here.  Fix how the function returns a command object.  Also fix Settings.  Also fix case of variables.
command update(cv::Mat& cannied, cv::Mat& drawMat)
{
	command cmd;

	left_line_follower_->update(cannied);
	right_line_follower_->update(cannied);

	drawMat = draw(cannied);

	optional<int> leftTargetAngle = left_line_follower_->getPreferedAngle();
	optional<int> rightTargetAngle = right_line_follower_->getPreferedAngle();
	optional<int> targetAngle = nullptr;

	if (leftTargetAngle && rightTargetAngle && Settings::useLeftLine)
	{
		// Give the right line just a bit more priority since it seems more reliable
		targetAngle = weighted_average(*rightTargetAngle, *leftTargetAngle, 3);
	} else if (leftTargetAngle && Settings::useLeftLine)
	{
		targetAngle = *leftTargetAngle;
	} else if (rightTargetAngle)
	{
		targetAngle = *rightTargetAngle;
	}else if(unfound_counter_++ > 5)
	{
		 cmd.setAngle(0);
	}
	
	if(targetAngle)
	{
		unfound_counter_ = 0;
		if(Settings::smoothening == 0)
		{
			cmd.setAngle(*targetAngle / 25.0);
		}
		else
		{
			int sum = (std::accumulate(prev_dirs_.begin(), prev_dirs_.end(), 0) + *targetAngle);
			int newAngle = sum / float(prev_dirs_.size() + 1);
			prev_dirs_.push_back(newAngle);
			if(prev_dirs_.size() > Settings::smoothening)
				prev_dirs_.erase(prev_dirs_.begin());
			
			cmd.setAngle(newAngle  / 25.0);
		}
	}
	

	return cmd;
}
	
int RoadFollower::find_car_end(const cv::Mat& cannied)
{
	POINT center_bottom(center_x_, cannied.size().height - 8);
	//SEARCH UPWARDS UNTIL _NOT_ HIT ON THE CENTER +/- 10
	bool hit = true;
	while (hit)
	{
		hit = firstnonzero_direction(cannied, center_bottom, Direction::RIGHT, 10).found
			|| firstnonzero_direction(cannied, center_bottom, Direction::LEFT, 10).found;
		if (hit)
			center_bottom.y--;
	}
	center_bottom.y--;
	return center_bottom.y;
}


POINT RoadFollower::find_line_start(const cv::Mat& cannied, float direction)
{
	POINT iter(center_x_, car_y_);
	SearchResult searchRes;
	//SEARCH UPWARDS UNTIL HIT ON THE RIGHT or LEFT (dependant on direction value)
	while (!searchRes.found)
	{
		searchRes = firstnonzero_direction(cannied, iter, direction, 360);// 0 = RIGHT
		if (!searchRes.found)
			iter.y--;
	}
	return searchRes.point;
}

cv::Mat RoadFollower::draw(const cv::Mat& cannied)
{
	cv::Mat colorCopy;
	cv::cvtColor(cannied, colorCopy, CV_GRAY2RGB);

	left_line_follower_->draw(&colorCopy,center_x_);
	right_line_follower_->draw(&colorCopy, center_x_);

	return colorCopy;
}

bool RoadFollower::left_line_found() {
	return left_line_follower_->is_found();
}

bool RoadFollower::right_line_found() {
	return right_line_follower_->is_found();
}

bool RoadFollower::is_left_lane()	{
	int left_gaps = left_line_follower_->totalGap();
	int right_gaps = right_line_follower_->totalGap();
	return (left_gaps < right_gaps);
}

bool RoadFollower::is_right_lane() {
	return (!is_left_lane());
}

int RoadFollower::dashed_line_gaps() {
	if (left_line_found() && ! right_line_found()) {
		return left_line_follower_->total_gap();
	} else if (right_line_found() && ! left_line_found()) {
		return right_line_follower_->total_gap();
	} else {
		return 0;
	}
}