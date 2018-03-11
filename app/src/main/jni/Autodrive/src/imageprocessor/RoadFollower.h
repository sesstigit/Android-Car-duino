#include "LineFollower.h"

class RoadFollower
{
 public:
	RoadFollower(const cv::Mat& cannied, int center_x);
	bool left_line_found();
	bool right_line_found();
	bool is_left_lane();
	bool is_right_lane();
	int dashed_line_gaps();
	command update(cv::Mat& cannied, cv::Mat& drawMat);

 private:
	int find_car_end(const cv::Mat& cannied);
	POINT find_line_start(const cv::Mat& cannied, float direction);
	cv::Mat draw(const cv::Mat& cannied);
	
	int car_y_ = 0;
	int center_x_ = 0;
	std::unique_ptr<LineFollower> left_line_follower_ = nullptr;
	std::unique_ptr<LineFollower> right_line_follower_ = nullptr;
	
	std::vector<int> prev_dirs_;
	int unfound_counter_ = 0;
};


