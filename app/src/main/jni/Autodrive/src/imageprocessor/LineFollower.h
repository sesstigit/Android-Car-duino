#include "ImageProcessor.h"
#include "Line.h"
#include "roadlinebuilder.hpp"

#include <chrono>

class LineFollower
{
 public:
	LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x,int carY);
 
	float target_road_distance_ = 0;
	std::unique_ptr<roadlinebuilder> road_builder_ = nullptr;
	void draw(cv::Mat* colorCopy, int centerX);
	bool is_found();
	float distance_deviation();
	int total_gap();
	optional<int> get_prefered_angle();
	void update(cv::Mat& cannied);
	

 private:
	RoadLine road_line_;
	int road_size_ = 40;
	bool is_found_ = false;
};
