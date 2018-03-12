#include "ImageProcessor.h"
#include "Line.h"
#include "roadlinebuilder.hpp"

#include <chrono>

class LineFollower
{
 public:
	LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x,int carY);
	void update(cv::Mat& cannied);

 private:
    // params
	RoadLine road_line_;
	std::unique_ptr<RoadLineBuilder> road_builder_;
    int road_size_ = 40;
	bool is_found_ = false;
    float target_road_distance_;
    
	// Private methods
	void draw(cv::Mat* colorCopy, int centerX);
    bool is_found();
    float distance_deviation();
    int total_gap();
    optional<int> get_prefered_angle();
};
