#include "LineFollower.h"

LineFollower::LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x,int carY)
{
	road_builder_ = make_unique<roadlinebuilder>(laneStartPoint, center_x, carY);
	roadLine = road_builder_->build(cannied, roadsize);
	target_road_distance_ = roadLine.getMeanStartDistance(5);
}

//TODO: this function uses Setttings from settings.h which are now in ImageProcessor.h
// How should the settings be accessed from here???
void LineFollower::draw(cv::Mat* colorCopy, int centerX)
{
	road_line_.draw(colorCopy);
   
	/* DRAW RECTANGLE FOR POSSIBLE FIRST HITS*/
	POINT upperLeft = road_builder_->last_start + POINT(-Settings::leftIterationLength, Settings::firstFragmentMaxDist);
	POINT lowerRight = road_builder_->last_start + POINT(Settings::rightIterationLength, 0);
	cv::rectangle(*colorCopy,upperLeft , lowerRight,cv::Scalar(255,0,255));
	
	//linef(road_builder_->last_start, road_builder_->last_start + POINT(8, -20)).draw(colorCopy, cv::Scalar(0, 255, 255), 1);

	/* DRAW VERTICAL LINE DISPLAYING DISTANCE TO ROAD AND TARGETED DISTANCE TO ROAD*/
	POINT offsetX = POINT(target_road_distance_,0);
	POINT bottom_center = POINT(centerX, colorCopy->size().height);
	linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(*colorCopy);
	offsetX.x = roadLine.getMeanStartDistance(5);
	if (int(offsetX.x) != 0)
		linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(*colorCopy, cv::Scalar(255, 255, 0));
}


// Prerequicite for wheter a road is found or not
bool LineFollower::is_found()
{
	return is_found_;
}

float LineFollower::distance_deviation()
{
	if(!is_found())
		return target_road_distance_;
	float startDistance = road_line_.getMeanStartDistance(4);
	return (startDistance - target_road_distance_) * 1.1f;
}

int LineFollower::total_gap()
{
	return road_line_.totalGap / road_line_.points.size();
}

optional<int> LineFollower::get_prefered_angle()
{
	if (is_found())
	{
		/* Start by setting the target angle to the mean road angle*/
		int degrees = Mathf::toDegrees(road_line_.getMeanAngle(4)) - 90;
		degrees = int((degrees / 65.f) * 25);
		degrees *= -1;
		
		degrees+=distanceDeviation();
		
		degrees = std::min(degrees, 25);
		degrees = std::max(degrees, -25);
		return degrees;
	}
	return nullptr;
}

void update(cv::Mat& cannied)
{
	road_line_ = road_builder_->build(cannied, road_size_);
	is_found_ = (road_line_.points.size() > 5 && fabs(road_line_.getMeanAngle() - Direction::FORWARD) < Mathf::PI_2);
}