
const float Mathf::PI = acosf(-1);
const float Mathf::PI_2 = PI / 2.f;

const float Direction::RIGHT = 0.f;
const float Direction::LEFT = Mathf::PI;
const float Direction::FORWARD = Mathf::PI_2;

    
#ifndef __ANDROID__
void show_image(cv::Mat mat, int resize, std::string wName)
{
	cv::resize(mat, mat, mat.size() * resize);//resize image
	cv::imshow(wName, mat);
}
#endif
    
SearchResult firstnonzero_direction(const cv::Mat& mat, cv::Point_ < float > start, float direction, int maxDist)
{
	SearchResult res;
	POINT new_pos = start + POINT(::std::cos(direction) * maxDist, -::std::sin(direction) * maxDist);
	cv::LineIterator it(mat, start, new_pos);
	for (int i = 0; i < it.count; i++, it++)
	{
		if (mat.at<uchar>(it.pos()))
		{
			res.found = true;
			res.distance = i;
			res.point = it.pos();
			break;
		}
	}
	return res;
}

optional<cv::Point> firstnonzero_horizontal(const cv::Mat& mat, cv::Point iterator)
{
	while (iterator.x < mat.size().width - 1)
	{
		if (mat.at<uchar>(iterator) != 0)
		{
			return iterator;
		}
		iterator.x++;
	}
	return nullptr;
}

    
numeric_t weighted_average(numeric_t val1, numeric_t val2, numeric_t val1_multiplier)
{
	return (val1*val1_multiplier + val2) / (val1_multiplier + 1);
}
