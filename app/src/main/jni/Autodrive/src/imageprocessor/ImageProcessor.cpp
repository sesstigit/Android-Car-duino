ImageProcessor::ImageProcessor(ImageConfig* img_conf) :
	img_conf_(img_conf),
	thresh1_(181),
	thresh2_(71),
	intensity_(110),
	blur_i_(11),
	road_follower_(nullptr),
	perspective_(nullptr),
	start_center_(nullptr)
}

//TODO: why is mat a pointer here, but a reference in next function?
bool ImageProcessor::init_processing(cv::Mat* mat) {
	auto found_pespective = find_perspective(mat);
	if (found_pespective)
	{
		perspective_ = *found_pespective;
		birds_eye_transform(mat, perspective_);
		if (img_conf_->normalize_lighting_)
			normalize_lighting(mat, blur_i_, intensity_ / 100.f);
		cv::Mat cannied_mat;
		cv::Canny(*mat, cannied_mat, thresh1_, thresh2_, 3);
		roadFollower = make_unique<roadfollower>(cannied_mat, mat->size().width / 2.f + centerDiff);
		return true;
	} else{
		cv::putText(*mat, "SEARCHING FOR STRAIGHT LANES...", POINT(50.f, mat->size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		return false;
	}
}


int ImageProcessor::continue_processing(cv::Mat& mat)
{
	birds_eye_transform(&mat, perspective_);
	if (img_conf_->normalize_lighting_)
		normalize_lighting(&mat, blur_i_, intensity_ / 100.f);

	cv::Mat cannied_mat;
	cv::Canny(mat, cannied_mat, thresh1_, thresh2_, 3);

	/* PAINT OVER BORDER ARTEFACTS FROM TRANSFORM*/
	leftImageBorder.draw(cannied_mat, cv::Scalar(0, 0, 0), Settings::transformLineRemovalThreshold);
	rightImageBorder.draw(cannied_mat, cv::Scalar(0, 0, 0), Settings::transformLineRemovalThreshold);
	
	//TODO: get rid of command object.  But then how can we tell whethter the angle has changed?
	command cmnd = roadFollower->update(cannied_mat, mat);
	float angle =  Direction::FORWARD;

	if (cmnd.changedAngle)
	{
		//TODO: *15 really needed
		angle = ((90 - cmnd.angle*15)* Autodrive::Mathf::PI) / 180.f ;
	}

	POINT center(mat.size().width / 2.f, (float) mat.size().height);
	Autodrive::linef(center, center + POINT(std::cos(angle) * 200, -sin(angle) * 200)).draw(mat, CV_RGB(0, 250, 0));
	return cmnd;
}

bool ImageProcessor::left_line_found()
{
	return roadFollower->left_line_found();
}

bool ImageProcessor::right_line_found()
{
	return roadFollower->right_line_found();
}

/*
	Returns wether the car is on the left lane
	Currently only works if both roadlines are found by comparing their gaps
*/
bool ImageProcessor::is_left_lane()
{
	if (! left_line_found() || ! right_line_found())
		return false;
	else
		return roadFollower->is_left_lane();
}

/*
Returns wether the car is on the right lane
*/
bool ImageProcessor::is_right_lane()
{
	if (! left_line_found() || ! right_line_found())
		return false;
	else
		return roadFollower->is_right_lane();
}

int ImageProcessor::dashed_line_gaps() {
	return roadFollower->dashed_line_gaps();
}

void normalize_lighting(cv::Mat* bgr_image,int blur = 20,float intensity = 0.5f)
{
	cv::Mat light_mat;
	cv::blur(*bgr_image, light_mat, cv::Size(blur, blur));
	cv::cvtColor(light_mat, light_mat, CV_BGR2GRAY);

	cv::Mat lab_image;
	cv::cvtColor(*bgr_image, *bgr_image, CV_BGR2Lab);

	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(*bgr_image, lab_planes);  // now we have the L image in lab_planes[0]

	lab_planes[0] = lab_planes[0] - light_mat*intensity;

	// Merge the the color planes back into an Lab image
	cv::merge(lab_planes, *bgr_image);

	// convert back to RGB
	cv::cvtColor(*bgr_image, *bgr_image, CV_Lab2BGR);
}