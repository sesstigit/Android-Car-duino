#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_
#define ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_

#include <string.h>
#include <math.h>

#include "Autodrive.h"


//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include "imageprocessor/imageprocessor.hpp"

using namespace cv;

Overtaking::Overtaking() {
	debugMode = true;
	distance_travelled_ = 0;
	overtaking_ = false;
	turn_left_ = 0;
	turn_left_calibration_ = 0;
	oomphturn_left_ = 0;
	turn_right_ = 0;
	turn_right_calibration_ = 0;
	oomphturn_right_ = 0;
	turn_left_calibration_finished_ = false;
	obstacle_distance_ = 70;
	obstacle_met_ = false;
	obstacle_passed_ = false;
	line_LHS_sensor_.value() = false;
	line_RHS_sensor_.value() = false;
	stop_ = false;
}

// ParkingManeuver is a subclass of Autodrive because it is one mode of Autodrive.  
Class Overtaking : public Autodrive {
	bool debugMode;
	int distance_travelled_;
	bool overtaking_;
	int turn_left_;
	int turn_left_calibration_;
	int oomph_turn_left_;
	int turn_right_;
	int turn_right_calibration_;
	int oomph_turn_right_;
	bool turn_left_calibration_finished_;
	int obstacle_distance_;
	bool obstacle_met_;
	bool obstacle_passed_;
	bool line_LHS_sensor_.value();
	bool line_RHS_sensor_.value();
	bool stop_;

	command run(command lastCommand, Mat* mat) {
		motor_.set_value(0.35);
		if (stop_) {
			motor_.set_value(0);
		}

		if (ultrasound_.front.value() > 0 && ultrasound_.front.value() < obstacle_distance_) {
			if (! overtaking) {
				overtaking = true;
				if (! turn_left_) turn_left_ = distance_travelled_;
				SensorData::line_LHS_sensor_.value() = false;
				SensorData::line_RHS_sensor_.value() = false;
			}
		}

		if (overtaking) { // obstacle spotted, start turning left
			if (turn_left_) {
				lastCommand.setAngle(-1);

				if (line_RHS_sensor_.value()) {
					turn_left_ = 0;

					if (! turn_left_calibration_) turn_left_calibration_ = distance_travelled_;
				}

				if (line_LHS_sensor_.value()) {
					turn_left_ = 0;

					if (! oomphturn_left_) oomphturn_left_ = distance_travelled_;
				}
			}

			if (oomphturn_left_) {
				if (distance_travelled_ - oomphturn_left_ > 10) {
					oomphturn_left_ = 0;
					
					if (! turn_left_calibration_) turn_left_calibration_ = distance_travelled_;
				}
			}

			if (turn_left_calibration_) {
				lastCommand.setAngle(1);

				if (distance_travelled_ - turn_left_calibration_ > 15) { // turn a bit to the right for 10cm to calibrate for easier lane following
					turn_left_calibration_ = 0;
					turn_left_calibration__finished_ = true;
					SensorData::line_LHS_sensor_.value() = false;
					SensorData::line_RHS_sensor_.value() = false;
				}
			}

			if (turn_left_calibration__finished_) {
				if (infrared_.rearright.value() > 1 && infrared_.rearright.value() < 21) { // if infrared_.rearright.value() sees something it met the obstacle
					if (! obstacle_met_) {
							turn_left_calibration__finished_ = false;
							obstacle_met_ = true;
						}
				}
			}

			if (obstacle_met_) {
				if (! ultrasound_.frontright.value() || ultrasound_.frontright.value() > 20) { // if ultrasound_.frontright.value() doesn't see anything, it passed the obstacle
					if (! obstacle_passed_) {
						if (! turn_right_) turn_right_ = distance_travelled_; // so start turning right

						obstacle_passed_ = true;
						SensorData::line_LHS_sensor_.value() = false;
						SensorData::line_RHS_sensor_.value() = false;
					}

				}

				if (turn_right_) {
					lastCommand.setAngle(1);

					if (line_LHS_sensor_.value()) {
						turn_right_ = 0;
						if (! turn_right_calibration_) turn_right_calibration_ = distance_travelled_;
					}

					if (line_RHS_sensor_.value()){
						turn_right_ = 0;

						if (! oomphturn_right_) oomphturn_right_ = distance_travelled_;
					}
				}

				if (oomphturn_right_) {
					if (distance_travelled_ - oomphturn_right_ > 6) {
						oomphturn_right_ = 0;
						if (! turn_right_calibration_) turn_right_calibration_ = distance_travelled_;
					}
				}

				if (turn_right_calibration_) {
					lastCommand.setAngle(-1);

					if (distance_travelled_ - turn_right_calibration_ > 20) { // turn a bit to the left for 10cm to calibrate for easier lane following
						turn_right_calibration_ = 0;
						obstacle_met_ = false;
						obstacle_passed_ = false;
						overtaking = false;
					}
				}
			}
		}

		if (debugMode) {
			if (overtaking) {
				cv::putText(*mat, "overtaking", POINT(50.f, mat->size().height / 6.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);

				if (line_LHS_sensor_.value()) {
					cv::putText(*mat, "line LEFT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
				}

				if (line_RHS_sensor_.value()) {
					cv::putText(*mat, "line RIGHT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
				}
			}

			char turningLeftText[50];
			sprintf(turningLeftText, "turning left: %d", turn_left_);
			if (turn_left_) {
				cv::putText(*mat, turningLeftText, POINT(50.f, mat->size().height / 4.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
			}

			char turningRightText[50];
			sprintf(turningRightText, "turning right: %d", turn_right_);
			if (turn_right_) {
				cv::putText(*mat, turningRightText, POINT(50.f, mat->size().height / 4.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
			}

			if (obstacle_met_) {
				cv::putText(*mat, "obstacle met", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
			}

			if (stop_) {
				cv::putText(*mat, "stop_ped", POINT(50.f, mat->size().height / 6.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
			}
		}

		return lastCommand;
	}
}
}