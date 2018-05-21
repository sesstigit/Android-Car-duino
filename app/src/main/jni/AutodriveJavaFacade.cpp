#include <jni.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//TODO: Only enable when debugging emulator
//#define _DEBUG
#undef _DEBUG
#include "Autodrive.h"
#ifdef USE_IMAGEPROCESSOR
#include "imageprocessor/ImageProcessor.h"
#else
#include "histogram/AdvImageProcessor.h"
#endif
#include "Car.h"
#include "CarCmd.h"
#include "CarSensor.h"
#include "ParkingManeuver.h"
#include "Overtaking.h"

using namespace std;
using namespace cv;
using namespace Autodrive;

#define TYPE(x) JNIEXPORT x JNICALL
#define NAME(x) Java_pegasus_bluetootharduino_Autodrive_##x
#define PARAMS(...) (JNIEnv*,jobject , __VA_ARGS__)

extern "C" 
{
    TYPE(void) NAME(drive)()
    {
		//Autodrive::Car* pcar = Autodrive::get_pcar();
        //pcar->drive();
		Autodrive::car.drive();
    }
	
    TYPE(void) NAME(reset)()
    {
        Autodrive::car.reset_mode();
    }

    TYPE(void) NAME(setParkingMode)()
    {
        Autodrive::car.set_initial_mode(Autodrive::AutoDriveMode::kDetectingGap);
    }
    
    TYPE(void) NAME(resetParking)()
    {
        Autodrive::car.parking_->reset();
    }

    TYPE(void) NAME(setCarLength) PARAMS(int _carLength)
    {
        Autodrive::car.car_length_ = _carLength;
    }

    TYPE(void) NAME(lineLeftFound)()
    {
        Autodrive::car.line_LHS_sensor_.set_value(true);
    }

    TYPE(void) NAME(lineRightFound)()
    {
        Autodrive::car.line_RHS_sensor_.set_value(true);
    }
    
    TYPE(jint) NAME(getCarLength)()
    {
        return Autodrive::car.car_length_;
    }

    TYPE(void) NAME(setPerspective) PARAMS(long p)
    {
	Autodrive::car.img_proc_->set_perspective((cv::Mat*)p);
    }

    TYPE(void) NAME(deletePerspective)()
    {
	Autodrive::car.img_proc_->delete_perspective();
    }

	//TYPE(void) NAME(setImage) PARAMS(long newMat) {
	//	Autodrive::car.image_ = (cv::Mat*)newMat;
    TYPE(void) NAME(getPerspective) PARAMS(long retMat)
    {
		cv::Mat& tempMat = *(cv::Mat*)Autodrive::car.img_proc_->get_perspective();

		tempMat.copyTo(*(cv::Mat*)retMat);	
		//Mat& mat = *(Mat*)addrMat;
		//Mat& newMat = *(Mat*)addrNewMat;
		//mat.copyTo(newMat);
    }

   /*----- DEBUGDATA -----*/
   
   // parking

   TYPE(jboolean)NAME(isGapDepthOk)()
   {
        return Autodrive::car.parking_->gap_depth_ok();
   }

   TYPE(jboolean)NAME(isInitialGap)()
   {
        return Autodrive::car.parking_->initial_gap();
   }

   
   TYPE(jint) NAME(gapLength)()
  	{
	    return Autodrive::car.parking_->gap_length();
	}
    
    TYPE(jint) NAME(angleTurned)()
    {
        return Autodrive::car.parking_->turned_angle();
    }
    
    TYPE(jint)NAME(getManeuver)()
    {
        switch(Autodrive::car.parking_->mode())
        {
            case Autodrive::ParkingManeuverMode::kNoManeuver:
                return 0;
            case Autodrive::ParkingManeuverMode::kParallelStandard:
                return 1;
            case Autodrive::ParkingManeuverMode::kParallelWide:
                return 2;
            case Autodrive::ParkingManeuverMode::kPerpendicularStandard:
                return 3;
            default:
                return -1;
        }
    }
    
    TYPE(jint)NAME(getManeuverState)()
    {
        switch(Autodrive::car.parking_->current_state())
        {
            case Autodrive::ParkingManeuverState::kNotMoving:
                return 0;
            case Autodrive::ParkingManeuverState::kForward:
                return 1;
            case Autodrive::ParkingManeuverState::kBackward:
                return 2;
            case Autodrive::ParkingManeuverState::kForwardRight:
                return 3;
            case Autodrive::ParkingManeuverState::kBackwardRight:
                return 4;
            case Autodrive::ParkingManeuverState::kForwardLeft:
                return 5;
            case Autodrive::ParkingManeuverState::kBackwardLeft:
                return 6;
            case Autodrive::ParkingManeuverState::kDone:
                return 7;
			//case Autodrive::ParkingManeuverState::kEmergency:
            //    return 8;
            default:
                return -1;
        }
    }
    
    /*----- SENSORDATA -----*/
    
    // getters - for debuging purposes
    
    TYPE(jint)NAME(usFront)()
    {
        return Autodrive::car.ultrasound_.front.value();
    }
    
    TYPE(jint)NAME(usFrontRight)()
    {
        return Autodrive::car.ultrasound_.frontright.value();
    }
    
    TYPE(jint)NAME(usRear)()
    {
        return Autodrive::car.ultrasound_.rear.value();
    }
    
    TYPE(jint)NAME(irFrontRight)()
    {
        return Autodrive::car.infrared_.frontright.value();
    }
    
    TYPE(jint)NAME(irRearRight)()
    {
        return Autodrive::car.infrared_.rearright.value();
    }
    
    TYPE(jint)NAME(irRear)()
    {
        return Autodrive::car.infrared_.rear.value();
    }
    
    TYPE(jint)NAME(gyroHeading)()
    {
        return Autodrive::car.gyro_.value();
    }
    
    TYPE(jint)NAME(razorHeading)()
    {
        return Autodrive::car.gyro_.value(); //FIX TODO we dont have a razor gyro, so just return normal gyro reading.
    }
    
   // setters

    TYPE(void) NAME(setImage) PARAMS(long newMat){
        Autodrive::car.image_ = (cv::Mat*)newMat;
    }
	
    TYPE(void) NAME(setUltrasound) PARAMS(int sensor,int value){
        switch (sensor)
        {
            case 0:
                Autodrive::car.ultrasound_.front.set_value(value);
                break;
            case 1:
                Autodrive::car.ultrasound_.frontright.set_value(value);
                break;
            default:
            case 2:
                Autodrive::car.ultrasound_.rear.set_value(value);
                break;
        }
    }

    TYPE(void) NAME(setInfrared) PARAMS(int sensor,int value){
        switch (sensor)
        {
            case 0:
                Autodrive::car.infrared_.frontright.set_value(value);
                break;
            case 1:
                Autodrive::car.infrared_.rearright.set_value(value);
                break;
            default:
            case 2:
                Autodrive::car.infrared_.rear.set_value(value);
                break;
        }

    }
	
    TYPE(void) NAME(setEncoderPulses) PARAMS(long value){
        Autodrive::car.distance_.set_value(value);
    }
    
    TYPE(void) NAME(setGyroHeading) PARAMS(int value){
        Autodrive::car.gyro_.set_value(value);
    }
    
    TYPE(void) NAME(setRazorHeading) PARAMS(int value){
        Autodrive::car.gyro_.set_value(value);  //FIX: we dont have a razor gyro.  If we get one, set value to normal gyro?
    }
    
	
	/*----- RESULTING AUTODRIVE DATA -----*/
	
    TYPE(jboolean) NAME(speedChanged)() 
    {
        return Autodrive::car.changed_speed();
    }

    TYPE(jboolean) NAME(angleChanged)()
    {
        return Autodrive::car.changed_angle();
    }

    TYPE(jdouble) NAME(getTargetSpeed)()
    {
        return Autodrive::car.motor_.value();
    }

    TYPE(jdouble) NAME(getTargetAngle)()
    {
        return Autodrive::car.steering_.value();
    }
	
	/* SETTINGS */
	
    TYPE(void) NAME(setSettingLightNormalization) PARAMS(bool on)
    {
        Autodrive::car.img_conf_.normalize_lighting_ = on;
    }
    
    TYPE(void) NAME(setSettingUseLeftLine) PARAMS(bool on)
    {
        Autodrive::car.img_conf_.use_left_line_ = on;
    }
    
    TYPE(void) NAME(setSettingSmoothening) PARAMS(int value)
    {
        Autodrive::car.img_conf_.smoothening_ = value;
    }

    TYPE(void) NAME(setSettingFirstFragmentMaxDist) PARAMS(int value)
    {
        Autodrive::car.img_conf_.first_fragment_max_dist_ = value;
    }

    TYPE(void) NAME(setSettingLeftIterationLength) PARAMS(int value)
    {
        Autodrive::car.img_conf_.left_iteration_length_ = value;
    }

    TYPE(void) NAME(setSettingRightIterationLength) PARAMS(int value)
    {
        Autodrive::car.img_conf_.right_iteration_length_ = value;
    }

    TYPE(void) NAME(setSettingMaxAngleDiff) PARAMS(float value)
    {
        Autodrive::car.img_conf_.max_angle_diff_ = value;
    }

    TYPE(void) NAME(setCannyThresh) PARAMS(int value)
    {
        Autodrive::car.img_conf_.canny_thresh_ = value;
    }
    
    TYPE(void) NAME(setCarScaleDriftFix) PARAMS(float value)
    {
        Autodrive::car.img_conf_.car_scale_drift_fix_ = value;
    }
	TYPE(void) NAME(setPidKp) PARAMS(float value)
	{
		Autodrive::car.img_conf_.pid_kp_ = (double)value;
	}
	TYPE(void) NAME(setPidKd) PARAMS(float value)
	{
		Autodrive::car.img_conf_.pid_kd_ = (double)value;
	}
	TYPE(void) NAME(setPidKi) PARAMS(float value)
	{
		Autodrive::car.img_conf_.pid_ki_ = (double)value;
	}
}

#undef TYPE
#undef NAME
#undef PARAMS
