#include <jni.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//TODO: Only enable when debugging emulator
//#define _DEBUG
#undef _DEBUG
#include "Autodrive.h"
#include "imageprocessor/ImageProcessor.h"
#include "Car.h"
//#include "Autodrive/src/Autodrive.h"
//#include "Autodrive/src/Car.h"
//#include "Autodrive/src/CarCmd.h"
//#include "Autodrive/src/CarSensor.h"
//#include "Autodrive/src/ParkingManeuver.h"
//#include "Autodrive/src/Overtaking.h"

using namespace std;
using namespace cv;
using namespace Autodrive;

#define TYPE(x) JNIEXPORT x JNICALL
#define NAME(x) Java_pegasus_bluetootharduino_Autodrive_##x
#define PARAMS(...) (JNIEnv*,jobject , __VA_ARGS__)

extern "C" 
{
	TYPE(jlong) NAME(createCar)() {
		Autodrive::Car* pCar = new Autodrive::Car();
		return (long)pCar;
	}

	TYPE(void) NAME(drive)(jlong lp) {
		Autodrive::Car* pCar = (Car*)lp;
		pCar->drive();
	}
	
	TYPE(void) NAME(drive3)(jlong lp) {
		Autodrive::Car* pCar = (Car*)lp;
		pCar->drive();
	}
	
    TYPE(void) NAME(drive2)()
    {
        Autodrive::car.drive();
    }
	
    TYPE(void) NAME(reset)()
    {
        return Autodrive::car.reset_mode();
    }

    TYPE(void) NAME(setParkingMode)()
    {
        Autodrive::car.set_initial_mode(Autodrive::AutoDriveMode::kDetectingGap);
    }
    
    TYPE(void) NAME(resetParking)()
    {
        Autodrive::car.parking()->reset();
    }

    TYPE(void) NAME(setLeftLane) PARAMS(bool boolean)
    {
        Autodrive::car.img_conf()->use_left_line_ = boolean;
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

   /*----- DEBUGDATA -----*/
   
   // parking

   TYPE(jboolean)NAME(isGapDepthOk)()
   {
        return Autodrive::car.parking()->gap_depth_ok();
   }

   TYPE(jboolean)NAME(isInitialGap)()
   {
        return Autodrive::car.parking()->initial_gap();
   }

   
   TYPE(jint) NAME(gapLength)()
  	{
	    return Autodrive::car.parking()->gap_length();
	}
    
    TYPE(jint) NAME(angleTurned)()
    {
        return Autodrive::car.parking()->turned_angle();
    }
    
    TYPE(jint)NAME(getManeuver)()
    {
        switch(Autodrive::car.parking()->mode())
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
        switch(Autodrive::car.parking()->current_state())
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
    
    //TYPE(jint)NAME(razorHeading)()
    //{
    //    return Autodrive::SensorData::razorHeading;
    //}
    
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
    
    //TYPE(void) NAME(setRazorHeading) PARAMS(int value){
    //    Autodrive::SensorData::razorHeading = value;
    //}
    
	
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
        Autodrive::car.img_conf()->normalize_lighting_ = on;
    }
    
	//Note: interpreted the same as setLeftLane (same meaning???)
    TYPE(void) NAME(setSettingUseLeftLine) PARAMS(bool on)
    {
        Autodrive::car.img_conf()->use_left_line_ = on;
    }
    
    TYPE(void) NAME(setSettingSmoothening) PARAMS(int value)
    {
        Autodrive::car.img_conf()->smoothening_ = value;
    }

    TYPE(void) NAME(setSettingFirstFragmentMaxDist) PARAMS(int value)
    {
        Autodrive::car.img_conf()->first_fragment_max_dist_ = value;
    }

    TYPE(void) NAME(setSettingLeftIterationLength) PARAMS(int value)
    {
        Autodrive::car.img_conf()->left_iteration_length_ = value;
    }

    TYPE(void) NAME(setSettingRightIterationLength) PARAMS(int value)
    {
        Autodrive::car.img_conf()->right_iteration_length_ = value;
    }

    TYPE(void) NAME(setSettingMaxAngleDiff) PARAMS(float value)
    {
        Autodrive::car.img_conf()->max_angle_diff_ = value;
    }

    TYPE(void) NAME(setPIDkp) PARAMS(float value)
    {
        Autodrive::car.img_conf()->kp_ = value;
    }

    TYPE(void) NAME(setPIDki) PARAMS(float value)
    {
        Autodrive::car.img_conf()->ki_ = value;
    }

    TYPE(void) NAME(setPIDkd) PARAMS(float value)
    {
        Autodrive::car.img_conf()->kd_ = value;
    }
}

#undef TYPE
#undef NAME
#undef PARAMS
