#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//TODO: Only enable when debugging emulator
#define _DEBUG
#include "Autodrive/Include/autodrive.hpp"
#include "Autodrive/Include/sensordata.hpp"
#include "Autodrive/Include/maneuver.hpp"

using namespace std;
using namespace cv;

#define TYPE(x) JNIEXPORT x JNICALL
#define NAME(x) Java_pegasus_bluetootharduino_Autodrive_##x
#define PARAMS(...) (JNIEnv*,jobject , __VA_ARGS__)

extern "C" 
{
    TYPE(void) NAME (drive)()
    {
        Autodrive::car.drive();
    }
	
    TYPE(void) NAME(reset)()
    {
        return Autodrive::car.reset_mode();
    }

    TYPE(void) NAME(setParkingMode)()
    {
        Autodrive::setInitialStatus(Autodrive::DETECTING_GAP);
    }
    
    TYPE(void) NAME(resetParking)()
    {
        Autodrive::Parking::Reset();
    }

    TYPE(void) NAME(setLeftLane) PARAMS(bool boolean)
    {
        Autodrive::Status::setLeftLane(boolean);
    }
    
    TYPE(void) NAME(setCarLength) PARAMS(int _carLength)
    {
        Autodrive::car.car_length_ = _carLength;
    }

    TYPE(void) NAME(lineLeftFound)()
    {
        Autodrive::SensorData::lineLeftFound = true;
    }

    TYPE(void) NAME(lineRightFound)()
    {
        Autodrive::SensorData::lineRightFound = true;
    }
    
    TYPE(jint) NAME(getCarLength)()
    {
        return Autodrive::car.car_length_;
    }

   /*----- DEBUGDATA -----*/
   
   // parking

   TYPE(jboolean)NAME(isGapDepthOk)()
   {
        return Autodrive::Parking::gapDepthOk;
   }

   TYPE(jboolean)NAME(isInitialGap)()
   {
        return Autodrive::Parking::initialGap;
   }

   
   TYPE(jint) NAME(gapLength)()
  	{
	    return Autodrive::Parking::gapLength;
	}
    
    TYPE(jint) NAME(angleTurned)()
    {
        return Autodrive::Status::currentAngle;
    }
    
    TYPE(jint)NAME(getManeuver)()
    {
        switch(Autodrive::Parking::currentManeuver.type)
        {
            case Autodrive::NO_MANEUVER:
                return 0;
            case Autodrive::PARALLEL_STANDARD:
                return 1;
            case Autodrive::PARALLEL_WIDE:
                return 2;
            case Autodrive::PERPENDICULAR_STANDARD:
                return 3;
            default:
                return -1;
        }
    }
    
    TYPE(jint)NAME(getManeuverState)()
    {
        switch(Autodrive::Parking::currentManeuver.currentState)
        {
            case Autodrive::maneuver::mState::NOT_MOVING:
                return 0;
            case Autodrive::maneuver::mState::FORWARD:
                return 1;
            case Autodrive::maneuver::mState::BACKWARD:
                return 2;
            case Autodrive::maneuver::mState::FORWARD_RIGHT:
                return 3;
            case Autodrive::maneuver::mState::BACKWARD_RIGHT:
                return 4;
            case Autodrive::maneuver::mState::FORWARD_LEFT:
                return 5;
            case Autodrive::maneuver::mState::BACKWARD_LEFT:
                return 6;
            case Autodrive::maneuver::mState::DONE:
                return 7;
            default:
                return -1;
        }
    }
    
    /*----- SENSORDATA -----*/
    
    // getters - for debuging purposes
    
    TYPE(jint)NAME(usFront)()
    {
        return Autodrive::car.ultrasound_.front.get_value();
    }
    
    TYPE(jint)NAME(usFrontRight)()
    {
        return Autodrive::car.ultrasound_.frontright.get_value();
    }
    
    TYPE(jint)NAME(usRear)()
    {
        return Autodrive::car.ultrasound_.rear.get_value();
    }
    
    TYPE(jint)NAME(irFrontRight)()
    {
        return Autodrive::car.infrared_.frontright.get_value();
    }
    
    TYPE(jint)NAME(irRearRight)()
    {
        return Autodrive::car.infrared_.rearright.get_value();
    }
    
    TYPE(jint)NAME(irRear)()
    {
        return Autodrive::car.infrared_.rear.get_value();
    }
    
    TYPE(jint)NAME(gyroHeading)()
    {
        return Autodrive::car.gyro_.get_value();
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
                Autodrive::SensorData::infrared_.rear.set_value(value);
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
        return Autodrive::speedChanged();
    }

    TYPE(jboolean) NAME(angleChanged)()
    {
        return Autodrive::angleChanged();
    }

    TYPE(jdouble) NAME(getTargetSpeed)()
    {
        return Autodrive::getSpeed();
    }

    TYPE(jdouble) NAME(getTargetAngle)()
    {
        return Autodrive::getAngle();
    }
	
	/* SETTINGS */
	
    TYPE(void) NAME(setSettingLightNormalization) PARAMS(bool on)
    {
        Autodrive::Settings::normalizeLightning = on;
    }
    
    TYPE(void) NAME(setSettingUseLeftLine) PARAMS(bool on)
    {
        Autodrive::Settings::useLeftLine = on;
    }
    
    TYPE(void) NAME(setSettingSmoothening) PARAMS(int value)
    {
        Autodrive::Settings::smoothening = value;
    }

    TYPE(void) NAME(setSettingFirstFragmentMaxDist) PARAMS(int value)
    {
        Autodrive::Settings::firstFragmentMaxDist = value;
    }

    TYPE(void) NAME(setSettingLeftIterationLength) PARAMS(int value)
    {
        Autodrive::Settings::leftIterationLength = value;
    }

    TYPE(void) NAME(setSettingRightIterationLength) PARAMS(int value)
    {
        Autodrive::Settings::rightIterationLength = value;
    }

    TYPE(void) NAME(setSettingMaxAngleDiff) PARAMS(float value)
    {
        Autodrive::Settings::maxAngleDiff = value;
    }

    TYPE(void) NAME(setPIDkp) PARAMS(float value)
    {
        Autodrive::Settings::kp = value;
    }

    TYPE(void) NAME(setPIDki) PARAMS(float value)
    {
        Autodrive::Settings::ki = value;
    }

    TYPE(void) NAME(setPIDkd) PARAMS(float value)
    {
        Autodrive::Settings::kd = value;
    }
}

#undef TYPE
#undef NAME
#undef PARAMS
