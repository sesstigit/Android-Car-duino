/**
*    This file is part of Android-Car-duino.
*
*    Android-Car-duino is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Android-Car-duino is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Android-Car-duino.  If not, see <http://www.gnu.org/licenses/>.
**/
package pegasus.bluetootharduino;

import org.opencv.core.Mat;

/**
 * Created by David on 2015-04-07.
 */
//! This class is an interface to the C++ autodrive library.
//! It loads some libraries including autodrive.
//! e.g. When auto driving mode is clicked in the App GUI, then
//! Java function drive() in this class calls the drive() function 
//! from the autodrive library.
public class Autodrive
{
    static
    {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("opencv_java3");
        System.loadLibrary("autodrive");
    }

    public static native void reset();
    public static native void drive();
    public static native void setParkingMode();
    
    public static native void setCarLength(int carLength);
    public static native int getCarLength();
    
    public static native void resetParking();

    public static native void setPerspective(long mataddr);
    public static native void deletePerspective();
    public static native void getPerspective(long mataddr);  //overwrite Mat at address mataddr
    
/*----- DEBUGDATA -----*/
    public static native int gapLength();
    public static native int getManeuver();
    public static native int getManeuverState();

    public static native boolean isInitialGap();
    public static native boolean isGapDepthOk();
    
    public static native int angleTurned();
    
    public static String maneuver()
    {
        switch(getManeuver())
        {
            case 0:
                return "NO_MANEUVER";
            case 1:
                return "PARALLEL_STANDARD";
            case 2:
                return "PARALLEL_WIDE";
            case 3:
                return "PERPENDICULAR_STANDARD";
           default:
                return "ERR";
        }
    }
    
    public static String maneuverstate()
    {
        switch(getManeuverState())
        {
            case 0:
                return "NOT_MOVING";
            case 1:
                return "FORWARD";
            case 2:
                return "BACKWARD";
            case 3:
                return "FORWARD_RIGHT";
            case 4:
                return "BACKWARD_RIGHT";
            case 5:
                return "FORWARD_LEFT";
            case 6:
                return "BACKWARD_LEFT";
            case 7:
                return "DONE";
            default:
                return "ERR";
        }
    }

/*----- SENSORDATA -----*/
    // getters - for debuging purposes   
    public static native int usFront();
    public static native int usFrontRight();
    public static native int usRear();
    public static native int irFrontRight();
    public static native int irRearRight();
    public static native int irRear();
    public static native int gyroHeading();
    public static native int razorHeading();

    public static native void setImage(long matAddrRgba);
    public static native void setUltrasound(int sensor, int value);
    public static native void setInfrared(int sensor, int value);
    public static native void setEncoderPulses(long value);
    public static native void setGyroHeading(int value);
    public static native void setRazorHeading(int value);
    public static native void lineLeftFound();
    public static native void lineRightFound();

/*----- RESULTING AUTODRIVE DATA -----*/
    public static native boolean speedChanged();
    public static native boolean angleChanged();
    private static native double getTargetSpeed();
    private static native double getTargetAngle();

    public static int getConvertedSpeed(){
        double targetSpeed = getTargetSpeed();  // -1.0<=targetSpeed<=1.0
        if(targetSpeed <= 0.0)
            return (int)(targetSpeed * carConfiguration.maxSpeed * 2.0);
        else
            return (int)(targetSpeed * carConfiguration.maxSpeed);
    }

    public static int getConvertedAngle(){
        double rads = getTargetAngle();  //angle in radians from Autodrive image processing
        int degs = (int)(rads * 180 / Math.PI); //convert angle to degrees
        int converted_degs = degs - 90;  //forward direction is 90 degrees.  So calculate offset angle from forward.  e.g. degs=30 -> converted_degs=-60 (turn right); or degs=100 -> converted_degs=10 (turn left)
        //scale the angle with maxAngle to make the car always turn sharper or slower
        converted_degs  =  converted_degs * carConfiguration.scaleSteering / 100;  //e.g. -60 with scaleAngle=50 becomes -30 degrees.

        return (int)(converted_degs);
    }

/*---- SETTINGS -----*/
    public static native void setSettingLightNormalization(boolean on);
    public static native void setSettingUseLeftLine(boolean on);

    public static native void setLeftLane(boolean on);

    // N Frames to take the mean value from, value should be between 0 - 8
    public static native void setSettingSmoothening(int value);

    // Maximum vertical distance to the first pixel from carY, value should be between 15-60
    public static native void setSettingFirstFragmentMaxDist(int value);

    // How many pixels to iterate to the left, for each pixel, value should be between 1-15
    public static native void setSettingLeftIterationLength(int value);

    // How many pixels to iterate to the right, for each pixel, value should be between 1-15
    public static native void setSettingRightIterationLength(int value);

    // Every pixel in a line can not have an angle from the previous pixel that deviates more than this,
    // value should be between 0.4-1.4
    public static native void setSettingMaxAngleDiff(float value);

    public static native void setCannyThresh(int value);

    public static native void setCarScaleDriftFix(float value);

    //PID controller parameters: Kp, Kd and Ki
    public static native void setPidKp(float value);

    public static native void setPidKd(float value);

    public static native void setPidKi(float value);
}
