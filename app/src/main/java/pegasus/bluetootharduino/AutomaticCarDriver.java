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

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

//! This is the main class called when Auto driving mode is clicked in the
//! App GUI. CameraActivity class calls processImage() for each camera frame.
//! This class downsizes the image to a lower resolution for faster processing,
//! and then calls the C++ autodrive library to actually process the image
//! such as finding road lanes. Lastly, any commands to the car to change
//! motor speed or steering are sent via bluetooth.
public class AutomaticCarDriver{

    int frameCount;
    long start;

    AutomaticCarDriver(){

        Autodrive.reset();
        start = System.nanoTime();
        frameCount = 0;
    }

    public Mat processImage(Mat image) {
        long elapsedTime;
        float frameRate;

        frameCount++;
        if ((frameCount % 100) == 0) {
            // log the framerate
            elapsedTime = System.nanoTime() - start;
            frameRate = 100/(float)(elapsedTime/1000000000);
            Log.i("Framerate=", "value=" + frameRate);
            start = System.nanoTime();
        }
        /* Commented code which reduced image size before processing, and
           restored size to display.  Instead give control to the image
           processsing library.
        Mat resized = new Mat();
        Size prevSize = image.size();
        Size size = new Size(240,135);
        Imgproc.resize(image, resized, size,0,0,Imgproc.INTER_NEAREST);
        Autodrive.setImage(resized.getNativeObjAddr());
        */
        Autodrive.setImage(image.getNativeObjAddr());
        Autodrive.drive();
        BluetoothConnection.send();
        //if(Settings.DisplayDebug)
        //    Imgproc.resize(resized, image, prevSize,0,0,Imgproc.INTER_NEAREST);
        return image;
    }
}
