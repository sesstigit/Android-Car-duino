package pegasus.bluetootharduino;

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

    AutomaticCarDriver(){
        Autodrive.reset();
    }

    public Mat processImage(Mat image) {
        Mat resized = new Mat();
        Size prevSize = image.size();
        Size size = new Size(240,135);
        Imgproc.resize(image, resized, size,0,0,Imgproc.INTER_NEAREST);
        Autodrive.setImage(resized.getNativeObjAddr());
        Autodrive.drive();
        BluetoothConnection.send();
        if(Settings.DisplayDebugInformation)
            Imgproc.resize(resized, image, prevSize,0,0,Imgproc.INTER_NEAREST);

        return image;
    }
}
