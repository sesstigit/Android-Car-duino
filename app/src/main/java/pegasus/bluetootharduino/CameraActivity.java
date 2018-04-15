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

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.GestureDetector;
import android.view.GestureDetector.OnGestureListener;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.IOException;

import static pegasus.bluetootharduino.Autodrive.getPerspective;

//! This class is activated when the user clicks Auto driving mode.
//! Each camera frame is processed for navigation with line:
//! driver.processImage(inputFrame.rgba());
//! i.e. the image has rgba format.
//! Debug information is sent to a view called "R.id.debugConsole". It includes all the car sensor readings!
public class CameraActivity extends Activity implements CvCameraViewListener2, OnGestureListener {

    private CameraBridgeViewBase mOpenCvCameraView;
    private static final String TAG = "CameraActivity";
    static TextView debugConsole;

    GestureDetector detector;
    AutomaticCarDriver driver = new AutomaticCarDriver();

    BluetoothConnection bt = new BluetoothConnection();
    BluetoothPairing blue = new BluetoothPairing();

    @SuppressWarnings("deprecation")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.camera_activity);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.CameraView);
        //mOpenCvCameraView.setMaxFrameSize(240,135);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        detector = new GestureDetector(this);

        debugConsole = (TextView) findViewById(R.id.debugConsole);

        if(blue.btEnabled) {
            try {
                bt.runBT();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    static public void updateDebuggingConsole() {
	    StringBuilder sb = new StringBuilder();

        sb.append("SENSORS:\n");
        sb.append("US F,FR,RL: " + String.valueOf(Autodrive.usFront()) + "," + String.valueOf(Autodrive.usFrontRight()) + "," + String.valueOf(Autodrive.usRear()) + "\n");
        sb.append("IR SF,SR,R: " + String.valueOf(Autodrive.irFrontRight()) + "," + String.valueOf(Autodrive.irRearRight()) + "," + String.valueOf(Autodrive.irRear()) + "\n");
        sb.append("gyro: " + String.valueOf(Autodrive.gyroHeading()) + "\n");
        sb.append("PARKING:\n");
        sb.append("gap length: " + String.valueOf(Autodrive.gapLength()) + "\n");
        sb.append("maneuver mode, state: " + Autodrive.maneuver() + "," + Autodrive.maneuverstate() + "\n");
        sb.append("current angle: " + Autodrive.angleTurned() + "\n");
        sb.append("is initial gap: " + String.valueOf(Autodrive.isInitialGap()) + "\n");
        sb.append("has correct depth: " + String.valueOf(Autodrive.isGapDepthOk()) + "\n");

	debugConsole.setText(sb.toString());

    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };


    @Override
    public void onResume()
    {
        driver = new AutomaticCarDriver();
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_6, this, mLoaderCallback);
    }


    /** activity implements the camera view */

    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {}

    @Override
    public void onCameraViewStopped() {}

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        //Note: CvCameraViewFrame only supports rgba and gray cv::Mat images
        return driver.processImage(inputFrame.rgba());
    }


    /** Uses swipe to change to the main activity*/

    @Override
    public boolean onDown(MotionEvent e) {
        return false;
    }

    @Override
    public void onShowPress(MotionEvent e) {}

    @Override
    public boolean onSingleTapUp(MotionEvent e) {
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
        return false;
    }

    @Override
    public void onLongPress(MotionEvent e) {}

    @Override
    public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX,	float velocityY) {

        if(e1.getX()<e2.getX()) {
            // disconnect safely
            if(bt.socket.isConnected()) {
                bt.sendToManualMode("stop");
                bt.disconnect();
            }
            /** Changes to Main screen */
            Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
            startActivity(changeToMain);
            return true;
        }

        return false;
    }

    public boolean onTouchEvent(MotionEvent ev) {
        return detector.onTouchEvent(ev);
    }

    /** Changes the behaviour of the back button */
    public void onBackPressed() {
        // save the perspective matrix to disk so it can be reused next time
        Mat m = new Mat();
        getPerspective(m.getNativeObjAddr());

        if (!m.empty()) {
            String filename = "perspective.bmp";
            //File file = new File(context.getFilesDir(), filename);
            Imgcodecs.imwrite(filename, m);
        }
        // disconnect safely
        if(bt.socket.isConnected()) {
            bt.sendToManualMode("stop");
            bt.disconnect();
        }
        Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
        startActivity(changeToMain);
    }
}
