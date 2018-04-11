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

import java.io.IOException;

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
        debugConsole.setText("");
        debugConsole.append("SENSORS:\n");
        debugConsole.append("ultrasonicFront: " + String.valueOf(Autodrive.usFront()) + "\n");
        debugConsole.append("ultrasonicFrontRight: " + String.valueOf(Autodrive.usFrontRight()) + "\n");
        debugConsole.append("ultrasonicRearLeft: " + String.valueOf(Autodrive.usRear()) + "\n");
        debugConsole.append("infraredSideFront: " + String.valueOf(Autodrive.irFrontRight()) + "\n");
        debugConsole.append("infraredSideRear: " + String.valueOf(Autodrive.irRearRight()) + "\n");
        debugConsole.append("infraredRear: " + String.valueOf(Autodrive.irRear()) + "\n");
        debugConsole.append("gyroscope: " + String.valueOf(Autodrive.gyroHeading()) + "\n");
        //debugConsole.append("razorboard: " + String.valueOf(Autodrive.razorHeading()) + "\n");
        debugConsole.append("\n");
        debugConsole.append("PARKING:\n");
        debugConsole.append("gap length: " + String.valueOf(Autodrive.gapLength()) + "\n");
        debugConsole.append("current maneuver: " + Autodrive.maneuver() + "\n");
        debugConsole.append("maneuver state: " + Autodrive.maneuverstate() + "\n");
        debugConsole.append("current angle: " + Autodrive.angleTurned() + "\n");
        debugConsole.append("is initial gap: " + String.valueOf(Autodrive.isInitialGap()) + "\n");
        debugConsole.append("has correct depth: " + String.valueOf(Autodrive.isGapDepthOk()) + "\n");
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

        // disconnect safely
        if(bt.socket.isConnected()) {
            bt.sendToManualMode("stop");
            bt.disconnect();
        }
        Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
        startActivity(changeToMain);
    }
}
