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
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.widget.SeekBar;
import android.widget.TextView;
import android.util.Log;

//! This class handles the advanced settings GUI screen.  It changes Autodrive settings to configure the car for different driving conditions.
public class AdvSettingsActivity extends Activity implements SeekBar.OnSeekBarChangeListener, GestureDetector.OnGestureListener {

    SeekBar cannyThresh, carMaxSpeed, carScaleSteering, carScaleDriftFix, smoothening, fragment, leftIteration, rightIteration, angle;
    GestureDetector detector;
    SharedPreferences shared;
    float progressValue;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.advanced_settings);

        shared = PreferenceManager.getDefaultSharedPreferences(this);

        detector = new GestureDetector(this);

        //SEEK BARS
        cannyThresh = (SeekBar)findViewById(R.id.cannyThresh);
        cannyThresh.setMax(300);
        cannyThresh.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress1)).setText("cannyThresh value set to " + shared.getFloat("cannyThresh", progressValue));
        cannyThresh.setProgress((int) (shared.getFloat("cannyThresh", progressValue) ));
        
        carMaxSpeed = (SeekBar)findViewById(R.id.carMaxSpeed);
        carMaxSpeed.setMax(600);
        carMaxSpeed.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress2)).setText("carMaxSpeed value set to " + shared.getFloat("carMaxSpeed", progressValue));
        carMaxSpeed.setProgress((int) (shared.getFloat("carMaxSpeed", progressValue) ));
        
        carScaleSteering = (SeekBar)findViewById(R.id.carScaleSteering);
        carScaleSteering.setMax(400);  //want -200 to 200
        carScaleSteering.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress3)).setText("carScaleSteering value set to " + shared.getFloat("carScaleSteering", progressValue));
        carScaleSteering.setProgress((int) (shared.getFloat("carScaleSteering", progressValue) + 200 ));
        
        carScaleDriftFix = (SeekBar)findViewById(R.id.carScaleDriftFix);
        carScaleDriftFix.setMax(100);  //want 0 to 10 float
        carScaleDriftFix.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress9)).setText("carScaleDriftFix value set to " + shared.getFloat("carScaleDriftFix", progressValue));
        carScaleDriftFix.setProgress((int) (shared.getFloat("carScaleDriftFix", progressValue) * 10 ));
        
        smoothening = (SeekBar)findViewById(R.id.smoothening);
        smoothening.setMax(8); // values 0-8
        smoothening.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress4)).setText("Smoothening value set to " + shared.getFloat("smoothening", progressValue));
        smoothening.setProgress((int) (shared.getFloat("smoothening", progressValue)));
        
        fragment = (SeekBar)findViewById(R.id.fragment);
        fragment.setMax(60); // values 15-60
        fragment.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress5)).setText("Fragment distance value set to " + shared.getFloat("fragment", progressValue));
        fragment.setProgress((int) (shared.getFloat("fragment", progressValue)));
        
        leftIteration = (SeekBar)findViewById(R.id.leftIteration);
        leftIteration.setMax(15); // values 1-15
        leftIteration.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress6)).setText("Left Iteration value set to " + shared.getFloat("leftIteration", progressValue));
        leftIteration.setProgress((int) (shared.getFloat("leftIteration", progressValue)));
        
        rightIteration = (SeekBar)findViewById(R.id.rightIteration);
        rightIteration.setMax(15); // values 1-15
        rightIteration.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress7)).setText("Right Iteration value set to " + shared.getFloat("rightIteration", progressValue));
        rightIteration.setProgress((int) (shared.getFloat("rightIteration", progressValue)));
        
        angle = (SeekBar)findViewById(R.id.angle);
        angle.setMax(14); // values 0.4-1.4
        angle.setOnSeekBarChangeListener(this);
        ((TextView)findViewById(R.id.progress8)).setText("Max angle value set to " + shared.getFloat("angle", progressValue));
        angle.setProgress((int) (shared.getFloat("angle", progressValue) *10));
        
    }

    /* SEEK BAR */
    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

        switch (seekBar.getId()) {
            case R.id.cannyThresh:
                if(fromUser) {
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress1)).setText("cannyThresh value set to " + progressValue);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("cannyThresh", progressValue);
                    sharedEditor.apply();
                    Autodrive.setCannyThresh(progress);
                    Log.i("SetCannyThresh:", "value=" + progress);
                }
                break;
            case R.id.carMaxSpeed:
                if(fromUser) {
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress2)).setText("carMaxSpeed value set to " + progressValue);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("carMaxSpeed", progressValue);
                    sharedEditor.apply();
                    //Autodrive.setCarMaxSpeed(progressValue);
                    carConfiguration.maxSpeed = progress;
                    Log.i("CarConfiguration.maxSpeed:", "value=" + progress);
                }
                break;
            case R.id.carScaleSteering:
                if(fromUser) {
                    progressValue = progress - 200;
                    ((TextView)findViewById(R.id.progress3)).setText("carScaleSteering value set to " + progressValue);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("carScaleSteering", progressValue);
                    sharedEditor.apply();
                    //Autodrive.setCarMaxAngle(progressValue);
                    carConfiguration.scaleSteering = (int) progressValue;
                    Log.i("carConfiguration.scaleSteering:", "value=" + progressValue);
                }
                break;
            case R.id.carScaleDriftFix:
                if(fromUser) {
                    progressValue = (float) (progress / 10.0);
                    ((TextView)findViewById(R.id.progress9)).setText("carScaleDriftFix value set to " + progressValue);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("carScaleDriftFix", progressValue);
                    sharedEditor.apply();
                    Autodrive.setCarScaleDriftFix(progressValue);
                    Log.i("setCarScaleDriftFix:", "value=" + progressValue);
                }
                break;
            case R.id.smoothening:
                if(fromUser) {
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress4)).setText("Smoothening value set to " + (float)progress);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("smoothening", progressValue);
                    sharedEditor.apply();
                    Autodrive.setSettingSmoothening(progress);
                    Log.i("setSettingSmoothening:", "value=" + progress);
                }
                break;
            case R.id.fragment:
                if(fromUser) {
                    if(progress < 15) {
                        progress = 15;
                        fragment.setProgress(progress);
                    }
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress5)).setText("Fragment distance value set to " + (float)progress);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("fragment", progressValue);
                    sharedEditor.apply();
                    Autodrive.setSettingFirstFragmentMaxDist(progress);
                    Log.i("setSettingFirstFragmentMaxDist:", "value=" + progress);
                }
                break;
            case R.id.leftIteration:
                if(fromUser) {
                    if(progress < 1) {
                        progress = 1;
                        leftIteration.setProgress(progress);
                    }
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress6)).setText("Left Iteration value set to " + (float)progress);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("leftIteration", progressValue);
                    sharedEditor.apply();
                    Autodrive.setSettingLeftIterationLength(progress);
                    Log.i("setSettingLeftIterationLength:", "value=" + progress);
                }
                break;
            case R.id.rightIteration:
                if(fromUser) {
                    if(progress < 1) {
                        progress = 1;
                        rightIteration.setProgress(progress);
                    }
                    progressValue = progress;
                    ((TextView)findViewById(R.id.progress7)).setText("Right Iteration value set to " + (float)progress);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("rightIteration", progressValue);
                    sharedEditor.apply();
                    Autodrive.setSettingRightIterationLength(progress);
                    Log.i("setSettingRightIterationLength:", "value=" + progress);
                }
                break;
            case R.id.angle:
                if(fromUser) {
                    progressValue = (float) (progress / 10.0);
                    if(progressValue < 0.4) {
                        progressValue = (float) 0.4;
                        angle.setProgress((int) progressValue);
                    } else if(progressValue > 1.4) {
                        progressValue = (float) 1.4;
                        angle.setProgress((int) progressValue);
                    }
                    ((TextView)findViewById(R.id.progress8)).setText("Max angle value set to " + progressValue);
                    SharedPreferences.Editor sharedEditor = shared.edit();
                    sharedEditor.putFloat("angle", progressValue);
                    sharedEditor.apply();
                    Autodrive.setSettingMaxAngleDiff(progressValue);
                    Log.i("setSettingMaxAngleDiff:", "value=" + progress);
                }
                break;
        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {}

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {}



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
        Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
        startActivity(changeToMain);
    }
}
