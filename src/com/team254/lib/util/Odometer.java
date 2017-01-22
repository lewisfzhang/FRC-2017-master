package com.team254.lib.util;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Drive;

public class Odometer {
    private static Odometer mOdometer;
    
    private double mX;
    private double mY;
    private double mHeading;
    
    private Drive mDrive = Drive.getInstance();
         
    private Odometer() {
        mX = 0;
        mY = 0;
        mHeading = 0;
        mDrive.zeroSensors();
    }
    
    public static Odometer getInstance() {
        if (mOdometer == null)
            mOdometer = new Odometer();
        return mOdometer;
    }
    
    protected class Odometer_Loop implements Loop {
        public void onStart(double timestamp) {
            mDrive.zeroSensors();
            // ZERO THE NAVX SENSOR!!!
        }

        public void onLoop(double timestamp) {
            update();
        }

        public void onStop(double timestamp) {
            
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Odometer_Loop());
    }
    
    public synchronized void update()
    {
        mY = (2 * Math.PI * Constants.kWheelRadius * (mDrive.getLEncoderTicks() + mDrive.getREncoderTicks())) / (4096 * 2.0);
        mX = (2 * Math.PI * Constants.kWheelRadius * (mDrive.getLEncoderTicks() - mDrive.getREncoderTicks())) / (4096 * Constants.kRobotWidth / 2);
        // TO BE IMPLEMENTED W/ KauaiLabs NavX SENSOR: mHeading = 
    }
    
    public Translation2d getTranslation() {
        return new Translation2d(mX, mY);
    }
    
    public double getX() {
        return mX;
    }
    
    public double getY() {
        return mY;
    }
    
    public double getHeading() {
        return mHeading;
    }
}
