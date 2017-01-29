package com.team254.lib.util;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Drive;

public class Odometer {
    private static Odometer mOdometer;
    
    private double mX;
    private double mY;
    private double mPrevRightEncoderRotations;
    private double mPrevLeftEncoderRotations;
    private Rotation2d mHeading;
    
    private Drive mDrive = Drive.getInstance();
         
    private Odometer() {
        mX = 0;
        mY = 0;
        mPrevRightEncoderRotations = 0;
        mPrevLeftEncoderRotations = 0;
        mHeading = new Rotation2d();
        //mDrive.zeroSensors();
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
        //mY = (2 * Math.PI * Constants.kWheelRadius * (mDrive.getLEncoderTicks() + mDrive.getREncoderTicks())) / (4096 * 2.0);
        //mX = (2 * Math.PI * Constants.kWheelRadius * (mDrive.getLEncoderTicks() - mDrive.getREncoderTicks())) / (4096 * Constants.kRobotWidth / 2);
        double dist = (2 * Math.PI * Constants.kWheelRadius * (mDrive.getLEncoderRotations() - mPrevLeftEncoderRotations + mDrive.getREncoderRotations() - mPrevRightEncoderRotations) / 2.0);
        mPrevRightEncoderRotations = mDrive.getREncoderRotations();
        mPrevLeftEncoderRotations = mDrive.getLEncoderRotations();
        
        mHeading = mDrive.getAngle();
        mX += mHeading.cos() * dist;
        mY -= mHeading.sin() * dist;
    }
    
    public Translation2d getTranslation() {
        return new Translation2d(mX, mY);
    }
    
    public RigidTransform2d getPose() {
        return new RigidTransform2d(new Translation2d(mX, mY), mHeading);
    }
    
    public double getX() {
        return mX;
    }
    
    public double getY() {
        return mY;
    }
    
    public Rotation2d getHeading() {
        return mHeading;
    }

    public void resetPosition() {
        mX = 0;
        mY = 0;
    }
    
    public void setPose(RigidTransform2d pose) {
        mX = pose.getTranslation().getX();
        mY = pose.getTranslation().getY();
        mHeading = pose.getRotation();
    }
}
