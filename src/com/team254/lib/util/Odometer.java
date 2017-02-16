package com.team254.lib.util;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class Odometer {
    private static Odometer mOdometer;
    
    private double mX;
    private double mY;
    private Rotation2d mHeading;
    private double timestamp;
    
    private Drive mDrive;
         
    private Odometer() {
        mX = 0;
        mY = 0;
        mHeading = new Rotation2d();
        timestamp = Timer.getFPGATimestamp();
        //mDrive.zeroSensors();
    }
    
    public static Odometer getInstance() {
        if (mOdometer == null)
            mOdometer = new Odometer();
        return mOdometer;
    }
    
    public static Odometer getInstance(Drive d) {
        if (mOdometer == null)
            mOdometer = new Odometer();
        return mOdometer;
    }

    
    protected class Odometer_Loop implements Loop {
        public void onStart(double timestamp) {
            //mDrive.zeroSensors();
            mDrive = Drive.getInstance();
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
        double now = Timer.getFPGATimestamp();
        double delta_t = now - timestamp;
        timestamp = now;
        double rSpeed = mDrive.getRightVelocityInchesPerSec();
        double lSpeed = mDrive.getLeftVelocityInchesPerSec();
        double dist = delta_t * (lSpeed + rSpeed) / 2;
        
        mHeading = mDrive.getGyroAngle();
        mX += mHeading.cos() * dist;
        mY += mHeading.sin() * dist;
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
