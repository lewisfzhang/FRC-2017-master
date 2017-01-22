package com.team254.frc2017.subsystems;

import java.util.Collection;
import java.util.LinkedList;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.CollisionDetectionListener;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and slave motor
    private static Accelerometer mAccel;
    
    private static Drive mInstance;
    
    private Collection<CollisionDetectionListener> mCollisionListeners = new LinkedList<CollisionDetectionListener>();

    private Drive() {
        // What kind of encoder will we be using in the drivetrain? Regular quadrature encoders?
        
        mLeftMaster = new CANTalon(2);
        mLeftMaster.changeControlMode(TalonControlMode.PercentVbus);
        mLeftMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
        mLeftSlave = new CANTalon(1);
        mLeftSlave.changeControlMode(TalonControlMode.Follower);
        mLeftSlave.set(2);
        
        mRightMaster = new CANTalon(3);
        mRightMaster.changeControlMode(TalonControlMode.PercentVbus);
        mRightMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
        mRightSlave = new CANTalon(4);
        mRightSlave.changeControlMode(TalonControlMode.Follower);
        mRightSlave.set(3);
        
        mAccel = new BuiltInAccelerometer(); 
    	mAccel = new BuiltInAccelerometer(Accelerometer.Range.k4G); 
    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    protected class Drive_Loop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            checkForCollision();
            outputToSmartDashboard();
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Drive_Loop());
    }
    
    public synchronized void setLRPower(double left, double right) {
        mLeftMaster.set(left);
        mRightMaster.set(right);
    }
    
    public void zeroSensors() {
        mLeftMaster.setPosition(0.0);
        mRightMaster.setPosition(0.0);
    }

    public void stop() {
        mLeftMaster.set(0);
        mRightMaster.set(0);
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive Motor RPM", mLeftMaster.getSpeed());
        SmartDashboard.putNumber("Right Drive Motor RPM", mRightMaster.getSpeed());
        SmartDashboard.putNumber("Left Encoder Position", getLEncoderTicks());
        SmartDashboard.putNumber("Right Encoder Position", getREncoderTicks());
    }

    // Returns 4096 ticks per rotation
    public int getLEncoderTicks() {
        return mLeftMaster.getEncPosition();
    }
    
    public int getREncoderTicks() {
        return mRightMaster.getEncPosition();
    }
    
    //Collision Code
    public void registerCollisionListener(CollisionDetectionListener listen) {
    	mCollisionListeners.add(listen);
    }
    
    private Double getAccelerometerMagnitude() {
    	return Math.hypot(mAccel.getX(), mAccel.getY());
    }
    
    private void checkForCollision() {
    	if (true/*TODO: Check if base locked*/ && getAccelerometerMagnitude() > Constants.kCollisionThreshold) {
    		for(CollisionDetectionListener listen : mCollisionListeners) {
    			listen.didCollide();
    		}
    	}
    }
}
