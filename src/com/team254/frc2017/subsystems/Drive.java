package com.team254.frc2017.subsystems;

import java.util.Collection;
import java.util.LinkedList;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Kinematics;
import com.team254.lib.util.Odometer;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.CollisionDetectionListener;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and slave motor
    private static Accelerometer mAccel;
    private static AHRS mNavXBoard;
    
    private static AdaptivePurePursuitController mPathController;
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
    	//mPathController = new AdaptivePurePursuitController(fixed_lookahead, max_accel, nominal_dt, path, reversed, path_completion_tolerance)
    	
    	mNavXBoard = new AHRS(SPI.Port.kMXP);
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
            mNavXBoard.reset();
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
    
    public Double getAccelerometerMagnitude() {
    	return Math.hypot(mAccel.getX(), mAccel.getY());
    }
    
    public float getAngle() {
        return mNavXBoard.getFusedHeading();
    }
    
    private void checkForCollision() {
    	if (true/*TODO: Check if base locked*/ && getAccelerometerMagnitude() > Constants.kCollisionThreshold) {
    		for(CollisionDetectionListener listen : mCollisionListeners) {
    			listen.didCollide();
    		}
    	}
    }
    
    private void updatePathFollower() {
        RigidTransform2d robot_pose = new RigidTransform2d();//Odometer.getInstance().getTranslation();
        RigidTransform2d.Delta command = mPathController.update(robot_pose);
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        updateVelocitySetpoint(setpoint.left, setpoint.right);
    }
    
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        mLeftMaster.set(inchesPerSecondToRpm(left_inches_per_sec));
        mRightMaster.set(inchesPerSecondToRpm(right_inches_per_sec));
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    private static double inchesToRotations(double inches) {
        return inches / (Constants.kWheelRadius * Math.PI);
    }
}
