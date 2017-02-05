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
import com.team254.lib.util.ADXRS453_Gyro;
import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Kinematics;
import com.team254.lib.util.Odometer;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.CollisionDetectionListener;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Drive extends Subsystem {
    private static CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and slave motor
    private static Accelerometer mAccel;
    private static AHRS mNavXBoard;
    //private static Encoder mRightEncoder, mLeftEncoder;    
    
    private static AdaptivePurePursuitController mPathController;
    private static Drive mInstance;
    
    private double mLeftRpm, mRightRpm;
    private SynchronousPIDF mLeftController, mRightController;

    
    private Collection<CollisionDetectionListener> mCollisionListeners = new LinkedList<CollisionDetectionListener>();
    
    private Drive() {
        // What kind of encoder will we be using in the drivetrain? Regular quadrature encoders?
        
        mLeftMaster = new CANTalon(11, 5);
        mLeftMaster.changeControlMode(TalonControlMode.Speed);
        mLeftMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);

        mLeftSlave = new CANTalon(10);
        mLeftSlave.changeControlMode(TalonControlMode.Follower);
        mLeftSlave.set(11);
        
        mRightMaster = new CANTalon(4, 5);
        mRightMaster.changeControlMode(TalonControlMode.Speed);
        mRightMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);

        mRightSlave = new CANTalon(3);
        mRightSlave.changeControlMode(TalonControlMode.Follower);
        mRightSlave.set(4);
        
        mAccel = new BuiltInAccelerometer(); 
    	mAccel = new BuiltInAccelerometer(Accelerometer.Range.k4G); 
    	
    	mNavXBoard = new AHRS(SPI.Port.kMXP);
        
    	mPathController = new AdaptivePurePursuitController("~/path.txt");
    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    protected class Drive_Loop implements Loop {
        private double mPrevTimestamp = 0;
        
        public void onStart(double timestamp) {
            mPathController.reset();
            mLeftMaster.set(0.0);
            mRightMaster.set(0.0);
            zeroSensors();
        }

        public void onLoop(double timestamp) {
            checkForCollision();
            synchronized (Drive.this) {
                mLeftMaster.set(RpmToInchesPerSecond(getLSpeed()));
                mRightMaster.set(RpmToInchesPerSecond(getRSpeed()));
            }
            if(!mPathController.isFinished()) {
                updatePathFollower();
            }
            
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
        mNavXBoard.reset();
        Odometer.getInstance().resetPosition();
    }

    public void stop() {
        mLeftMaster.set(0);
        mRightMaster.set(0);
        updateVelocitySetpoint(0,0);
        //zeroSensors();
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive Motor Inches sec", RpmToInchesPerSecond(mLeftRpm));
        SmartDashboard.putNumber("Right Drive Motor Inches sec", RpmToInchesPerSecond(mRightRpm));
        SmartDashboard.putNumber("Left Encoder Position", getLEncoderTicks());
        SmartDashboard.putNumber("Right Encoder Position", getREncoderTicks());
        SmartDashboard.putNumber("Field X Position", Odometer.getInstance().getX());
        SmartDashboard.putNumber("Field Y Position", Odometer.getInstance().getY());
        SmartDashboard.putNumber("Heading", Odometer.getInstance().getHeading().getDegrees());
    }

    // Returns 4096 ticks per rotation
    public int getLEncoderTicks() {
        return mLeftMaster.getEncPosition();
    }
    
    public int getREncoderTicks() {
        return mRightMaster.getEncPosition();
    }
    
    //Returns the number speed in RPM of the left side
    public double getLSpeed() {
        return mLeftMaster.getSpeed();
    }
    
    public double getRSpeed() {
        return mRightMaster.getSpeed();
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
    
    private void updatePathFollower() {
        RigidTransform2d robot_pose = Odometer.getInstance().getPose();
        RigidTransform2d.Delta command = mPathController.update(robot_pose);
        if(!mPathController.isFinished()) {
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
        } else {
            stop();
        }
    }
    
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        mRightController.setSetpoint(right_inches_per_sec);
        mLeftController.setSetpoint(left_inches_per_sec);
        SmartDashboard.putNumber("Right Wheel Speed", right_inches_per_sec);
        SmartDashboard.putNumber("Left Wheel Speed", left_inches_per_sec );
    }
 
    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    public static double RpmToInchesPerSecond(double rpm) {
        return rpm / 60 * Constants.kWheelRadius * Math.PI * 2;
    }
    
    public static double inchesToRotations(double inches) {
        return inches / (Constants.kWheelRadius * Math.PI);
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-mNavXBoard.getFusedHeading());
    }
}
