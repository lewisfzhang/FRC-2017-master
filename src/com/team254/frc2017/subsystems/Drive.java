package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kVelocityControlSlot = 0;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP,
        VELOCITY_SETPOINT
    }

    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private final Odometer mOdometer;
    private final AHRS mNavXBoard;
    private final AdaptivePurePursuitController mPathController;
    private DriveControlState mDriveControlState;

    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            setOpenLoop(DriveSignal.NEUTRAL);
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        return;
                    case VELOCITY_SETPOINT:
                        //Talons handle this
                        return;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new CANTalon(Constants.kLeftDriveMasterId);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 1);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        if (mLeftMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
                != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder.", false);
        }

        mLeftSlave = new CANTalon(Constants.kLeftDriveSlaveId);
        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave.set(Constants.kLeftDriveMasterId);

        mRightMaster = new CANTalon(Constants.kRightDriveMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 1);
        mRightMaster.setInverted(true);
        mRightMaster.reverseOutput(true);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (mRightMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative)
                != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder.", false);
        }

        mRightSlave = new CANTalon(Constants.kRightDriverSlaveId);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mRightSlave.setInverted(true);
        mRightSlave.set(Constants.kRightDriveMasterId);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        mLeftMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);

        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);
        
        //Path Following stuff
        mNavXBoard = new AHRS(SPI.Port.kMXP);
        mPathController = new AdaptivePurePursuitController(Constants.kAutoFilePath);
        mOdometer = Odometer.getInstance(mInstance);
        
        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        mRightMaster.set(signal.getRight());
        mLeftMaster.set(signal.getLeft());
        setBrakeMode(false);
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
        mShifter.set(!highGear);
    }

    public boolean isBreakMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            mRightMaster.enableBrakeMode(on);
            mRightSlave.enableBrakeMode(on);
            mLeftMaster.enableBrakeMode(on);
            mLeftSlave.enableBrakeMode(on);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left speed (rpm)", mLeftMaster.getSpeed());
        SmartDashboard.putNumber("right speed (rpm)", mRightMaster.getSpeed());
    }

    @Override
    public void zeroSensors() {

    }

    /**
     * Start up velocity mode
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    private void configureTalonsForSpeedControl() {
        if (mDriveControlState != DriveControlState.VELOCITY_SETPOINT) {
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mLeftMaster.setProfile(kVelocityControlSlot);
            mLeftMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setProfile(kVelocityControlSlot);
            mRightMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            setHighGear(true);
            setBrakeMode(true);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
            mLeftMaster.set(inchesPerSecondToRpm(left_inches_per_sec));
            mRightMaster.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    //call this function to make the robot path follow
//    private void updatePathFollower() {
//        RigidTransform2d robot_pose = mOdometer.getPose();
//        RigidTransform2d.Delta command = mPathController.update(robot_pose);
//        if(!mPathController.isFinished()) {
//            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
//    
//            // Scale the command to respect the max velocity limits
//            double max_vel = 0.0;
//            max_vel = Math.max(max_vel, Math.abs(setpoint.left));
//            max_vel = Math.max(max_vel, Math.abs(setpoint.right));
//            if (max_vel > Constants.kPathFollowingMaxVel) {
//                double scaling = Constants.kPathFollowingMaxVel / max_vel;
//                setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
//            }
//            updateVelocitySetpoint(setpoint.left, setpoint.right);
//        } else {
//            stop();
//        }
//    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mLeftMaster.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(mRightMaster.getSpeed());
    }

    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(mNavXBoard.getAngle());
    }

    public double getLSpeed() {
        return rpmToInchesPerSecond(mLeftMaster.getSpeed());
    }
    
    public double getRSpeed() {
        return rpmToInchesPerSecond(mRightMaster.getSpeed());
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-mNavXBoard.getFusedHeading());
    }
}
