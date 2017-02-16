package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.RobotState;
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
        VELOCITY_SETPOINT,
        PATH_FOLLOWING
    }

    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private final AHRS mNavXBoard;
    private AdaptivePurePursuitController mPathController;
    private DriveControlState mDriveControlState;

    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            setOpenLoop(DriveSignal.NEUTRAL);
            setBrakeMode(false);
            setVelocitySetpoint(0, 0);
            mNavXBoard.reset();
            //mDriveControlState = DriveControlState.PATH_FOLLOWING; //testing
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        return;
                    case VELOCITY_SETPOINT:
                        return;
                    case PATH_FOLLOWING:
                        if(mPathController != null && !mPathController.isFinished()) {
                            updatePathFollower();
                        } else {
                            setVelocitySetpoint(0.0, 0.0);
                        }
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
        mLeftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 10);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        mLeftMaster.reverseOutput(false);
        CANTalon.FeedbackDeviceStatus leftSensorPresent =
                mLeftMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }

        mLeftSlave = new CANTalon(Constants.kLeftDriveSlaveId);
        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave.set(Constants.kLeftDriveMasterId);
        mLeftSlave.reverseOutput(false);

        mRightMaster = new CANTalon(Constants.kRightDriveMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 10);
        mRightMaster.reverseSensor(false);
        mRightMaster.reverseOutput(true);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        CANTalon.FeedbackDeviceStatus rightSensorPresent =
                mRightMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }

        mRightSlave = new CANTalon(Constants.kRightDriverSlaveId);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mRightSlave.set(Constants.kRightDriveMasterId);
        mRightSlave.reverseOutput(false);

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
            setBrakeMode(false);
        }
        mRightMaster.set(signal.getRight());
        mLeftMaster.set(signal.getLeft());
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
        SmartDashboard.putNumber("left speed (ips)", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right speed (ips)", getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
    }
    
    public synchronized void resetEncoders() {
        mLeftMaster.setEncPosition(0);
        mLeftMaster.setPosition(0);
        mRightMaster.setPosition(0);
        mRightMaster.setEncPosition(0);
        mLeftSlave.setPosition(0);
        mRightSlave.setPosition(0);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.reset();
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
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setProfile(kVelocityControlSlot);
            mRightMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
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
        if (mDriveControlState == DriveControlState.VELOCITY_SETPOINT ||
                mDriveControlState == DriveControlState.PATH_FOLLOWING) {
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
        return Rotation2d.fromDegrees(-mNavXBoard.getAngle());
    }

    private void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
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
            updateVelocitySetpoint(0,0);
        }
    }

    public void setStartPathTest() {
        mPathController = new AdaptivePurePursuitController(Constants.kAutoFilePath);
        mDriveControlState = DriveControlState.PATH_FOLLOWING;
    }
}
