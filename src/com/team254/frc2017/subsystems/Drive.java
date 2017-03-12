package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.*;
import com.team254.lib.util.control.Lookahead;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.control.PathFollower;
import com.team254.lib.util.drivers.CANTalonFactory;
import com.team254.lib.util.drivers.NavX;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Twist2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, VELOCITY_SETPOINT, PATH_FOLLOWING, AIM_TO_GOAL, TURN_TO_HEADING
    }

    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.AIM_TO_GOAL || state == DriveControlState.TURN_TO_HEADING) {
            return true;
        }
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private final NavX mNavXBoard;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                mNavXBoard.reset();
            }
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
                    if (mPathFollower != null) {
                        updatePathFollower(timestamp);
                    }
                    return;
                case AIM_TO_GOAL:
                    if (!Superstructure.getInstance().isShooting()) {
                        updateGoalHeading(timestamp);
                    }
                    // fallthrough intended
                case TURN_TO_HEADING:
                    updateTurnToHeading(timestamp);
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
        mLeftMaster = CANTalonFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        mLeftMaster.reverseOutput(false);
        CANTalon.FeedbackDeviceStatus leftSensorPresent = mLeftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }

        mLeftSlave = CANTalonFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveId,
                Constants.kLeftDriveMasterId);
        mLeftSlave.reverseOutput(false);
        mLeftMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);

        mRightMaster = CANTalonFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.reverseSensor(false);
        mRightMaster.reverseOutput(true);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        CANTalon.FeedbackDeviceStatus rightSensorPresent = mRightMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }

        mRightSlave = CANTalonFactory.createPermanentSlaveTalon(Constants.kRightDriverSlaveId,
                Constants.kRightDriveMasterId);
        mRightSlave.reverseOutput(false);
        mRightMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        reloadGains();

        mIsHighGear = false;
        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new NavX(SPI.Port.kMXP);

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
            mLeftMaster.configNominalOutputVoltage(0.0, 0.0);
            mRightMaster.configNominalOutputVoltage(0.0, 0.0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
        mRightMaster.set(-signal.getRight());
        mLeftMaster.set(signal.getLeft());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
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
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getOutputVoltage());
        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        if (usesTalonVelocityControl(mDriveControlState)) {
            SmartDashboard.putNumber("left speed error (ips)",
                    rpmToInchesPerSecond(mLeftMaster.getSetpoint()) - left_speed);
            SmartDashboard.putNumber("right speed error (ips)",
                    rpmToInchesPerSecond(mRightMaster.getSetpoint()) - right_speed);
        } else {
            SmartDashboard.putNumber("left speed error (ips)", 0.0);
            SmartDashboard.putNumber("right speed error (ips)", 0.0);
        }
        synchronized (this) {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            } else {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
        SmartDashboard.putBoolean("drive on target", isOnTarget());
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
        mNavXBoard.zeroYaw();
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kHighGearVelocityControlSlot);
            mLeftMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kHighGearVelocityControlSlot);
            mRightMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            setBrakeMode(true);
        }
    }

    private void configureTalonsForPositionControl() {
        if (!usesTalonPositionControl(mDriveControlState)) {
            // We entered a position control state.
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kLowGearPositionControlSlot);
            mLeftMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kLowGearPositionControlSlot);
            mRightMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            setBrakeMode(true);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mLeftMaster.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            mRightMaster.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            mLeftMaster.set(inchesToRotations(left_position_inches));
            mRightMaster.set(inchesToRotations(right_position_inches));
        } else {
            System.out.println("Hit a bad position control state");
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
        return mNavXBoard.getYaw();
    }

    public synchronized NavX getNavXBoard() {
        return mNavXBoard;
    }

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }

    public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    }

    private void updateGoalHeading(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters(timestamp, true);
        if (aim.isPresent()) {
            mTargetHeading = aim.get().getRobotToGoal();
        }
    }

    private void updateTurnToHeading(double timestamp) {
        if (Superstructure.getInstance().isShooting()) {
            // Do not update heading while shooting - just base lock. By not updating the setpoint, we will fight to
            // keep position.
            return;
        }
        final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                wheel_delta.right + getRightDistanceInches());
    }

    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getMeasuredVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget() {
        return mIsOnTarget;
    }
    
    public synchronized boolean isAutoAiming() {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    public synchronized void setWantAimToGoal() {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = getGyroAngle();
        }
        setHighGear(false);
    }

    public synchronized void setWantTurnToHeading(Rotation2d heading) {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
        }
        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
            mTargetHeading = heading;
            mIsOnTarget = false;
        }
        setHighGear(false);
    }

    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }
    
    public synchronized void forceDoneWithPath() {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mLeftMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        mLeftMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        mRightMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mRightMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        mRightMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);

        mLeftMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
    }
    
    public synchronized double getAccelX() {
        return mNavXBoard.getRawAccelX();
    }
}
