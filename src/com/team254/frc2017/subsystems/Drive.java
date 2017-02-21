package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.paths.GearToHopperBlue;
import com.team254.lib.util.*;
import com.team254.lib.util.motion.HeadingProfileFollower;
import com.team254.lib.util.motion.MotionProfileConstraints;
import com.team254.lib.util.motion.MotionProfileGoal;
import com.team254.lib.util.motion.MotionProfileGoal.CompletionBehavior;
import com.team254.lib.util.motion.MotionState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.Optional;

public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, VELOCITY_SETPOINT, PATH_FOLLOWING, AIM_TO_GOAL, TURN_TO_HEADING
    }

    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.OPEN_LOOP) {
            return false;
        }
        return true;
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private final AHRS mNavXBoard;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private HeadingProfileFollower mProfileFollower = new HeadingProfileFollower(0, 0, 0, 0, 0);
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
                    if (mPathFollower != null && !mPathFollower.isFinished()) {
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
        mLeftMaster = new CANTalon(Constants.kLeftDriveMasterId);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 10);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        mLeftMaster.reverseOutput(false);
        CANTalon.FeedbackDeviceStatus leftSensorPresent = mLeftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
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
        CANTalon.FeedbackDeviceStatus rightSensorPresent = mRightMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }

        mRightSlave = new CANTalon(Constants.kRightDriverSlaveId);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mRightSlave.set(Constants.kRightDriveMasterId);
        mRightSlave.reverseOutput(false);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        reloadGains();

        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
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

    public synchronized void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
        mShifter.set(!highGear);
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
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
        SmartDashboard.putNumber("gyro vel", getGyroVelocity());
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
        configureTalonsForSpeedControl(mIsHighGear);
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    private void configureTalonsForSpeedControl(boolean wantsHighGear) {
        final boolean enterVelocityControl = !usesTalonVelocityControl(mDriveControlState);
        final boolean changeVelocityControlGearing = enterVelocityControl || (wantsHighGear != isHighGear());
        if (enterVelocityControl) {
            // We entered a velocity control state.
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            setBrakeMode(true);
        }
        if (changeVelocityControlGearing) {
            // We changed gears while remaining in a velocity control state.
            setHighGear(wantsHighGear);
            final double nominal_abs_output = wantsHighGear ? Constants.kDriveHighGearNominalOutput
                    : Constants.kDriveLowGearNominalOutput;
            mLeftMaster.setProfile(wantsHighGear ? kHighGearVelocityControlSlot : kLowGearVelocityControlSlot);
            mLeftMaster.configNominalOutputVoltage(nominal_abs_output, -nominal_abs_output);
            mRightMaster.setProfile(wantsHighGear ? kHighGearVelocityControlSlot : kLowGearVelocityControlSlot);
            mRightMaster.configNominalOutputVoltage(nominal_abs_output, -nominal_abs_output);

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

    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle.getDegrees());
    }

    public synchronized double getGyroVelocity() {
        return -mNavXBoard.getRate() * 180.0 / Math.PI;
    }

    private void updateGoalHeading(double timestamp) {
        Optional<ShooterAimingParameters> aim = RobotState.getInstance().getAimingParameters(timestamp, true);
        if (aim.isPresent()) {
            mTargetHeading = aim.get().getRobotToGoalInField();
        }
    }

    private void updateTurnToHeading(double timestamp) {
        if (Superstructure.getInstance().isShooting()) {
            updateVelocitySetpoint(0.0, 0.0);
        }
        final Map.Entry<InterpolatingDouble, RigidTransform2d> latest_field_to_robot = mRobotState
                .getLatestFieldToVehicle();
        final RigidTransform2d field_to_robot = latest_field_to_robot.getValue();
        final double t_observation = latest_field_to_robot.getKey().value;
        final double target_heading = mTargetHeading.getDegrees();
        final double kGoalPosTolerance = 0.75;
        final double kGoalVelTolerance = 5.0;
        mProfileFollower.setGoal(
                new MotionProfileGoal(target_heading, 0.0, CompletionBehavior.OVERSHOOT, kGoalPosTolerance, kGoalVelTolerance));
        final MotionState motion_state = new MotionState(t_observation, field_to_robot.getRotation().getDegrees(),
                getGyroVelocity(), 0.0);
        final double angular_velocity_command = mProfileFollower.update(motion_state, timestamp + Constants.kLooperDt);
        mIsOnTarget = mProfileFollower.onTarget();

        Kinematics.DriveVelocity wheel_vel = Kinematics
                .inverseKinematics(new RigidTransform2d.Delta(0, 0, angular_velocity_command * Math.PI / 180.0));

        updateVelocitySetpoint(wheel_vel.left, wheel_vel.right);
    }

    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized void resetProfileGains() {
        mProfileFollower.setGains(Constants.kDriveTurnKp, Constants.kDriveTurnKi, Constants.kDriveTurnKv,
                Constants.kDriveTurnKffv, Constants.kDriveTurnKffa);
    }

    public synchronized boolean isOnTarget() {
        return mIsOnTarget && mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    public synchronized void setWantAimToGoal() {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            mIsOnTarget = false;
            // We aim in low gear.
            configureTalonsForSpeedControl(false);
            resetProfileGains();
            mProfileFollower.resetProfile();
            mProfileFollower.resetSetpoint();
            mProfileFollower.setConstraints(
                    new MotionProfileConstraints(Constants.kDriveTurnMaxVel, Constants.kDriveTurnMaxAcc));
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            mTargetHeading = getGyroAngle();
        }
    }

    public synchronized void setWantTurnToHeading(Rotation2d heading) {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
            mIsOnTarget = false;
            configureTalonsForSpeedControl(false);
            resetProfileGains();
            mProfileFollower.resetProfile();
            mProfileFollower.resetSetpoint();
            mProfileFollower.setConstraints(
                    new MotionProfileConstraints(Constants.kDriveTurnMaxVel, Constants.kDriveTurnMaxAcc));
            mTargetHeading = heading;
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
        }
    }

    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl(true);
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(Constants.kAutoLookAhead, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kLooperDt));
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
            return false;
        }
    }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING && mProfileFollower != null) {
            return mProfileFollower.isFinishedProfile();
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.setPID(Constants.kDriveLowGearVelocityKp, Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone, Constants.kDriveLowGearVelocityRampRate,
                kLowGearVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveLowGearVelocityKp, Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone, Constants.kDriveLowGearVelocityRampRate,
                kLowGearVelocityControlSlot);

        mLeftMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
    }
}
