package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import com.team254.frc2017.Constants;
import com.team254.frc2017.Kinematics;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.paths.TestArcPath;
import com.team254.lib.util.*;
import com.team254.lib.util.motion.MotionProfileConstraints;
import com.team254.lib.util.motion.MotionProfileGoal;
import com.team254.lib.util.motion.MotionState;
import com.team254.lib.util.motion.ProfileFollower;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

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
        PATH_FOLLOWING,
        AIM_TO_GOAL,
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private final AHRS mNavXBoard;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private AdaptivePurePursuitController mPathController;

    // These gains get reset below!!
    private ProfileFollower mProfileFollower = new ProfileFollower(0,0,0,0,0);

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            setOpenLoop(DriveSignal.NEUTRAL);
            setBrakeMode(false);
            setVelocitySetpoint(0, 0);
            mNavXBoard.reset();
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
                            updatePathFollower(timestamp);
                        } else {
                            setVelocitySetpoint(0.0, 0.0);
                        }
                        return;
                    case AIM_TO_GOAL:
                        updateTurnToHeadingSimplePid(timestamp);
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
            mLeftMaster.configNominalOutputVoltage(0,0);
            mRightMaster.configNominalOutputVoltage(0,0);
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
        SmartDashboard.putNumber("left speed (ips)", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right speed (ips)", getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getPosition());
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getPosition());
        SmartDashboard.putNumber("gyro vel", getGyroVelocity());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
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
                mDriveControlState == DriveControlState.PATH_FOLLOWING ||
                mDriveControlState == DriveControlState.AIM_TO_GOAL) {
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

    public synchronized double getGyroVelocity() {
        return -mNavXBoard.getRawGyroZ();
    }

    private void updateTurnToHeading(double timestamp) {
        final Map.Entry<InterpolatingDouble, RigidTransform2d> latest_field_to_robot = mRobotState
                .getLatestFieldToVehicle();
        final RigidTransform2d field_to_robot = latest_field_to_robot.getValue();
        final double t_observation = latest_field_to_robot.getKey().value;
        final ShooterAimingParameters aim = mRobotState.getAimingParameters(t_observation);

        // To deal with angle rollover - but still get the desired behavior whenever the goal or actual state changes -
        // we need to "unwrap" our angles around the previous goal. This requires using a custom version of a transform
        // operator that does NOT bound the resulting angle (e.g. new_angle = prev_angle +
        // shortest_distance_from_prev_to_new).
        final MotionProfileGoal prev_goal = mProfileFollower.getGoal();
        final Rotation2d prev_goal_to_field = (prev_goal == null ? Rotation2d.identity()
                : Rotation2d.fromDegrees(prev_goal.pos()).inverse());
        final Rotation2d field_to_new_goal = aim.getFieldToGoal();

        // Update the goal.
        mProfileFollower.setGoal(
                new MotionProfileGoal(prev_goal.pos() + prev_goal_to_field.rotateBy(field_to_new_goal).getDegrees()));

        // Update the prior setpoint (so we don't see a sudden jump in error).
        final MotionState prev_setpoint = mProfileFollower.getSetpoint();
        if (prev_setpoint != MotionState.kInvalidState) {
            mProfileFollower.resetSetpoint(new MotionState(prev_setpoint.t(),
                    prev_goal.pos()
                            + prev_goal_to_field.rotateBy(Rotation2d.fromDegrees(prev_setpoint.pos())).getDegrees(),
                    prev_setpoint.vel(), prev_setpoint.acc()));
        }

        // Update the actual state (which should also be in the previous goal frame).
        final MotionState motion_state = new MotionState(t_observation,
                prev_goal.pos() + prev_goal_to_field.rotateBy(field_to_robot.getRotation()).getDegrees(),
                getGyroVelocity(), 0.0);
        final double angular_velocity_command = mProfileFollower.update(motion_state, timestamp + Constants.kLooperDt);

        Kinematics.DriveVelocity wheel_vel = Kinematics.inverseKinematics(
                new RigidTransform2d.Delta(0, 0, Rotation2d.fromDegrees(angular_velocity_command).getRadians()));

        updateVelocitySetpoint(wheel_vel.left, wheel_vel.right);
    }

    private void updateTurnToHeadingSimplePid(double timestamp) {
        Map.Entry<InterpolatingDouble, RigidTransform2d> latest_field_to_robot = mRobotState.getLatestFieldToVehicle();
        RigidTransform2d field_to_robot = latest_field_to_robot.getValue();
        ShooterAimingParameters aim = mRobotState.getAimingParameters(timestamp);

        double error = aim.getFieldToGoal().getDegrees() - field_to_robot.getRotation().getDegrees();
        SmartDashboard.putNumber("drive_turn_goal", aim.getFieldToGoal().getDegrees());
        SmartDashboard.putNumber("drive_turn_error", error);
        SmartDashboard.putNumber("drive_turn_cur", field_to_robot.getRotation().getDegrees() );
        double velocitySignal = error * Constants.kDriveTurnSimpleKp;
        SmartDashboard.putNumber("drive_turn_vel", velocitySignal );
        SmartDashboard.putNumber("goal_dist", aim.getRange());

        Kinematics.DriveVelocity wheelVel = Kinematics.inverseKinematics(
                new RigidTransform2d.Delta(0,0, Rotation2d.fromDegrees(velocitySignal).getRadians()));

        SmartDashboard.putNumber("drive_turn_wheel_vel", wheelVel.left );

        mIsOnTarget = error < Constants.kOnTargetErrorThreshold;

        updateVelocitySetpoint(-velocitySignal, velocitySignal);
    }

    private void updatePathFollower(double timestamp) {
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

    public void resetProfileGains() {
        mProfileFollower.setGains(
                Constants.kDriveTurnKp,
                Constants.kDriveTurnKi,
                Constants.kDriveTurnKv,
                Constants.kDriveTurnKffv,
                Constants.kDriveTurnKffa
        );
    }

    public boolean isOnTarget() {
        return mIsOnTarget && mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    public void setWantAimToGoal() {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            configureTalonsForSpeedControl();
            resetProfileGains();
            mProfileFollower.resetProfile();
            mProfileFollower.resetSetpoint();
            mProfileFollower.setConstraints(new MotionProfileConstraints(Constants.kDriveTurnMaxVel, Constants.kDriveTurnMaxAcc));
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
        }
    }
    
    
    private Path mCurrentPath = null;
    public void setWantDrivePath(Path path) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            mPathController = new AdaptivePurePursuitController(path);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        }    
    }
}
