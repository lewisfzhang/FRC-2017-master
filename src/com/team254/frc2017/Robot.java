package com.team254.frc2017;

import com.team254.frc2017.Constants.RobotName;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.loops.RobotStateEstimator;
import com.team254.frc2017.loops.VisionProcessor;
import com.team254.frc2017.paths.TestArcPath;
import com.team254.frc2017.subsystems.*;
import com.team254.frc2017.vision.VisionServer;
import com.team254.frc2017.web.WebServer;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.RigidTransform2d;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Subsystems
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    // All Subsystems
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    Drive.getInstance(),
                    Feeder.getInstance(),
                    Hopper.getInstance(),
                    Intake.getInstance(),
                    Shooter.getInstance(),
                    Superstructure.getInstance()));

    // Other parts of the robot
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private WebServer mHTTPServer = new WebServer();

    private VisionServer mVisionServer = VisionServer.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }
    
    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            // initialize robot constants
            RobotName name = Constants.getRobotName();
            SmartDashboard.putString("MAC Address", Constants.getMACAddress());
            ConstantsModifier.initConstants(name);

            mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different autonomous modes using
     * the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the switch structure below with additional
     * strings. If using the SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        mEnabledLooper.start();
        zeroAllSensors();
        //mDrive.setVelocitySetpoint(-15,15);
        //mDrive.setWantAimToGoal();
        mRobotState.reset(Timer.getFPGATimestamp(), TestArcPath.getStartPose());
        mDrive.setWantDrivePath(TestArcPath.buildPath(), TestArcPath.isReversed());
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            zeroAllSensors();
            mSuperstructure.reloadConstants();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        try {
            // Drive base
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            mDrive.setHighGear(!mControlBoard.getLowGear());

            if (mControlBoard.getAimButton()) {
                mDrive.setWantAimToGoal();
            } else {
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn()));
            }

            // Super structure.  For testing check if null.
            // TODO: Remove for comp.
            if (mSuperstructure != null) {
                if (mControlBoard.getIntakeButton()) {
                    mSuperstructure.setWantIntakeOn();
                } else {
                    mSuperstructure.setWantIntakeStopped();
                }

                if (mControlBoard.getFeedButton()) {
                    mSuperstructure.setmWantedState(Superstructure.WantedState.MANUAL_FEED);
                } else {
                    mSuperstructure.setmWantedState(Superstructure.WantedState.IDLE);
                }

                if (mControlBoard.getSpinShooterButton()) {
                    mSuperstructure.setShooterOpenLoop(8.0);
                } else if (mControlBoard.getShootButton()) {
                    mSuperstructure.setClosedLoopRpm(3000.0);
                } else {
                    mSuperstructure.setShooterOpenLoop(0);
                }
            }

            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            mEnabledLooper.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        allPeriodic();
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {

    }

    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
    }
}
