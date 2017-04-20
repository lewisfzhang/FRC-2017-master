package com.team254.frc2017;

import com.team254.frc2017.Constants.RobotName;
import com.team254.frc2017.auto.AutoModeExecuter;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.loops.RobotStateEstimator;
import com.team254.frc2017.loops.VisionProcessor;
import com.team254.frc2017.subsystems.*;
import com.team254.frc2017.subsystems.MotorGearGrabber.WantedState;
import com.team254.frc2017.vision.VisionServer;
import com.team254.lib.util.*;
import com.team254.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Subsystems
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private MotorGearGrabber mGearGrabber = MotorGearGrabber.getInstance();
    private LED mLED = LED.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;

    // All Subsystems
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(Drive.getInstance(), Superstructure.getInstance(), Shooter.getInstance(),
                    Feeder.getInstance(), Hopper.getInstance(), Intake.getInstance(), LED.getInstance(),
                    MotorGearGrabber.getInstance()));

    // Other parts of the robot
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private VisionServer mVisionServer = VisionServer.getInstance();

    private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);


    private LatchedBoolean mCommitTuning = new LatchedBoolean();
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    private final VideoStreamServiceController mVideoStreamServiceController = new VideoStreamServiceController();

    private boolean mPegDistanceControllerDidEngage = false;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            // SmartDashboardUtil.deleteAllKeys();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            // initialize robot constants
            RobotName name = Constants.getRobotName();
            SmartDashboard.putString("MAC Address", Constants.getMACAddress());
            ConstantsModifier.initConstants(name);

            mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

            AutoModeSelector.initAutoModeSelector();

            // mVideoStreamServiceController.registerWatcher();

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

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
            mSuperstructure.setActuateHopper(false);
            mSuperstructure.setOverrideCompressor(true);

            mAutoModeExecuter = null;

            Intake.getInstance().reset();

            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            mSuperstructure.reloadConstants();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
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
            // Shift to high
            mDrive.setHighGear(true);
            zeroAllSensors();
            mSuperstructure.reloadConstants();
            mSuperstructure.setOverrideCompressor(false);
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

            if (mControlBoard.getAimButton() || mControlBoard.getDriveAimButton()) {

                if (Constants.kIsShooterTuning) {
                    mDrive.setWantAimToGoal();
                } else if(mControlBoard.getDriveAimButton()) {
                    mDrive.setWantDriveTowardsGoal();
                } else {
                    mDrive.setWantAimToGoal();
                }
                
                if((mControlBoard.getDriveAimButton() && !mDrive.isApproaching()) || !mControlBoard.getDriveAimButton()) {
                    if (mControlBoard.getUnjamButton()) {
                        mSuperstructure.setWantedState(Superstructure.WantedState.UNJAM_SHOOT);
                    } else {
                        mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
                    }
                } else {
                    mSuperstructure.setWantedState(Superstructure.WantedState.RANGE_FINDING);
                }
            } else {
                boolean engagePegControl = mGearGrabber.getRawDistanceInches() < 20;
                if (mControlBoard.getWantGearDriveLimit() && (mPegDistanceControllerDidEngage || engagePegControl)) {
                    mDrive.setWantDriveToPeg();
                    mPegDistanceControllerDidEngage = true;
                } else {
                    mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                            !mControlBoard.getLowGear()));
                    mPegDistanceControllerDidEngage = false;

                }
                boolean wantLowGear = mControlBoard.getWantGearDriveLimit() || mControlBoard.getLowGear();
                mDrive.setHighGear(!wantLowGear);


                Intake.getInstance().setCurrentThrottle(mControlBoard.getThrottle());

                boolean wantsExhaust = mControlBoard.getExhaustButton();

                if (Constants.kIsShooterTuning) {
                    mLED.setWantedState(LED.WantedState.FIND_RANGE);
                    if (mCommitTuning.update(mControlBoard.getLowGear())) {
                        // Commit to TuningMap.
                        double rpm = mSuperstructure.getCurrentTuningRpm();
                        double range = mSuperstructure.getCurrentRange();
                        System.out.println("Tuning range: " + range + " = " + rpm);
                        mTuningFlywheelMap.put(new InterpolatingDouble(range), new InterpolatingDouble(rpm));
                        mSuperstructure.incrementTuningRpm();
                    }
                }

                // Exhaust has highest priority for intake.
                if (wantsExhaust) {
                    mSuperstructure.setWantIntakeReversed();
                } else if (mControlBoard.getIntakeButton()) {
                    mSuperstructure.setWantIntakeOn();
                } else if (!mSuperstructure.isShooting()) {
                    mSuperstructure.setWantIntakeStopped();
                }

                // Hanging has highest priority for feeder, followed by exhausting, unjamming, and finally
                // feeding.
                if (mControlBoard.getHangButton()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.HANG);
                } else if (wantsExhaust) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.EXHAUST);
                } else if (mControlBoard.getUnjamButton()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.UNJAM);
                } else if (mControlBoard.getFeedButton()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_FEED);
                } else if (mControlBoard.getRangeFinderButton()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.RANGE_FINDING);
                } else {
                    mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                }

                if (mControlBoard.getFlywheelSwitch()) {
                    mSuperstructure.setClosedLoopRpm(Constants.kDefaultShootingRPM);
                } else if (mControlBoard.getShooterOpenLoopButton()) {
                    mSuperstructure.setShooterOpenLoop(0);
                } else if (mControlBoard.getShooterClosedLoopButton()) {
                    mSuperstructure.setClosedLoopRpm(Constants.kDefaultShootingRPM);
                } else if (!mControlBoard.getHangButton() && !mControlBoard.getRangeFinderButton()) {
                    mSuperstructure.setShooterOpenLoop(0);
                }

                if (mControlBoard.getBlinkLEDButton()) {
                    mLED.setWantedState(LED.WantedState.BLINK);
                }
            }

            boolean score_gear = mControlBoard.getScoreGearButton();
            boolean grab_gear = mControlBoard.getGrabGearButton();

            if (score_gear && grab_gear) {
                mGearGrabber.setWantedState(WantedState.CLEAR_BALLS);
            } else if (score_gear) {
                mGearGrabber.setWantedState(WantedState.SCORE);
            } else if (grab_gear) {
                mGearGrabber.setWantedState(WantedState.ACQUIRE);
            } else {
                mGearGrabber.setWantedState(WantedState.IDLE);
            }

            mSuperstructure.setActuateHopper(!mControlBoard.getActuateHopperButton());
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

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            // If are tuning, dump map so far.
            if (Constants.kIsShooterTuning) {
                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
                    System.out.println("{" +
                            entry.getKey().value + ", " + entry.getValue().value + "},");
                }
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            mLED.setLEDOn();
        } else {
            mLED.setLEDOff();
        }

        zeroAllSensors();
        allPeriodic();
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = Feeder.getInstance().checkSystem();
        results &= MotorGearGrabber.getInstance().checkSystem();
        results &= Shooter.getInstance().checkSystem();
        results &= Hopper.getInstance().checkSystem();

        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
    }

    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        SmartDashboard.putBoolean("camera_connected", mVisionServer.isConnected());
    }
}
