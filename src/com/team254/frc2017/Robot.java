package com.team254.frc2017;

import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Drive;
import com.team254.frc2017.subsystems.Proto_Feeder;
import com.team254.frc2017.subsystems.Proto_Intake;
import com.team254.frc2017.subsystems.Proto_Shooter;
import com.team254.frc2017.web.WebServer;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    private Drive mDrive = Drive.getInstance();
    //private Proto_Intake mIntake = Proto_Intake.getInstance();
    private Proto_Shooter mShooter = Proto_Shooter.getInstance();
    private Proto_Feeder mFeeder = Proto_Feeder.getInstance();

    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private WebServer mHTTPServer = new WebServer();

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        // mDrive.registerEnabledLoops(mEnabledLooper);
        //mIntake.registerEnabledLoops(mEnabledLooper);
        mShooter.registerEnabledLoops(mEnabledLooper);
        mHTTPServer.startServer();
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
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {

        //Start loopers
        mEnabledLooper.start();
        
        //re-update feeder constants & apply to talons TODO: remove this later
        mFeeder.updateConstants();

        mShooter.updateConstants();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {

        if (mControlBoard.getSpinShooterButton()) {
            mShooter.setRpmSetpoint(Constants.kFlywheelTarget);
        } else {
            mShooter.setRpmSetpoint(0.0);
        }

        if (mControlBoard.getShootButton()) {
            mFeeder.setSetpoint(/*Constants.kFeedRPM*/5);
        } else {
            mFeeder.setSetpoint(0.0);
        }
        mShooter.outputToSmartDashboard();
        mFeeder.outputToSmartDashboard();
    }

    @Override
    public void disabledInit() {
        mEnabledLooper.stop();
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
    }
}