package com.team254.frc2017;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
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
    //Drive mDrive = Drive.getInstance();
    //Proto_Intake mIntake = Proto_Intake.getInstance();
    Proto_Shooter mShooterA = new Proto_Shooter(1, 2, Constants.kFlywheelAMotor1Inverted, Constants.kFlywheelAMotor2Inverted, 0, 1, Constants.kFlywheelAEncoderInverted, 0, 0, 0, 0, "A");
    Proto_Shooter mShooterB = new Proto_Shooter(4, 5, Constants.kFlywheelBMotor1Inverted, Constants.kFlywheelBMotor2Inverted, 2, 3, Constants.kFlywheelBEncoderInverted, 0, 0, 0, 0, "B");
    //Proto_Feeder mFeeder = Proto_Feeder.getInstance();

    ControlBoard mControlBoard = ControlBoard.getInstance();

    Looper mEnabledLooper = new Looper();

    WebServer mHTTPServer = new WebServer();
    
    CANTalon feeda, feedb;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        feeda = new CANTalon(3);
        feeda.changeControlMode(TalonControlMode.Voltage);
        feedb = new CANTalon(6);
        feedb.changeControlMode(TalonControlMode.Voltage);
        // mDrive.registerEnabledLoops(mEnabledLooper);
        // mIntake.registerEnabledLoops(mEnabledLooper);
        mShooterA.registerEnabledLoops(mEnabledLooper);
        mShooterB.registerEnabledLoops(mEnabledLooper);
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
        mEnabledLooper.start();
        
        //re-update feeder constants & apply to talons
        //mFeeder.updateConstants();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {

        if (mControlBoard.getSpinShooterButton()) {
            mShooterA.setRpmSetpoint(Constants.kFlywheelTarget);
            mShooterB.setRpmSetpoint(Constants.kFlywheelTarget);
        } else {
            mShooterA.setManualVoltage(0.0);
            mShooterB.setManualVoltage(0.0);
        }

        if (mControlBoard.getShootButton()) {
            //mFeeder.setSetpoint(Constants.kFeedRPM);
            feeda.set(-9);
            feedb.set(-9);
        } else {
            //mFeeder.setSetpoint(0.0);
            feeda.set(0);
            feedb.set(0);
        }
        mShooterA.outputToSmartDashboard();
        mShooterB.outputToSmartDashboard();
        //mFeeder.outputToSmartDashboard();
        
        // Set gains for tuning
        mShooterA.setPIDF(Constants.kFlywheelAKp, Constants.kFlywheelAKi, Constants.kFlywheelAKd, Constants.kFlywheelAKf);
        mShooterB.setPIDF(Constants.kFlywheelBKp, Constants.kFlywheelBKi, Constants.kFlywheelBKd, Constants.kFlywheelBKf);
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