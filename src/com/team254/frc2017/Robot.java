package com.team254.frc2017;

import java.util.List;

import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.pixy.VisionServer;
import com.team254.frc2017.subsystems.Drive;
import com.team254.frc2017.subsystems.Proto_Feeder;
import com.team254.frc2017.subsystems.Proto_Intake;
import com.team254.frc2017.subsystems.Proto_Shooter;
import com.team254.frc2017.web.WebServer;
import com.team254.lib.util.pixy.*;
import com.team254.lib.util.pixy.Frame.*;
import com.team254.lib.util.pixy.constants.PixyNumberConstants;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    /*
    private Drive mDrive = Drive.getInstance();
    private Proto_Intake mIntake = Proto_Intake.getInstance();
    private Proto_Shooter mShooter = Proto_Shooter.getInstance();
    private Proto_Feeder mFeeder = Proto_Feeder.getInstance();*/
    private PixyCam mCamera = new PixyCam();

    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private WebServer mHTTPServer = new WebServer();
    
    private VisionServer mVisionServer = VisionServer.getInstance();

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        // mDrive.registerEnabledLoops(mEnabledLooper);
        // mIntake.registerEnabledLoops(mEnabledLooper);
        // mShooter.registerEnabledLoops(mEnabledLooper);
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
        System.out.println("CX: " + PixyNumberConstants.cx + "\nCY: " + PixyNumberConstants.cy);
        
        //re-update feeder constants & apply to talons TODO: remove this later
        //mFeeder.updateConstants();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Frame kFrame = mCamera.getFrame();
        List<Frame.Block> kBlocks = kFrame.getBlocks();
        if(kBlocks.size() > 0) {     
            Block target = kBlocks.get(0);
            for (Block b : kBlocks) {
                if (b.centerY < target.centerY) {
                    target = b;
                }
            }
            
            SmartDashboard.putNumber("Height", target.height);
            SmartDashboard.putNumber("Width", target.width);
            SmartDashboard.putNumber("Distance", getPhysicalDistance(target));
            System.out.println(getPhysicalDistance(target));
        } else {
            System.out.println("No targets detected.");
        }
        /*
        if (mControlBoard.getSpinShooterButton()) {
            mShooter.setRpmSetpoint(Constants.kFlywheelTarget);
        } else {
            mShooter.setManualVoltage(0.0);
        }

        if (mControlBoard.getShootButton()) {
            mFeeder.setSetpoint(Constants.kFeedRPM);
        } else {
            mFeeder.setSetpoint(0.0);
        }
        mShooter.outputToSmartDashboard();
        mFeeder.outputToSmartDashboard();
        */
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
    
    private double getPhysicalDistance(Block target) {
//        System.out.println("Target Center: " + target.centerY);
//        System.out.println("Y Angle Pos: " + Math.atan(-target.centerY/Constants.kFocalX));
        return Constants.kBoilerHeight * 1/(Math.tan(Math.atan(-target.centerY/Constants.kFocalX)+ Math.toRadians(Constants.kCameraAngle)));
    }
}