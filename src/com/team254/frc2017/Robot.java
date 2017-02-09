package com.team254.frc2017;

import com.team254.frc2017.Constants.RobotName;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Drive;
import com.team254.frc2017.web.WebServer;

import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Subsystems
    private Drive mDrive = Drive.getInstance();

    // Other parts of the robot
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private WebServer mHTTPServer = new WebServer();

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        mDrive.registerEnabledLoops(mEnabledLooper);
        mHTTPServer.startServer();

        // initialize robot constants
        try {
            RobotName name = Constants.getRobotName();
            SmartDashboard.putString("MAC Address", Constants.getMACAddress());
            ConstantsModifier.initConstants(name);
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
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
        // Start loopers
        mEnabledLooper.start();

        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }


    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        try {
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();

            mDrive.setHighGear(!mControlBoard.getLowGear());
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn()));
        } catch (Throwable t) {

        }
    }

    @Override
    public void disabledInit() {
        mEnabledLooper.stop();

        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
    }
}