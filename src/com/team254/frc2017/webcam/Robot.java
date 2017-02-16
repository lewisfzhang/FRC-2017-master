package com.team254.frc2017.webcam;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    CameraServer mCameraServer = CameraServer.getInstance();
    UsbCamera mUsbCamera;
    CvSource mOutputStream;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mUsbCamera = mCameraServer.startAutomaticCapture();
        mUsbCamera.setResolution(320, 240);
        mUsbCamera.setFPS(15);
        mUsbCamera.setExposureManual(0);
        mOutputStream = mCameraServer.putVideo("Blur", 320, 240);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {}

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        CvSink video = mCameraServer.getVideo(mUsbCamera);
        Mat source = new Mat();
        video.grabFrame(source);
        Mat gray_img = new Mat();
        Imgproc.cvtColor(source, gray_img, Imgproc.COLOR_BGR2GRAY);
        Mat thresholded_img = new Mat();
        // Binary threshold
        Imgproc.threshold(gray_img, thresholded_img, 0, 255, 0);
        mOutputStream.putFrame(thresholded_img);
    }

    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {}
}