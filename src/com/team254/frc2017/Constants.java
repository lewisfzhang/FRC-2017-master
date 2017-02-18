package com.team254.frc2017;

import com.team254.lib.util.ConstantsBase;
import edu.wpi.first.wpilibj.Solenoid;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 * 
 * @TODO Update the robot MAC addresses (comp, practice, programming)
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;

    public static double kCenterOfTargetHeight = 100; // TODO: Fix

    public enum RobotName {
        COMP_BOT, PRAC_BOT, PROG_BOT
    }

    public static RobotName kRobotName;
    public static HashMap<String, RobotName> kMACAddresses = new HashMap<String, RobotName>();

    // ROBOT PHYSICAL CONSTANTS
    // Wheels
    public static double kDriveWheelDiameterInches = 3.419; //calibrated on 2/15/17
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 1.0;
    
    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    // Shooting suggestions
    public static double kOnTargetErrorThreshold = 1.0;

    // CONTROL LOOP GAINS
    // DRIVE GAINS

    // Wheel velocity - onboard talon
    public static double kDriveVelocityKp = 0.4;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 5.0;
    public static double kDriveVelocityKf = .15; //0.145;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // Turn to heading
    public static double kDriveTurnKp = 8;
    public static double kDriveTurnKi = 0.0;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.075;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 600;
    public static double kDriveTurnMaxAcc = 300;

    public static double kDriveTurnSimpleKp = 2.0;


    // SHOOTER GAINS
    // TODO: set some gains
    public static double kShooterKI = 0.0;
    public static double kShooterKF = 0.0;
    public static double kShooterBangLowThresholdFraction = 0.5;
    public static double kShooterSaturationDelay = 0.1;
    public static double kShooterMaxOutput = 10.0;
    public static double kShooterMinOutput = -10.0;
    public static double kShooterSaturationOutput = 10.0;

    public static double kShooterTalonKP = 0.015;
    public static double kShooterTalonKI = 5.0e-6;
    public static double kShooterTalonKD = 0;
    public static double kShooterTalonKF = 0.0228;


    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    // TALONS
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    // Drive
    public static final int kLeftDriveMasterId = 12;
    public static final int kLeftDriveSlaveId = 11;
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriverSlaveId = 4;

    //Feeder
    public static final int kFeederMasterId = 7;
    public static final int kFeederSlaveId = 8;

    // Intake
    public static final int kIntakeMasterId = 5;
    public static final int kIntakeSlaveId = 10;

    // Hopper / Floor
    public static final int kHopperMasterId = 6;
    public static final int kHopperSlaveId = 9;

    // Shooter
    public static final int kRightShooterMasterId = 1;
    public static final int kRightShooterSlaveId = 2;
    public static final int kLeftShooterSlave1Id = 13;
    public static final int kLeftShooterSlave2Id = 14;

    //PATH FOLLOWING
    public static double kAutoLookAhead = 25.0; //inches
    public static double kSegmentCompletionTolerance = 2.0; //inches
    public static double kMaxAccel = 25.0; //inches per second^2
    public static double kMaxDecel = 50.0; //should be positive
    public static double kPathFollowingMaxVel = 10000; //inches per second
    public static String kAutoFilePath = "~/path.txt"; //file path to the auto path file 
    public static double kMinSpeed = 10.0; //inches per second

    // SOLENOIDS
    public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0

    // Analog Inputs
    // TODO add some!

    // Camera Number
    public static int kPixyNumber = 1;

    // Phone
    public static int kAndroidAppTcpPort = 8254;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 0.3;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    // Robot state
    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = -6.454;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 19.75;
    public static double kCameraPitchAngleDegrees = 35.75;
    public static double kCameraYawAngleDegrees = -1.0;
    public static double kCameraDeadband = 0.0;

    // Flywheel PID
    // TODO give these actual values

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    /**
     * @return the robot's name, either COMP_BOT, PRAC_BOT, or PROG_BOT
     */
    public static RobotName getRobotName() {
        RobotName name = kMACAddresses.get(getMACAddress());
        if (name == null) {
            name = RobotName.COMP_BOT;
        }
        return name;
    }

    static {
        kMACAddresses.put("herp", RobotName.COMP_BOT);
        kMACAddresses.put("derp", RobotName.PRAC_BOT);
        kMACAddresses.put("00-80-2F-17-C8-2D", RobotName.PROG_BOT);

        new Constants().loadFromFile();
    }
}
