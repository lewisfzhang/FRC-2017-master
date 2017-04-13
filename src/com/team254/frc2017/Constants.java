package com.team254.frc2017;

import com.team254.lib.util.ConstantsBase;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.math.PolynomialRegression;

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

    // Target parameters
    // Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
    // Section 3.13
    // ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
    // Parts GE-17203-FLAT and GE-17371 (sheet 7)
    public static double kBoilerTargetTopHeight = 88.0;  // TODO: Verify on field
    public static double kBoilerRadius = 7.5;  // TODO: Verify on field

    public static boolean kIsShooterTuning = false;
    public static double kShooterTuningRpmFloor = 2900;
    public static double kShooterTuningRpmCeiling = 3500;
    public static double kShooterTuningRpmStep = 50;

    public static double kShooterTuningRpm = 3500.0;

    public enum RobotName {
        COMP_BOT, PRAC_BOT, PROG_BOT
    }

    public static RobotName kRobotName;
    public static HashMap<String, RobotName> kMACAddresses = new HashMap<String, RobotName>();

    // ROBOT PHYSICAL CONSTANTS
    // Wheels
    public static double kDriveWheelDiameterInches = 3.419; //calibrated on 2/15/17
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 0.924;
    
    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    // Shooting suggestions
    public static double kOnTargetErrorThreshold = 3.0;

    // Voltages
    public static double kIntakeVoltageMax = 7.5;
    public static double kIntakeVoltageMin = 4.5;
    public static final double kIntakeVoltageDifference = kIntakeVoltageMax - kIntakeVoltageMin;


    // CONTROL LOOP GAINS
    // DRIVE GAINS

    // Wheel velocity - High gear
    // Setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 0.5;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 4.0;
    public static double kDriveHighGearVelocityKf = .15;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 15.0 * 12.0;  // 15 fps

    // Wheel position - Low gear
    // Setpoint is in rotations.
    // Error is in encoder ticks (4096 per revolution)
    // Output is +/- 1023, where 1023 corresponds to +12V
    public static double kDriveLowGearPositionKp = 1.0;
    public static double kDriveLowGearPositionKi = 0.002;
    public static double kDriveLowGearPositionKd = 100.0;
    public static double kDriveLowGearPositionKf = .45;
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 240.0;  // V/s
    public static double kDriveLowGearNominalOutput = 0.5;  // V
    public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches);  // 6 fps in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches);  // 18 fps/s in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;

    // Turn to heading
    public static double kDriveTurnKp = 3.0;
    public static double kDriveTurnKi = 1.5;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.0;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 360.0;
    public static double kDriveTurnMaxAcc = 720.0;


    // SHOOTER GAINS
    public static double kShooterTalonKP = 0.16;
    public static double kShooterTalonKI = 0.00008;
    public static double kShooterTalonKD = 0.0;
    public static double kShooterTalonKF = 0.035;
    public static double kShooterRampRate = 60.0;

    public static double kShooterTalonHoldKP = 0.0;
    public static double kShooterTalonHoldKI = 0.0;
    public static double kShooterTalonHoldKD = 0.0;
    public static double kShooterHoldRampRate = 360.0;

    public static double kShooterVoltageCompensationRampRate = 10.0;
    public static int kShooterTalonIZone = 1000;// 73 rpm
    public static int kShooterOpenLoopCurrentLimit = 35;

    public static double kShooterSetpointDeadbandRpm = 1.0;

    // Used to determine when to switch to hold profile.
    public static double kShooterStartOnTargetRpm = 30.0;
    public static double kShooterStopOnTargetRpm = 150.0;
    public static double kShooterMinOnTargetDuration = 0.1;  // seconds

    // Feeder gains
    public static double kFeederKP = 0.02;
    public static double kFeederKI = 0.0;
    public static double kFeederKD = 0.2;
    public static double kFeederKF = 0.009;
    public static double kFeederRampRate = 240.0;
    public static double kFeederVoltageCompensationRampRate = 10.0;
    public static double kFeederFeedSpeedRpm = 4500.0;
    public static double kFeederSensorGearReduction = 3.0;
    
    // Hopper gains
    public static double kHopperRampRate = 48.0;

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
    public static final int kFeederMasterId = 8;
    public static final int kFeederSlaveId = 7;

    // Intake
    public static final int kIntakeMasterId = 5;
    public static final int kIntakeSlaveId = 10;

    // Hopper / Floor
    public static final int kHopperMasterId = 6;
    public static final int kHopperSlaveId = 9;

    // Shooter
    public static final int kRightShooterMasterId = 2;
    public static final int kRightShooterSlaveId = 1;
    public static final int kLeftShooterSlave1Id = 13;
    public static final int kLeftShooterSlave2Id = 14;

    // Grabber
    public static final int kGearGrabberId = 15;

    // SOLENOIDS
    public static final int kShifterSolenoidId = 0; // PCM 0, Solenoid 0
    public static final int kIntakeDeploySolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
    public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7

    // Analog Inputs
    public static int kLeftBallSensorId = 1;
    public static int kRightBallSensorId = 0;
    public static int kLEDOnId = 2;

    // Digital Outputs
    public static int kGreenLEDId = 9;
    public static int kRangeLEDId = 8;

    // Relays
    public static int kLEDRelayId = 0;

    // Camera Number
    public static int kPixyNumber = 1;

    // Phone
    public static int kAndroidAppTcpPort = 8254;

    //PATH FOLLOWING
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 35.0; // inches per second
    public static double kMaxLookAhead = 18.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
    
    public static double kInertiaSteeringGain = 0.0; //0005; // angular velocity command is multiplied by this gain * our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 2.0; //inches
    public static double kPathFollowingMaxAccel = 120.0; //inches per second^2
    public static double kPathFollowingMaxVel = 120.0; //inches per second
    public static double kPathFollowingProfileKp = 0.75;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;


    // Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    // Robot state
    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = -3.3211;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 20.9;
    public static double kCameraPitchAngleDegrees = 30.0;
    public static double kCameraYawAngleDegrees = 0.25;
    public static double kCameraDeadband = 0.0;

    // Flywheel PID
    // TODO give these actual values

    public static double kDefaultShootingDistanceInches = 95.8;
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = true;  // Change to 'true' to use the best-fit polynomial instead.
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;


    public static double kShooterOptimalRange = 100.0;
    public static double kShooterOptimalRangeFloor = 87.0;
    public static double kShooterOptimalRangeCeiling = 115.0;

    public static double kShooterAbsoluteRangeFloor = 85.0;
    public static double kShooterAbsoluteRangeCeiling = 135.0;

    public static double[][] kFlywheelDistanceRpmValues = {
            // Pre-champs 4/9
            {89.14, 2900.0},
            {94.87, 2950.0},
            {104.33, 3000.0},
            {107.64, 3050.0},
            {114.34, 3100.0},
            {121.23, 3150.0},
            {125.18, 3200.0},
            {128.23, 3250.0}
    };

    static {
        for (double[] pair : kFlywheelDistanceRpmValues) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        kDefaultShootingRPM = kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(Constants.kDefaultShootingDistanceInches)).value;
        
        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelDistanceRpmValues, 2);
    }


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
        kMACAddresses.put("00-80-2F-25-78-B0", RobotName.PROG_BOT);

        new Constants().loadFromFile();
    }
}
