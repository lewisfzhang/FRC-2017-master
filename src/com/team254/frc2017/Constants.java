package com.team254.frc2017;

import com.team254.lib.util.ConstantsBase;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;

    // ROBOT PHYSICAL CONSTANTS
    public static double kTrackScrubFactor = 1.0; // FIXME placeholder
    public static double kTrackEffectiveDiameterInches = 25.0; // FIXME
                                                               // placeholder
    
    //pixycam distance stuff
    public static double kCameraAngle = 26.5; // degrees, originally 26.5
    public static double kFocalX = 316.736; //default for PixyCam 1
    public static int kPixySPIRefreshRate = 500000;
    public static double kBoilerHeight = 54; // distance from camera to center of top boiler tape strip

    

    public static double kCollisionThreshold = 0.5;
    // CONTROL LOOP GAINS
    // TODO add some!

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    // TALONS
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    // TODO add some!

    // SOLENOIDS
    // TODO add some!

    // Analog Inputs
    // TODO add some!
    
    //Camera Number
    public static int kPixyNumber = 1;
    
    //Feeder
    public static double kFeedKp = 0.0;
    public static double kFeedKi = 0.0;
    public static double kFeedKd = 0.0;
    public static double kFeedKf = 0.0;
    public static int kFeedIZone = 0;
    public static double kFeedRampRate = 10000; //hella ramp
    public static int kFeedVelocityAllowableError = 0;
    
    public static double kFeedRPM = 1000;

    // Flywheel PID
    // TODO give these actual values

    /* 2" single wheel shooter constants
    public static boolean kFlywheelMotor1Inverted = true;
    public static boolean kFlywheelMotor2Inverted = false;
    public static boolean kFlywheelEncoderInverted = false;
    public static double kFlywheelReduction = 1.0;
    public static double kFlywheelKp = 0.008;
    public static double kFlywheelKi = 0.001;
    public static double kFlywheelKd = 0.0;
    public static double kFlywheelKf = 0.001;
    public static double kFlywheelOnTargetTolerance = 20.0;
    public static double kFlywheelTarget = 9000.0;
    public static double kFlywheelFeedRollerVoltage = 9.0;
    */
    
    /* 4" single wheel shooter constants */
    public static boolean kFlywheelMotor1Inverted = true;
    public static boolean kFlywheelMotor2Inverted = false;
    public static boolean kFlywheelEncoderInverted = true;
    public static double kFlywheelReduction = 1.0;
    public static double kFlywheelKp = 0.003;
    public static double kFlywheelKi = 0.001;
    public static double kFlywheelKd = 0.0;
    public static double kFlywheelKf = 0.0007;
    public static double kFlywheelOnTargetTolerance = 20.0;
    public static double kFlywheelTarget = 4000.0;
    public static double kFlywheelFeedRollerVoltage = 9.0;
        
    /* Triple roller shooter constants
    public static boolean kFlywheelMotor1Inverted = true;
    public static boolean kFlywheelMotor2Inverted = false;
    public static boolean kFlywheelEncoderInverted = true;
    public static double kFlywheelKp = 0.005;
    public static double kFlywheelKi = 0.001;
    public static double kFlywheelKd = 0.0;
    public static double kFlywheelKf = 0.00085;
    public static double kFlywheelOnTargetTolerance = 20.0;
    public static double kFlywheelTarget = 2800.0;
     */

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    // DIGITAL IO
    // TODO add some!

    // PWM
    // TODO add some!

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    static {
        new Constants().loadFromFile();
    }
}
