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
    public static double kFlywheelReduction = 1.0;
    
    public static boolean kFlywheelAMotor1Inverted = true;
    public static boolean kFlywheelAMotor2Inverted = true;
    public static boolean kFlywheelAEncoderInverted = false;
    public static double kFlywheelAKp = 0.003;
    public static double kFlywheelAKi = 0.001;
    public static double kFlywheelAKd = 0.0;
    public static double kFlywheelAKf = 0.0023;
    public static double kFlywheelATarget = 3150.0;
    public static double kFlywheelAKj = 3.0;
    public static double kFlywheelAKLoadRatio = 1.026;
    
    public static boolean kFlywheelBMotor1Inverted = false;
    public static boolean kFlywheelBMotor2Inverted = true;
    public static boolean kFlywheelBEncoderInverted = true;
    public static double kFlywheelBKp = 0.003;
    public static double kFlywheelBKi = 0.001;
    public static double kFlywheelBKd = 0.0;
    public static double kFlywheelBKf = 0.0025;
    public static double kFlywheelBTarget = 4000.0;
    
    public static double kFlywheelOnTargetTolerance = 20.0;
    
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
