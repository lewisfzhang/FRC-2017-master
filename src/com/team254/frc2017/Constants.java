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

    // Flywheel PID
    // TODO give these actual values
    public static double kFlywheelReduction = 3.0;
//    public static double kFlywheelKp = 0.00070175438/30;
    public static double kFlywheelKp = 0.005;
    public static double kFlywheelKi = 0.001;
    public static double kFlywheelKd = 0.0;
    public static double kFlywheelKf = 0.00085;
    public static double kFlywheelOnTargetTolerance = 20.0;
    public static double kFlywheelTarget = 2800.0;
    public static double kFlywheelFeedRollerVoltage = 9.0;

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
