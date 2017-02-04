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
    public enum RobotName {
        COMP_BOT, PRAC_BOT, PROG_BOT
    }
    public static RobotName kRobotName;
    public static HashMap<String, RobotName> kMACAddresses = new HashMap<String, RobotName>();   
    private static NetworkInterface kNetworkInfo;
    
    // ROBOT PHYSICAL CONSTANTS
    public static double kTrackScrubFactor = 1.0; // FIXME placeholder
    public static double kTrackEffectiveDiameterInches = 25.0; // FIXME
                                                               // placeholder
    
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
        return kMACAddresses.get(getMACAddress());
    }

    static {
        kMACAddresses.put("herp", RobotName.COMP_BOT);
        kMACAddresses.put("derp", RobotName.PRAC_BOT);
        kMACAddresses.put("terp", RobotName.PROG_BOT);
        
        new Constants().loadFromFile();
    }
}
