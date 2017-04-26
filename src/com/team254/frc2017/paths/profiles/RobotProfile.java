package com.team254.frc2017.paths.profiles;

/**
 * Holds corrective values for how each robot actually drives
 */
public interface RobotProfile {
    
    //red
    public double getRedBoilerGearXCorrection();
    
    public double getRedBoilerGearYCorrection();
    
    public double getRedHopperXOffset();
    
    public double getRedHopperYOffset();
    
    //blue
    public double getBlueBoilerGearXCorrection();
    
    public double getBlueBoilerGearYCorrection();
    
    public double getBlueHopperXOffset();
    
    public double getBlueHopperYOffset();
 
}
