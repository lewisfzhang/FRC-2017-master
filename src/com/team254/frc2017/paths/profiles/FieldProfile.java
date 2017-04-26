package com.team254.frc2017.paths.profiles;

/**
 * Holds field measurements
 */
public interface FieldProfile {
 
    public double getRedCenterToBoiler();
    
    public double getRedWallToAirship();
    
    public double getRedCenterToHopper();
    
    public double getRedWallToHopper();
    
    public double getBlueCenterToBoiler();
    
    public double getBlueWallToAirship();
    
    public double getBlueCenterToHopper();
    
    public double getBlueWallToHopper();
    
}
