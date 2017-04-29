package com.team254.frc2017.paths.profiles;

public class CompBot implements RobotProfile {

    @Override
    public double getRedBoilerGearXCorrection() {
        return 0.5;
    }

    @Override
    public double getRedBoilerGearYCorrection() {
        return 3.0;
    }

    @Override
    public double getRedHopperXOffset() {
        return 0.0;
    }

    @Override
    public double getRedHopperYOffset() {
        return -5.0;
    }

    @Override
    public double getBlueBoilerGearXCorrection() {
        return 2.5;
    }

    @Override
    public double getBlueBoilerGearYCorrection() {
        return 1.0;
    }

    @Override
    public double getBlueHopperXOffset() {
        return -4.0;
    }

    @Override
    public double getBlueHopperYOffset() {
        return 3.0;
    }
    
}
