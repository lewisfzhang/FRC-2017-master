package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToBoilerGearRed implements PathContainer {
    
    @Override
    public Path buildPath() {
        return PathAdapter.getRedGearPath();
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getRedStartPose(); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}