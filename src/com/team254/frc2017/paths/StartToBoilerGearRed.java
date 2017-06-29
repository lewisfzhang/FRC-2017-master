package com.team254.frc2017.paths;

import com.team254.frc2017.paths.profiles.PathAdapter;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;

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