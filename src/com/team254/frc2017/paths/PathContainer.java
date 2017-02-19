package com.team254.frc2017.paths;

import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;

public interface PathContainer {
    Path buildPath();
    RigidTransform2d getStartPose();
    boolean isReversed();
}
