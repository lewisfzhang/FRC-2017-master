package com.team254.frc2017.paths;

import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;

public interface PathContainer {
    Path buildPath();
    RigidTransform2d getStartPose();
    boolean isReversed();
}
