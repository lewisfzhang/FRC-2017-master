package com.team254.lib.util;

import com.team254.frc2017.Constants;

public class SpeedController {
    private Path mPath;
    
    public SpeedController(Path path) {
        mPath = path;
    }

    public double getSpeed(Translation2d robotPos, double length) {
        double startSpd = mPath.getStartSpeed(); // 0
        double endSpd = mPath.getEndSpeed(); // 0
        double maxSpd = mPath.getMaxSpeed(); // 50

        length = mPath.getSegmentLength(); // 100
        double remainingDist = mPath.getSegmentRemainingDist(robotPos);
        double travelledDist = length - remainingDist;

        double accelSegmentSpeed = 0.5 * (startSpd + Math.sqrt(startSpd * startSpd + 2 * Constants.kMaxAccel * travelledDist));
        double deccelSegmentSpeed = 0.5 * (endSpd + Math.sqrt(endSpd * endSpd + 2 * Constants.kMaxDecel * remainingDist));

        return Math.min(accelSegmentSpeed, Math.min(deccelSegmentSpeed, maxSpd));
    }
    
}