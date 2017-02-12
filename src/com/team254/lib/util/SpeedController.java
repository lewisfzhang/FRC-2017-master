package com.team254.lib.util;

import com.team254.frc2017.Constants;

public class SpeedController {
    private Path mPath;
    
    public SpeedController(Path path) {
        mPath = path;
    }
    
    public double getSpeed(Translation2d robotPos, double length) {
        double startSpd = mPath.getStartSpeed();
        double endSpd = mPath.getEndSpeed();
        double maxSpd = mPath.getMaxSpeed();
        length = mPath.getSegmentLength();
        double dist = length - mPath.getSegmentRemainingDist(robotPos);
        System.out.println(length);
        System.out.println(dist);
        if(Double.isNaN(length))
            System.exit(1);
        
        if(maxSpd < endSpd && maxSpd < startSpd) {
            return maxSpd;
        }
        if(maxSpd > startSpd && maxSpd < endSpd) {
            return Math.min(Math.sqrt(2 * Constants.kMaxAccel * dist + startSpd * startSpd), maxSpd);
        }
        if(maxSpd > endSpd && maxSpd < startSpd) {
            return Math.min(Math.sqrt(2 * Constants.kMaxDecel * (length - dist) + endSpd * endSpd), maxSpd);
        }
        double intersection = ( endSpd * endSpd - startSpd * startSpd - 2 * Constants.kMaxDecel * length ) / (2 * Constants.kMaxAccel - 2 * Constants.kMaxDecel);
        if(dist <= intersection)
            return Math.min(Math.sqrt(2 * Constants.kMaxAccel * dist + startSpd * startSpd), maxSpd);
        else
            return Math.min(Math.sqrt(2 * Constants.kMaxDecel * (length - dist) + endSpd * endSpd), maxSpd);
    }
    
}