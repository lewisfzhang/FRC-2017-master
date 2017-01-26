package com.team254.lib.util;

import com.team254.frc2017.Constants;

public class SpeedController {
    private Path mPath;
    
    public SpeedController(Path path) {
        mPath = path;
    }
    
    public double getSpeed(Translation2d robotPos) {
        double startSpd = mPath.getStartSpeed();
        double endSpd = mPath.getEndSpeed();
        double maxSpd = mPath.getMaxSpeed();
        double length = mPath.getSegmentLength();
        double dist = length - mPath.getSegmentRemainingDist(robotPos);
        System.out.println("Start: "+startSpd);
        System.out.println("End: "+endSpd);
        System.out.println("Max: "+maxSpd);
        System.out.println("Length: "+length);
        System.out.println("Dist: "+dist);
        
        if(maxSpd < endSpd && maxSpd < startSpd) {
            return maxSpd;
        }
        if(maxSpd > startSpd && maxSpd < endSpd) {
            return Math.min(Constants.kMaxAccel * dist + startSpd, maxSpd);
        }
        if(maxSpd > endSpd && maxSpd < startSpd) {
            return Math.min(Constants.kMaxDecel * (length - dist) + endSpd, maxSpd);
        }
        double intersection = ( Constants.kMaxDecel*length + endSpd - startSpd ) / (Constants.kMaxAccel + Constants.kMaxDecel);
        if(dist <= intersection)
            return Math.min(Constants.kMaxAccel * dist + startSpd, maxSpd);
        else
            return Math.min(Constants.kMaxDecel * (length - dist) + endSpd, maxSpd);
    }
    
}
