package com.team254.frc2017.paths;

import com.team254.lib.util.Path;
import com.team254.lib.util.PathSegment;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class TestArcPath {
    
    public static Path buildPath() {
        Path sPath = new Path();
        sPath.addSegment(new PathSegment.Translation(0,0,64,0,100,sPath.getLastMotionState(), 100));
        sPath.addSegment(new PathSegment.Translation(64,0,100,36,64,36,100,sPath.getLastMotionState(), 100));
        sPath.addSegment(new PathSegment.Translation(100,36,100,64,100,sPath.getLastMotionState(), 100));
        sPath.addSegment(new PathSegment.Translation(100,64,136,100,136,64,100,sPath.getLastMotionState(), 60));
        sPath.addSegment(new PathSegment.Translation(136,100,200,100,60,sPath.getLastMotionState(), 0));
        sPath.extrapolateLast();
        return sPath;
    }
    
    public static RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0)); 
    }
    
}
