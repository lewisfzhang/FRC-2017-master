package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class TestArcPath extends PathBuilder {
    
    public static Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0,0,0,0));
        sWaypoints.add(new Waypoint(100,0,36,100));
        sWaypoints.add(new Waypoint(100,100,36,100));
        sWaypoints.add(new Waypoint(200,100,0,100));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0)); 
    }

    
}
