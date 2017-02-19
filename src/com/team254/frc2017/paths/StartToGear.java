package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class StartToGear {
    
    public static Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,72,0,0)); //start position
        sWaypoints.add(new Waypoint(80,72,12,30)); //arc center
        sWaypoints.add(new Waypoint(109,107,0,30)); //end pos

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 72), Rotation2d.fromDegrees(180.0)); 
    }

    public static boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":72},"speed":0,"radius":0,"comment":"start position"},{"position":{"x":80,"y":72},"speed":30,"radius":12,"comment":"arc center"},{"position":{"x":109,"y":107},"speed":30,"radius":0,"comment":"end pos"}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGear
}