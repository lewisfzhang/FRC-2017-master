package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class CenterToHopperBlue implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(50,254,0,0));
        sWaypoints.add(new Waypoint(112,293,0,100));
        sWaypoints.add(new Waypoint(148,300,0,100));
        
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 100), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":72},"speed":0,"radius":0,"comment":"start position"},{"position":{"x":80,"y":72},"speed":30,"radius":12,"comment":"arc center"},{"position":{"x":109,"y":107},"speed":30,"radius":0,"comment":"end pos"}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGear
}