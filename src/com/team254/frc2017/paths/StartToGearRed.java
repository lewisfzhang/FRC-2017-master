package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class StartToGearRed implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,89,0,0));
        sWaypoints.add(new Waypoint(85,89,0,120));
        sWaypoints.add(new Waypoint(104.5,89,0,40));
        sWaypoints.add(new Waypoint(114,109,0,40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 89), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":89},"speed":0,"radius":0,"comment":""},{"position":{"x":80,"y":89},"speed":30,"radius":0,"comment":""},{"position":{"x":109,"y":121},"speed":30,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGearRed
}