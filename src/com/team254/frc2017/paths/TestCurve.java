package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class TestCurve implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,273,0,0));
        sWaypoints.add(new Waypoint(96,273,20,80));
        sWaypoints.add(new Waypoint(124,307,6,80));
        sWaypoints.add(new Waypoint(157,307,0,80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 273), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":273},"speed":0,"radius":0,"comment":""},{"position":{"x":100,"y":273},"speed":60,"radius":20,"comment":""},{"position":{"x":132,"y":307},"speed":60,"radius":8,"comment":""},{"position":{"x":157,"y":307},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: TestCurve
}