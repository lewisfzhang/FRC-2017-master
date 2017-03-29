package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToBoilerGearBlue implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,235,0,0));
        sWaypoints.add(new Waypoint(101,235,18,60));
        sWaypoints.add(new Waypoint(116,212,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 235), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":273},"speed":0,"radius":0,"comment":""},{"position":{"x":88,"y":273},"speed":60,"radius":40,"comment":""},{"position":{"x":120,"y":216},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGearBlue
}