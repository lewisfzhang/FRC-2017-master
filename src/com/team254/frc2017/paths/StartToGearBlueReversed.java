package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToGearBlueReversed implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(112,217,0,0));
        sWaypoints.add(new Waypoint(80,264,40,20));
        sWaypoints.add(new Waypoint(16,264,0,20));


        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 273), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":273},"speed":0,"radius":0,"comment":""},{"position":{"x":88,"y":273},"speed":60,"radius":40,"comment":""},{"position":{"x":120,"y":216},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToGearBlue
}