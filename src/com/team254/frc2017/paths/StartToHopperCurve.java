package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToHopperCurve implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,273,0,0));
        sWaypoints.add(new Waypoint(36,273,12,40));
        sWaypoints.add(new Waypoint(60,304,18,40,"DeployIntake"));
        sWaypoints.add(new Waypoint(101.5,304,0,40));

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
    // WAYPOINT_DATA: [{"position":{"x":16,"y":273},"speed":0,"radius":0,"comment":""},{"position":{"x":36,"y":273},"speed":60,"radius":12,"comment":""},{"position":{"x":60,"y":303},"speed":60,"radius":18,"comment":""},{"position":{"x":95,"y":303},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: StartToHopperCurve
}