package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class BoilerToHopperBlue implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,274,0,0));
        sWaypoints.add(new Waypoint(42,274,16,40));
        sWaypoints.add(new Waypoint(74,303,16,40,"DeployIntake"));
        sWaypoints.add(new Waypoint(103,303,0,40,"PathFinished"));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 50), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":26,"y":36},"speed":0,"radius":0,"comment":""},{"position":{"x":42,"y":20},"speed":40,"radius":10,"comment":""},{"position":{"x":100,"y":20},"speed":40,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: BoilerToHopperRed
}