package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class GearToHopperBlue implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(112,215,0,0));
        sWaypoints.add(new Waypoint(103,235,10,50));
        sWaypoints.add(new Waypoint(138,287,28,50));
        sWaypoints.add(new Waypoint(108,302,0,50,"DeployIntake"));
        sWaypoints.add(new Waypoint(90,302,0,30));


        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(112, 215), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":109,"y":107},"speed":0,"radius":0,"comment":"gear position"},{"position":{"x":91,"y":58},"speed":30,"radius":12,"comment":""},{"position":{"x":100,"y":20},"speed":30,"radius":12,"comment":""},{"position":{"x":132,"y":20},"speed":20,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: GearToHopper
}