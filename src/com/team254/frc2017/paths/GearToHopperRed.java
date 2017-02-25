package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class GearToHopperRed implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(110,109,0,0)); //dat gear doe
        sWaypoints.add(new Waypoint(150,73,12,100));
        sWaypoints.add(new Waypoint(130,30,4,100, "DeployIntake"));
        sWaypoints.add(new Waypoint(91.5,22,0,100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(114, 109), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":109,"y":107},"speed":0,"radius":0,"comment":"gear position"},{"position":{"x":91,"y":58},"speed":30,"radius":12,"comment":""},{"position":{"x":100,"y":20},"speed":30,"radius":12,"comment":""},{"position":{"x":132,"y":20},"speed":20,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: GearToHopper
}