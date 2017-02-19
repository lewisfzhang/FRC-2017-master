package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class GearToHopper {
    
    public static Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(109,107,0,0)); //gear position
        sWaypoints.add(new Waypoint(91,58,12,30));
        sWaypoints.add(new Waypoint(100,20,12,30));
        sWaypoints.add(new Waypoint(132,20,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(109, 107), Rotation2d.fromDegrees(180.0)); 
    }

    public static boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":109,"y":107},"speed":0,"radius":0,"comment":"gear position"},{"position":{"x":91,"y":58},"speed":30,"radius":12,"comment":""},{"position":{"x":100,"y":20},"speed":30,"radius":12,"comment":""},{"position":{"x":132,"y":20},"speed":20,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: GearToHopper
}