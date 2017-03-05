package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToHopperBlue implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16,234,0,0));
        sWaypoints.add(new Waypoint(93,234,48,100));
        sWaypoints.add(new Waypoint(93,295,0,100,"RamWall"));
        sWaypoints.add(new Waypoint(93,300,0,60));
        sWaypoints.add(new Waypoint(93,312,0,60));
        
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(16, 234), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
    // WAYPOINT_DATA: [{"position":{"x":16,"y":234},"speed":0,"radius":0,"comment":""},{"position":{"x":95,"y":234},"speed":60,"radius":48,"comment":""},{"position":{"x":95,"y":300},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: StartToHopperBlue
}