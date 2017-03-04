package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class ShootToLoadBlue implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(90,245,0,0));
        sWaypoints.add(new Waypoint(250,245,100,120));
        sWaypoints.add(new Waypoint(416,100,0,120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(90, 245), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
    // WAYPOINT_DATA: [{"position":{"x":90,"y":245},"speed":0,"radius":0,"comment":""},{"position":{"x":250,"y":245},"speed":120,"radius":100,"comment":""},{"position":{"x":416,"y":100},"speed":120,"radius":0,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: ShootToLoadBlue
}