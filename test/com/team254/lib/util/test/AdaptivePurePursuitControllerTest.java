package com.team254.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.Test;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.*;
import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Path;
import com.team254.lib.util.PathSegment;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class AdaptivePurePursuitControllerTest {
    private static final double kEpsilon = 1E-9;
    private static final double kReallyBigNumber = 1E9;
    
    @Test
    public void testJoinRadius() {
        // Robot is at the origin, facing positive x
        RigidTransform2d robot_pose = new RigidTransform2d();

        // Lookahead point is straight ahead
        Translation2d lookahead_point = new Translation2d(30, 0);
        //Radius should be yuuuuge
        assertTrue(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point) > kReallyBigNumber);
      
        robot_pose.setRotation(Rotation2d.fromDegrees(45));
        lookahead_point = new Translation2d(30, 30);
        assertTrue(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point) > kReallyBigNumber);

        robot_pose.setRotation(Rotation2d.fromDegrees(-45));
        lookahead_point = new Translation2d(30, -30);
        assertTrue(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point) > kReallyBigNumber);
        
        robot_pose.setRotation(Rotation2d.fromDegrees(180));
        lookahead_point = new Translation2d(-30, 0);
        assertTrue(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point) > kReallyBigNumber);
        
        robot_pose.setRotation(Rotation2d.fromDegrees(-20));
        lookahead_point = new Translation2d(40, 10);
        assertEquals(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point), 36.83204234182525, kEpsilon);
        
        robot_pose.setRotation(Rotation2d.fromDegrees(-130));
        lookahead_point = new Translation2d(-40, 10);
        System.out.println(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point));
        assertEquals(AdaptivePurePursuitController.getRadius(robot_pose, lookahead_point), 22.929806792642722, kEpsilon);
    }

    @Test
    public void testArcPath() {
        Path path = TestArcPath.buildPath();
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(path, false);

        double dt = .01;

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        while (!controller.isFinished() && t < 250) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(robot_pose);
            robot_pose = robot_pose.transformBy(new RigidTransform2d(new Translation2d(command.dx * dt, 0),
                    Rotation2d.fromRadians(command.dtheta * dt)));

            System.out.println(
                    "t = " + t + ", lin vel " + command.dx + ", ang vel " + command.dtheta + ", pose " + robot_pose);
            t += dt;
        }
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertEquals(200, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance);
        assertEquals(100, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance);
    }

//    @Test
//    public void testControllerReversed() {
//        List<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Translation2d(0, 0), 1));
//        waypoints.add(new Waypoint(new Translation2d(1, 0), 1));
//        waypoints.add(new Waypoint(new Translation2d(2, 0), 2, "StartedTurn"));
//        waypoints.add(new Waypoint(new Translation2d(2, -1), 2));
//        waypoints.add(new Waypoint(new Translation2d(2, -2), 1, "FinishedTurn"));
//        waypoints.add(new Waypoint(new Translation2d(3, -2), 1));
//        waypoints.add(new Waypoint(new Translation2d(4, -2), 3));
//        waypoints.add(new Waypoint(new Translation2d(5, -2), 1));
//        Path path = new Path(waypoints);
//
//        double dt = .01;
//        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(0.25, 1.0, dt, path, true,
//                kEpsilon);
//
//        RigidTransform2d robot_pose = RigidTransform2d.fromRotation(Rotation2d.fromRadians(Math.PI));
//        double t = 0;
//        while (!controller.isDone() && t < 10) {
//            // Follow the path
//            RigidTransform2d.Delta command = controller.update(robot_pose, t);
//            robot_pose = robot_pose.transformBy(new RigidTransform2d(new Translation2d(command.dx * dt, 0),
//                    Rotation2d.fromRadians(command.dtheta * dt)));
//
//            System.out.println(
//                    "t = " + t + ", lin vel " + command.dx + ", ang vel " + command.dtheta + ", pose " + robot_pose);
//            t += dt;
//        }
//        assertTrue(controller.isDone());
//        assertEquals(2, controller.getMarkersCrossed().size());
//        assertEquals(5, robot_pose.getTranslation().getX(), .01);
//        assertEquals(-2, robot_pose.getTranslation().getY(), .01);
//    }
}