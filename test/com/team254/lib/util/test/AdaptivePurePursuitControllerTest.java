package com.team254.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.Test;

import com.team254.frc2017.Constants;
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
    public void testPath1() {
        //Make a super simple path to drive straight forwards 200 inches
        Path path = new Path();
        path.addSegment(new PathSegment.Translation(0.0, 0.0, 200.0, 0.0, 50.0));
        
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(path);

        double dt = .01;

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        while (!controller.isFinished() && t < 100) {
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
        assertEquals(0, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance);
    }
    
    @Test
    public void testPath2() {
        //Test a path with some arcs in it
        Path path = new Path();
        path.addSegment(new PathSegment.Translation(0.0,0.0,136.0,0.0,40.0));
        path.addSegment(new PathSegment.Translation(136,0,160,24,136,24,40));
        path.addSegment(new PathSegment.Translation(160.0,24.0,160.0,100.0,40.0));

        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(path);

        double dt = .01;

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        while (!controller.isFinished() && t < 100) {
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
        assertEquals(160, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance);
        assertEquals(100, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance);
    }

    @Test
    public void testPath3() {
        //Test a path with some sharper turns
        Path path = new Path();
        path.addSegment(new PathSegment.Translation(0.0,0.0,176.0,0.0,120.0));
        path.addSegment(new PathSegment.Translation(176,0,210.73312629199899,21.46625258399798,176,38.83281572999746,120));
        path.addSegment(new PathSegment.Translation(210.73312629199899,21.46625258399798,245.52786404500043,91.05572809000084,60));
        path.addSegment(new PathSegment.Translation(245.52786404500043,91.05572809000084,246.4887655841161,109.36329177569044,225.01674906383332,101.31128558058441,100));
        path.addSegment(new PathSegment.Translation(246.4887655841161,109.36329177569044,197.02246883176784,241.27341644861912,100));
        path.addSegment(new PathSegment.Translation(197.02246883176784,241.27341644861912,178.9059960754954,243.3589941132431,187.43008082579254,237.67627094637837,100));
        path.addSegment(new PathSegment.Translation(178.9059960754954,243.3589941132431,50.0,50.0,100));
        
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(path);

        double dt = .01;

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        while (!controller.isFinished() && t < 100) {
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
        assertEquals(50, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance);
        assertEquals(50, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance);
    }
    
//
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
