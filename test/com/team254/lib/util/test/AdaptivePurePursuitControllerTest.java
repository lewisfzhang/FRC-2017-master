package com.team254.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.*;
import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Path;
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
    
    @Test
    public void testAuto() {
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(StartToGear.buildPath(), StartToGear.isReversed());

        double dt = .01;

        RigidTransform2d robot_pose = StartToGear.getStartPose();
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
        assertEquals(109, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance*2);
        assertEquals(107, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance*2);
       
        controller = new AdaptivePurePursuitController(GearToHopper.buildPath(), GearToHopper.isReversed());
        while (!controller.isFinished() && t < 500) {
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
        assertEquals(132, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance*2);
        assertEquals(20, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance*2);
    }
}