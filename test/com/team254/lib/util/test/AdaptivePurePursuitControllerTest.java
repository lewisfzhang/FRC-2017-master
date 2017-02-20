package com.team254.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class AdaptivePurePursuitControllerTest {
    private static final double kEpsilon = 1E-6;
    private static final double kReallyBigNumber = 1E5;
    
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
}
