package com.team254.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.GearToHopper;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToGear;
import com.team254.frc2017.paths.TestArcPath;
import com.team254.lib.util.Path;
import com.team254.lib.util.PathFollower;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class PathFollowerTest {

    static final PathFollower.Parameters kParameters = new PathFollower.Parameters(25.0, // Fixed lookahead
            0.0, // Profile kp
            0.0, // Profile ki
            0.0, // Profile kv
            1.0, // Profile kffv
            0.0, // Profile kffa
            Constants.kPathFollowingMaxVel, // Profile max abs vel
            Constants.kPathFollowingMaxAccel, // Profile max abs accel
            0.01 // Profile dt
    );

    @Test
    public void testArcPath() {
        Path path = TestArcPath.buildPath();
        PathFollower controller = new PathFollower(path, false, kParameters);

        final double dt = kParameters.dt;

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 10.0) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.fromVelocity(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertEquals(200, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance);
        assertEquals(100, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance);
    }

    @Test
    public void testAuto() {
        PathContainer container = new StartToGear();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        final double dt = kParameters.dt;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.fromVelocity(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertEquals(109, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance * 3);
        assertEquals(107, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance * 3);

        container = new GearToHopper();
        controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);
        while (!controller.isFinished() && t < 50.0) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.fromVelocity(command.scaled(dt)));

            t += dt;
            final double prev_vel = velocity;
            velocity = command.dx;
            displacement += velocity * dt;

            System.out.println("t = " + t + ", displacement " + displacement + ", lin vel " + command.dx + ", lin acc "
                    + (velocity - prev_vel) / dt + ", ang vel " + command.dtheta + ", pose " + robot_pose + ", CTE "
                    + controller.getCrossTrackError() + ", ATE " + controller.getAlongTrackError());
        }
        System.out.println(robot_pose);
        assertTrue(controller.isFinished());
        assertEquals(132, robot_pose.getTranslation().getX(), Constants.kSegmentCompletionTolerance * 3);
        assertEquals(20, robot_pose.getTranslation().getY(), Constants.kSegmentCompletionTolerance * 3);
    }
}