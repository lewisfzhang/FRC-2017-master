package com.team254.lib.util.control;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.GearToHopperBlue;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToBoilerGearBlue;
import com.team254.frc2017.paths.StartToBoilerGearRed;
import com.team254.lib.util.control.PathFollower;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Twist2d;

public class PathFollowerTest {

    static final PathFollower.Parameters kParameters = new PathFollower.Parameters(
            new Lookahead(16.0, 16.0, 0.0, 120.0),
            0.0, // Inertia gain
            0.0, // Profile kp
            0.0, // Profile ki
            0.0, // Profile kv
            1.0, // Profile kffv
            0.0, // Profile kffa
            Constants.kPathFollowingMaxVel, // Profile max abs vel
            Constants.kPathFollowingMaxAccel  // Profile max abs accel
    );

    @Test
    public void testArcPath() {
        PathContainer container = new StartToBoilerGearRed();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);


        final double dt = 0.01;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 10.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

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
        assertEquals(110, robot_pose.getTranslation().x(), Constants.kSegmentCompletionTolerance );
        assertEquals(109, robot_pose.getTranslation().y(), Constants.kSegmentCompletionTolerance*3);
    }

    @Test
    public void testAuto() {
        PathContainer container = new StartToBoilerGearBlue();
        PathFollower controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);

        final double dt = 0.01;;

        RigidTransform2d robot_pose = container.getStartPose();
        double t = 0;
        double displacement = 0.0;
        double velocity = 0.0;
        while (!controller.isFinished() && t < 25.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

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
        assertEquals(110, robot_pose.getTranslation().x(), Constants.kSegmentCompletionTolerance * 3);
        assertEquals(214, robot_pose.getTranslation().y(), Constants.kSegmentCompletionTolerance * 3);

        container = new GearToHopperBlue();
        controller = new PathFollower(container.buildPath(), container.isReversed(), kParameters);
        while (!controller.isFinished() && t < 50.0) {
            // Follow the path
            Twist2d command = controller.update(t, robot_pose, displacement, velocity);
            robot_pose = robot_pose.transformBy(RigidTransform2d.exp(command.scaled(dt)));

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
        assertEquals(91, robot_pose.getTranslation().x(), Constants.kSegmentCompletionTolerance * 5);
        assertEquals(301, robot_pose.getTranslation().y(), Constants.kSegmentCompletionTolerance * 5);
    }
}