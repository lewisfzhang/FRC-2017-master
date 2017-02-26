package com.team254.frc2017;

import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Twist2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */

public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double linear_velocity = (left_wheel_delta + right_wheel_delta) / 2;
        double delta_v = (right_wheel_delta - left_wheel_delta) / 2;
        double delta_rotation = delta_v * 2 * Constants.kTrackScrubFactor / Constants.kTrackWidthInches;
        return new Twist2d(linear_velocity, 0, delta_rotation);
    }

    /**
     * Forward kinematics using encoders and explicitly measured rotation (ex. from gyro)
     */
    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads) {
        return new Twist2d((left_wheel_delta + right_wheel_delta) / 2, 0, delta_rotation_rads);
    }

    /**
     * For convenience, forward kinematic with an absolute rotation and previous rotation.
     */
    public static Twist2d forwardKinematics(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
            Rotation2d current_heading) {
        return new Twist2d((left_wheel_delta + right_wheel_delta) / 2, 0,
                prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /** Append the result of forward kinematics to a previous pose. */
    public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose, double left_wheel_delta,
            double right_wheel_delta, Rotation2d current_heading) {
        Twist2d with_gyro = forwardKinematics(current_pose.getRotation(), left_wheel_delta, right_wheel_delta,
                current_heading);
        return integrateForwardKinematics(current_pose, with_gyro);
    }

    public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose,
            Twist2d forward_kinematics) {
        return current_pose.transformBy(RigidTransform2d.exp(forward_kinematics));
    }

    public static class DriveVelocity {
        public final double left;
        public final double right;

        public DriveVelocity(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static DriveVelocity inverseKinematics(Twist2d velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveVelocity(velocity.dx, velocity.dx);
        }
        double delta_v = Constants.kTrackWidthInches * velocity.dtheta / (2 * Constants.kTrackScrubFactor);
        return new DriveVelocity(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}
