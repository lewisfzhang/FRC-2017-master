package com.team254.lib.util;

import com.team254.lib.util.motion.MotionProfileConstraints;
import com.team254.lib.util.motion.MotionProfileGoal;
import com.team254.lib.util.motion.MotionProfileGoal.CompletionBehavior;
import com.team254.lib.util.motion.MotionState;
import com.team254.lib.util.motion.ProfileFollower;

/**
 * A PathFollower follows a predefined path using a combination of feedforward and feedback control. It uses an
 * AdaptivePurePursuitController to choose a reference pose and generate a steering command (curvature), and then a
 * ProfileFollower to generate a profile (displacement and velocity) command.
 */
public class PathFollower {
    public static class Parameters {
        public final double fixed_lookahead;
        public final double profile_kp;
        public final double profile_ki;
        public final double profile_kv;
        public final double profile_kffv;
        public final double profile_kffa;
        public final double profile_max_abs_vel;
        public final double profile_max_abs_acc;
        public final double dt;

        public Parameters(double fixed_lookahead, double profile_kp, double profile_ki, double profile_kv,
                double profile_kffv, double profile_kffa, double profile_max_abs_vel, double profile_max_abs_acc,
                double dt) {
            this.fixed_lookahead = fixed_lookahead;
            this.profile_kp = profile_kp;
            this.profile_ki = profile_ki;
            this.profile_kv = profile_kv;
            this.profile_kffv = profile_kffv;
            this.profile_kffa = profile_kffa;
            this.profile_max_abs_vel = profile_max_abs_vel;
            this.profile_max_abs_acc = profile_max_abs_acc;
            this.dt = dt;
        }
    }

    AdaptivePurePursuitController mSteeringController;
    RigidTransform2d.Delta mLastSteeringDelta;
    ProfileFollower mVelocityController;
    final double mDt;

    double mCrossTrackError = 0.0;
    double mAlongTrackError = 0.0;

    /**
     * Create a new PathFollower for a given path.
     */
    public PathFollower(Path path, boolean reversed, Parameters parameters) {
        mSteeringController = new AdaptivePurePursuitController(path, reversed, parameters.fixed_lookahead);
        mLastSteeringDelta = RigidTransform2d.Delta.identity();
        mVelocityController = new ProfileFollower(parameters.profile_kp, parameters.profile_ki, parameters.profile_kv,
                parameters.profile_kffv, parameters.profile_kffa);
        mVelocityController.setConstraints(
                new MotionProfileConstraints(parameters.profile_max_abs_vel, parameters.profile_max_abs_acc));
        mDt = parameters.dt;
    }

    /**
     * Get new velocity commands to follow the path.
     * 
     * @param t
     *            The current timestamp
     * @param pose
     *            The current robot pose
     * @param displacement
     *            The current robot displacement (total distance driven).
     * @param velocity
     *            The current robot velocity.
     * @return The velocity command to apply until time t + dt.
     */
    public RigidTransform2d.Delta update(double t, RigidTransform2d pose, double displacement, double velocity) {
        if (!mSteeringController.isFinished()) {
            final AdaptivePurePursuitController.Command steering_command = mSteeringController.update(pose);
            mCrossTrackError = steering_command.cross_track_error;
            if (!mSteeringController.isFinished()) {
                mLastSteeringDelta = steering_command.delta;
                mVelocityController.setGoal(new MotionProfileGoal(displacement + steering_command.delta.dx,
                        Math.abs(steering_command.end_velocity), CompletionBehavior.VIOLATE_MAX_ACCEL));
            }
        }
       
        final double velocity_command = mVelocityController.update(new MotionState(t, displacement, velocity, 0.0),
                t + mDt);
        mAlongTrackError = mVelocityController.getPosError();
        final double scale = velocity_command / mLastSteeringDelta.dx;
        return mLastSteeringDelta.scaled(scale);
    }

    public double getCrossTrackError() {
        return mCrossTrackError;
    }

    public double getAlongTrackError() {
        return mAlongTrackError;
    }

    public boolean isFinished() {
        return mSteeringController.isFinished() && mVelocityController.isFinishedProfile();
    }
}
