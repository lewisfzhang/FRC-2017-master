package com.team254.lib.util.motion;

import static com.team254.lib.util.Util.*;

/**
 * A controller for tracking a profile generated to attain a MotionProfileGoal. The controller uses feedforward on
 * acceleration, velocity, and position; proportional feedback on velocity and position; and integral feedback on
 * position.
 */
public class ProfileFollower {
    protected double mKp;
    protected double mKi;
    protected double mKv;
    protected double mKffv;
    protected double mKffa;

    protected double mMinOutput = Double.NEGATIVE_INFINITY;
    protected double mMaxOutput = Double.POSITIVE_INFINITY;
    protected double mLatestPosError;
    protected double mLatestVelError;
    protected double mTotalError;

    protected MotionProfileGoal mGoal = null;
    protected MotionProfileConstraints mConstraints = null;
    protected SetpointGenerator mSetpointGenerator = new SetpointGenerator();
    protected SetpointGenerator.Setpoint mLatestSetpoint = null;

    /**
     * Create a new ProfileFollower.
     * 
     * @param kp
     *            The proportional gain on position error.
     * @param ki
     *            The integral gain on position error.
     * @param kv
     *            The proportional gain on velocity error (or derivative gain on position error).
     * @param kffv
     *            The feedforward gain on velocity. Should be 1.0 if the units of the profile match the units of the
     *            output.
     * @param kffa
     *            The feedforward gain on acceleration.
     */
    public ProfileFollower(double kp, double ki, double kv, double kffv, double kffa) {
        resetProfile();
        setGains(kp, ki, kv, kffv, kffa);
    }

    public void setGains(double kp, double ki, double kv, double kffv, double kffa) {
        mKp = kp;
        mKi = ki;
        mKv = kv;
        mKffv = kffv;
        mKffa = kffa;
    }

    /**
     * Completely clear all state related to the current profile (min and max outputs are maintained).
     */
    public void resetProfile() {
        mTotalError = 0.0;
        mLatestPosError = 0.0;
        mLatestVelError = 0.0;
        mSetpointGenerator.reset();
        mGoal = null;
        mConstraints = null;
        resetSetpoint();
    }

    /**
     * Specify a goal (and constraints for achieving the goal).
     * 
     * @param goal
     * @param constraints
     */
    public void setGoal(MotionProfileGoal goal, MotionProfileConstraints constraints) {
        if (mGoal != null && !mGoal.equals(goal) && mLatestSetpoint != null) {
            // Clear the final state bit since the goal has changed.
            mLatestSetpoint.final_setpoint = false;
        }
        mGoal = goal;
        mConstraints = constraints;
    }

    /**
     * Reset just the setpoint. This means that the latest_state provided to update() will be used rather than feeding
     * forward the previous setpoint the next time update() is called. This almost always forces a MotionProfile update,
     * and may be warranted if tracking error gets very large.
     */
    public void resetSetpoint() {
        mLatestSetpoint = null;
    }

    /**
     * Update the setpoint and apply the control gains to generate a control output.
     * 
     * @param latest_state
     *            The latest *actual* state, used only for feedback purposes (unless this is the first iteration or
     *            reset()/resetSetpoint() was just called, in which case this is the new start state for the profile).
     * @param t
     *            The timestamp for which the setpoint is desired.
     * @return An output that reflects the control output to apply to achieve the new setpoint.
     */
    public double update(MotionState latest_state, double t) {
        MotionState prev_state = latest_state;
        if (mLatestSetpoint != null) {
            prev_state = mLatestSetpoint.motion_state;
        }
        final double dt = Math.max(0.0, t - prev_state.t());
        mLatestSetpoint = mSetpointGenerator.getSetpoint(mConstraints, mGoal, prev_state, dt);

        // Update error.
        mLatestPosError = mLatestSetpoint.motion_state.pos() - latest_state.pos();
        mLatestVelError = mLatestSetpoint.motion_state.vel() - latest_state.vel();

        // Calculate the feedforward and proportional terms.
        double output = mKp * mLatestPosError + mKv * mLatestVelError + mKffv * mLatestSetpoint.motion_state.vel()
                + (Double.isNaN(mLatestSetpoint.motion_state.acc()) ? 0.0 : mKffa * mLatestSetpoint.motion_state.acc());
        if (output >= mMinOutput && output <= mMaxOutput) {
            // Update integral.
            mTotalError += mLatestPosError * dt;
            output += mKi * mTotalError;
        } else {
            // Reset integral windup.
            mTotalError = 0.0;
        }
        // Clamp to limits.
        output = Math.max(mMinOutput, Math.min(mMaxOutput, output));
        return output;
    }

    public void setMinOutput(double min_output) {
        mMinOutput = min_output;
    }

    public void setMaxOutput(double max_output) {
        mMaxOutput = max_output;
    }

    public double getPosError() {
        return mLatestPosError;
    }

    public double getVelError() {
        return mLatestVelError;
    }

    /**
     * We are finished the profile when the final setpoint has been generated. Note that this does not check whether we
     * are anywhere close to the final setpoint, however.
     * 
     * @return True if the final setpoint has been generated for the current goal.
     */
    public boolean isFinishedProfile() {
        return mGoal != null && mLatestSetpoint != null && mLatestSetpoint.final_setpoint;
    }

    /**
     * We are on target if the final setpoint has been generated and we have achieved the goal (where the definition of
     * achievement depends on the goal's completion behavior).
     * 
     * @return True if we have actually achieved the current goal.
     */
    public boolean onTarget() {
        if (!isFinishedProfile()) {
            return false;
        }
        switch (mGoal.completion_behavior()) {
        case VIOLATE_MAX_ABS_VEL:
        case VIOLATE_MAX_ACCEL:
            // Check that pos is at or has passed the goal.
            return epsilonEquals(getPosError(), 0.0, mGoal.pos_tolerance())
                    || Math.signum(getPosError()) * Math.signum(mLatestSetpoint.motion_state.vel()) >= 0.0;
        case OVERSHOOT:
        default:
            // Check that we are at both the desired position and velocity.
            return epsilonEquals(getPosError(), 0.0, mGoal.pos_tolerance())
                    && epsilonEquals(getVelError(), 0.0, mGoal.vel_tolerance());
        }
    }
}
