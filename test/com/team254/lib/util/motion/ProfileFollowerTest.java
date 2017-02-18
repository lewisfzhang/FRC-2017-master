package com.team254.lib.util.motion;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.lib.util.motion.MotionProfileGoal.CompletionBehavior;

public class ProfileFollowerTest {

    protected abstract class Dynamics {
        protected MotionState mState;

        public Dynamics(MotionState state) {
            mState = state;
        }

        public MotionState getState() {
            return mState;
        }

        public abstract void update(double command_vel, double dt);
    }

    protected class IdealDynamics extends Dynamics {
        public IdealDynamics(MotionState state) {
            super(state);
        }

        @Override
        public void update(double command_vel, double dt) {
            final double acc = (command_vel - mState.vel()) / dt;
            mState = mState.extrapolate(mState.t() + dt, acc);
        }
    }

    protected class ScaledDynamics extends Dynamics {
        protected double mVelRatio;

        public ScaledDynamics(MotionState state, double vel_ratio) {
            super(state);
            mVelRatio = vel_ratio;
        }

        @Override
        public void update(double command_vel, double dt) {
            final double acc = (command_vel * mVelRatio - mState.vel()) / dt;
            mState = mState.extrapolate(mState.t() + dt, acc);
        }
    }

    protected class DeadbandDynamics extends Dynamics {
        protected double mDeadband;

        public DeadbandDynamics(MotionState state, double deadband) {
            super(state);
            mDeadband = deadband;
        }

        @Override
        public void update(double command_vel, double dt) {
            if (command_vel > -mDeadband && command_vel < mDeadband) {
                command_vel = 0.0;
            } else {
                command_vel = Math.signum(command_vel) * (Math.abs(command_vel) - mDeadband);
            }
            final double acc = (command_vel - mState.vel()) / dt;
            mState = mState.extrapolate(mState.t() + dt, acc);
        }
    }

    protected MotionState followProfile(ProfileFollower follower, Dynamics dynamics, double dt, int max_iterations) {
        int i = 0;
        for (; i < max_iterations && !follower.onTarget(); ++i) {
            MotionState state = dynamics.getState();
            final double t = state.t() + dt;
            final double command_vel = follower.update(state, t);
            dynamics.update(command_vel, dt);
            System.out.println("State: " + state + ", Pos error: " + follower.getPosError() + ", Vel error: "
                    + follower.getVelError() + ", Command: " + command_vel);
            if (follower.isFinishedProfile()) {
                System.out.println("Follower has finished profile");
            }
        }
        if (i == max_iterations) {
            System.out.println("Iteration limit reached");
        }
        System.out.println("Final state: " + dynamics.getState());
        return dynamics.getState();
    }

    @Test
    public void testStationaryToStationaryFeedforward() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.0, 0.0, 0.0, 1.0, 0.0);
        follower.setGoalAndConstraints(goal, constraints);
        MotionState final_state = followProfile(follower, new IdealDynamics(start_state), dt, 1500);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToStationaryUpdateGoal() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.0, 0.0, 0.0, 1.0, 0.0);
        follower.setGoalAndConstraints(goal, constraints);
        Dynamics dynamics = new IdealDynamics(start_state);
        MotionState final_state = followProfile(follower, dynamics, dt, 500);
        assertFalse(goal.atGoalState(final_state));

        goal = new MotionProfileGoal(0.0);
        follower.setGoalAndConstraints(goal, constraints);
        final_state = followProfile(follower, dynamics, dt, 1500);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToStationaryResetSetpoint() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.0, 0.0, 0.0, 1.0, 0.0);
        follower.setGoalAndConstraints(goal, constraints);
        Dynamics dynamics = new IdealDynamics(start_state);
        MotionState final_state = followProfile(follower, dynamics, dt, 500);
        assertFalse(goal.atGoalState(final_state));

        follower.resetSetpoint();
        final_state = followProfile(follower, dynamics, dt, 1500);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToStationaryResetProfile() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.0, 0.0, 0.0, 1.0, 0.0);
        follower.setGoalAndConstraints(goal, constraints);
        Dynamics dynamics = new IdealDynamics(start_state);
        MotionState final_state = followProfile(follower, dynamics, dt, 500);
        assertFalse(goal.atGoalState(final_state));

        follower.resetProfile();
        follower.setGoalAndConstraints(goal, constraints);
        final_state = followProfile(follower, dynamics, dt, 1500);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToStationaryFeedback() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0, 0.0, CompletionBehavior.OVERSHOOT, 1.0, 1.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.5, 0.001, 0.5, 1.0, 0.1);
        follower.setGoalAndConstraints(goal, constraints);
        MotionState final_state = followProfile(follower, new ScaledDynamics(start_state, .8), dt, 2000);
        assertTrue(goal.atGoalState(final_state));
    }
    
    @Test
    public void testStationaryToStationaryFeedbackFast() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0, 0.0, CompletionBehavior.OVERSHOOT, 1.0, 1.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.5, 0.001, 0.5, 1.0, 0.1);
        follower.setGoalAndConstraints(goal, constraints);
        MotionState final_state = followProfile(follower, new ScaledDynamics(start_state, 1.2), dt, 2000);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToStationaryFeedbackDeadband() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(100.0, 0.0, CompletionBehavior.OVERSHOOT, 1.0, 1.0);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.5, 0.001, 0.5, 1.0, 0.1);
        follower.setGoalAndConstraints(goal, constraints);
        MotionState final_state = followProfile(follower, new DeadbandDynamics(start_state, 2.0), dt, 2000);
        assertTrue(goal.atGoalState(final_state));
    }

    @Test
    public void testStationaryToMovingOvershoot() {
        MotionProfileConstraints constraints = new MotionProfileConstraints(10.0, 10.0);
        MotionProfileGoal goal = new MotionProfileGoal(-100.0, 10.0, CompletionBehavior.VIOLATE_MAX_ACCEL, 1.0, 0.1);
        MotionState start_state = new MotionState(0.0, 0.0, 0.0, 0.0);
        final double dt = 0.01;

        ProfileFollower follower = new ProfileFollower(0.0, 0.0, 0.0, 1.0, 0.0);
        follower.setGoalAndConstraints(goal, constraints);
        MotionState final_state = followProfile(follower, new ScaledDynamics(start_state, 1.2), dt, 2000);
        assertTrue(goal.atGoalPos(final_state.pos()));
    }
}
