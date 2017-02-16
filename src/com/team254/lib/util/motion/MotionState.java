package com.team254.lib.util.motion;

import static com.team254.lib.util.Util.*;
import static com.team254.lib.util.motion.MotionUtil.*;

public class MotionState {
    protected final double t;
    protected final double pos;
    protected final double vel;
    protected final double acc;

    public static MotionState kInvalidState = new MotionState(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public MotionState(double t, double pos, double vel, double acc) {
        this.t = t;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }

    public MotionState(MotionState state) {
        this(state.t, state.pos, state.vel, state.acc);
    }

    public double t() {
        return t;
    }

    public double pos() {
        return pos;
    }

    public double vel() {
        return vel;
    }

    public double vel2() {
        return vel * vel;
    }

    public double acc() {
        return acc;
    }

    public MotionState extrapolate(double t) {
        return extrapolate(t, acc);
    }

    public MotionState extrapolate(double t, double acc) {
        final double dt = t - this.t;
        return new MotionState(t, pos + vel * dt + .5 * acc * dt * dt, vel + acc * dt, acc);
    }

    public double nextTimeAtPos(double pos) {
        if (epsilonEquals(pos, this.pos, kEpsilon)) {
            // Already at pos.
            return t;
        }
        if (epsilonEquals(acc, 0.0, kEpsilon)) {
            // Zero acceleration case.
            final double delta_pos = pos - this.pos;
            if (!epsilonEquals(vel, 0.0, kEpsilon) && Math.signum(delta_pos) == Math.signum(vel)) {
                // Constant velocity heading towards pos.
                return delta_pos / vel + t;
            }
            return Double.NaN;
        }

        // Solve the quadratic formula.
        // ax^2 + bx + c == 0
        // x = dt
        // a = .5 * acc
        // b = vel
        // c = this.pos - pos
        final double disc = vel * vel - 2.0 * acc * (this.pos - pos);
        if (disc < 0.0) {
            // Extrapolating this MotionState never reaches the desired pos.
            return Double.NaN;
        }
        final double sqrt_disc = Math.sqrt(disc);
        final double max_dt = (-vel + sqrt_disc) / acc;
        final double min_dt = (-vel - sqrt_disc) / acc;
        if (min_dt >= 0.0) {
            return t + min_dt;
        }
        if (max_dt >= 0.0) {
            return t + max_dt;
        }
        //
        return Double.NaN;
    }

    @Override
    public String toString() {
        return "(t=" + t + ", pos=" + pos + ", vel=" + vel + ", acc=" + acc + ")";
    }

    @Override
    public boolean equals(Object other) {
        return (other instanceof MotionState) && equals((MotionState) other, kEpsilon);
    }

    public boolean equals(MotionState other, double epsilon) {
        return coincident(other, epsilon) && epsilonEquals(acc, other.acc, epsilon);
    }

    public boolean coincident(MotionState other) {
        return coincident(other, kEpsilon);
    }

    public boolean coincident(MotionState other, double epsilon) {
        return epsilonEquals(t, other.t, epsilon) && epsilonEquals(pos, other.pos, epsilon)
                && epsilonEquals(vel, other.vel, epsilon);
    }

    public MotionState flipped() {
        return new MotionState(t, -pos, -vel, -acc);
    }
}
