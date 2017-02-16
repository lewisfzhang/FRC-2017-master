package com.team254.lib.util.motion;

public class MotionProfileConstraints {
    protected double max_abs_vel = Double.POSITIVE_INFINITY;
    protected double max_abs_acc = Double.POSITIVE_INFINITY;

    public MotionProfileConstraints(double max_vel, double max_acc) {
        this.max_abs_vel = Math.abs(max_vel);
        this.max_abs_acc = Math.abs(max_acc);
    }

    public double max_abs_vel() {
        return max_abs_vel;
    }

    public double max_abs_acc() {
        return max_abs_acc;
    }
}
