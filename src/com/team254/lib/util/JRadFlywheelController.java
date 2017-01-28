package com.team254.lib.util;

/**
 * Integral controller whose integral contribution is a function of the setpoint.
 * Basic math:
 * v[t + dt] = v[t] + kj * dt * (kLoadRatio * setpointSpeed - curSpeed)
 */
public class JRadFlywheelController {

    private double mKj;
    private double mKLoadRatio;

    private double mMaxOutput = 1.0;
    private double mMinOutput = -1.0;

    // controller state
    private double mPrevOutput = 0.0;

    private double mSetpoint = 0.0;

    public JRadFlywheelController(double kj, double kLoadRatio) {
        mKj = kj;
        mKLoadRatio = kLoadRatio;
    }

    double calculate(double input, double dt) {
        double newOutput = mPrevOutput + mKj * dt * (mKLoadRatio * mSetpoint - input);
        mPrevOutput = newOutput;
        return newOutput;
    }

    public void setSetpoint(double setpoint) {
        setpoint = mSetpoint;
    }

    public void setKj(double kj) {
        mKj = kj;
    }

    public void setKLoadRatio(double kLoadRatio) {
        mKLoadRatio = kLoadRatio;
    }

    public void reset() {
        mPrevOutput = 0;
        mSetpoint = 0;
    }
}
