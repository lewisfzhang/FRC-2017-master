package com.team254.lib.util;

import java.util.ArrayList;

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

    private CSVWriter mLogger;
    private double[] mErrorHistory = new double[50];
    private int mErrorHistoryIdx = 0;

    public JRadFlywheelController(double kj, double kLoadRatio) {
        mKj = kj;
        mKLoadRatio = kLoadRatio;
        mLogger = new CSVWriter(
                "/home/lvuser/SHOOTER-LOGS-JRAD.csv",
                new String[]{"time", "dt", "error", "newOutput", "meanSquareError"});
    }

    public double calculate(double input, double dt, double now) {
        double newOutput = mPrevOutput + mKj * dt * (mKLoadRatio * mSetpoint - input);
        mPrevOutput = newOutput;

        double error = input - mSetpoint;
        mErrorHistory[mErrorHistoryIdx] = error;
        mErrorHistoryIdx++;
        if (mErrorHistoryIdx >= mErrorHistory.length) {
            mErrorHistoryIdx = 0;
        }

        double meanSquareError = 0;
        for (double e : mErrorHistory) {
            meanSquareError += e * e / mErrorHistory.length;
        }

        mLogger.addValue(0, now);
        mLogger.addValue(1, dt);
        mLogger.addValue(2, error);
        mLogger.addValue(3, newOutput);
        mLogger.addValue(4, meanSquareError);
        mLogger.write();

        return newOutput;
    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    public void setKj(double kj) {
        mKj = kj;
    }

    public void setKLoadRatio(double kLoadRatio) {
        mKLoadRatio = kLoadRatio;
    }

    public void setOutputRange(double min, double max) {
        mMinOutput = min;
        mMaxOutput = max;
    }

    public void reset() {
        mPrevOutput = 0;
        mSetpoint = 0;
        mLogger.flush();
    }

    public double getSetpoint() {
        return mSetpoint;
    }

    public void setInitialOutput(double newOutput) {
        mPrevOutput = newOutput;
    }
}
