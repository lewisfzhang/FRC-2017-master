package com.team254.lib.util.control;

import com.team254.lib.util.Util;

/**
 * A modified bang-bang controller:
 * In normal operation, this is an integral controller.
 * When input falls below a threshold % of setpoint:
 * - maintain output value and wait for threshold time
 * - save integral state
 * - then saturate output until input > setpoint
 * - the return to integral control, using saved integral state
 */
public class ShooterController {

    private double mIntegralState;
    private double mTimeBeforeSaturation;
    private boolean mInIntegralMode;

    private double mSetpoint;

    private double mkI;
    private double mKF;
    private double mkBangLowThresholdFraction;
    private double mkSaturationDelay;
    private double mkMaxOutput;
    private double mkMinOutput;
    private double mkSaturationOutput;

    public double calculate(double input, double dTime) {
        // Update my control mode
        if (mInIntegralMode) {
            if (input < mSetpoint * mkBangLowThresholdFraction) {
                mTimeBeforeSaturation -= dTime;
            } else {
                mTimeBeforeSaturation = mkSaturationDelay;
            }

            if (mTimeBeforeSaturation <= 0) {
                mInIntegralMode = false;
            }
        } else {
            if (input >= mSetpoint) {
                mInIntegralMode = true;
                mTimeBeforeSaturation = mkSaturationDelay;
            }
        }

        // Apply my control
        if (mInIntegralMode) {
            // integral control
            double error = mSetpoint - input;
            mIntegralState += dTime * error;
            return Util.limit(mkI * mIntegralState + mKF * mSetpoint, mkMinOutput, mkMaxOutput);
        } else {
            return mkSaturationOutput;
        }
    }

    public void setSetpoint(double s) {
        mSetpoint = s;
    }

    public void setConstants(
            double kI,
            double kF,
            double kBangLowThresholdFraction,
            double kSaturationDelay,
            double kMaxOutput,
            double kMinOutput,
            double kSaturationOutput) {
        mkI = kI;
        mKF = kF;
        mkBangLowThresholdFraction = kBangLowThresholdFraction ;
        mkSaturationDelay = kSaturationDelay;
        mkMaxOutput = kMaxOutput;
        mkMinOutput = kMinOutput;
        mkSaturationOutput = kSaturationOutput;
    }

    public void resetState() {
        mIntegralState = 0.0;
        // Assume that if we're resetting, We'll be starting from 0 speed when we come back
        mTimeBeforeSaturation = 0.0;
        mInIntegralMode = false;
    }
}
