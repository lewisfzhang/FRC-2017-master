package com.team254.lib.util.control;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import com.team254.lib.util.control.ShooterController;

/**
 * Created by leighpauls on 2/15/17.
 */
public class ShooterControllerTest {

    ShooterController mShooterController;

    private double mkI = 0.1;
    private double mKF = 7.0 / 3000.0;
    private double mkBangLowThresholdFraction = 0.5;
    private double mkSaturationDelay = 0.1;
    private double mkMaxOutput = 10.0;
    private double mkMinOutput = -10.0;
    private double mkSaturationOutput = 9.0;

    private static final double DTIME = 0.005;
    private static final double EPS = 0.00001;

    @Before
    public void setup() {
        mShooterController = new ShooterController();
        mShooterController.setConstants(
                mkI,
                mKF,
                mkBangLowThresholdFraction,
                mkSaturationDelay,
                mkMaxOutput,
                mkMinOutput,
                mkSaturationOutput);
    }

    @Test
    public void testZeroSetpoint() {
        mShooterController.setSetpoint(0);
        assertEq(0, mShooterController.calculate(0, DTIME));
    }


    @Test
    public void testStartsInSaturation() {
        mShooterController.setSetpoint(3000);
        assertEq(mkSaturationOutput, mShooterController.calculate(0, DTIME));
    }

    @Test
    public void testEntersFeedForward() {
        double setpoint = 3000.0;
        mShooterController.setSetpoint(setpoint);
        assertEq(mKF * setpoint, mShooterController.calculate(setpoint, DTIME));
    }

    @Test
    public void testIntegratesError() {
        double setpoint = 3000.0;
        double error = 10;
        mShooterController.setSetpoint(setpoint);
        assertEq(
                mKF * setpoint - mkI * error * DTIME,
                mShooterController.calculate(setpoint + error, DTIME));
        assertEq(
                mKF * setpoint - 2 * mkI * error * DTIME,
                mShooterController.calculate(setpoint + error, DTIME));
    }

    @Test
    public void testModeTransitions() {
        double setpoint = 3000.0;
        mShooterController.setSetpoint(setpoint);

        double error = 100;
        // initially in saturation mode
        assertEq(mkSaturationOutput, mShooterController.calculate(setpoint - error, DTIME));

        // move to integration
        double expectedOuput = mKF * setpoint - mkI * error * DTIME;
        assertEq(expectedOuput, mShooterController.calculate(setpoint + error, DTIME));
        expectedOuput -= mkI * error * DTIME;
        assertEq(expectedOuput, mShooterController.calculate(setpoint + error, DTIME));
        expectedOuput += mkI * error * DTIME;
        assertEq(expectedOuput, mShooterController.calculate(setpoint - error, DTIME));

        // stay in integration before the threshold time
        double bigError = setpoint * 0.75;
        expectedOuput += mkI * bigError * DTIME;
        assertEq(expectedOuput, mShooterController.calculate(setpoint - bigError, DTIME));

        // exit integration back into saturation after threshold
        assertEq(mkSaturationOutput, mShooterController.calculate(setpoint - bigError, mkSaturationDelay));

        // return to integral, integration state should have been maintained
        assertEq(expectedOuput, mShooterController.calculate(setpoint, DTIME));
    }

    private static void assertEq(double expected, double actual) {
        Assert.assertEquals(expected, actual, EPS);
    }
}
