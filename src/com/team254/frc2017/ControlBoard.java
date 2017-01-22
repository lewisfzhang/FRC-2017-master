package com.team254.frc2017;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A basic framework for the control board Like the drive code, one instance of the ControlBoard object is created upon
 * startup, then other methods request the singleton ControlBoard instance.
 */
public class ControlBoard {
    private static ControlBoard mInstance = new ControlBoard();

    public static ControlBoard getInstance() {
        return mInstance;
    }

    private final Joystick mStick;

    private ControlBoard() {
        mStick = new Joystick(0);
    }

    // DRIVER CONTROLS
    public double getThrottle() {
        return -mStick.getY();
    }

    public double getTurn() {
        return mStick.getX();
    }

    public boolean getQuickTurn() {
        return mStick.getRawButton(6);
    }

    public boolean getLowGear() {
        return mStick.getRawButton(5);
    }

    // OPERATOR CONTROLS
    public boolean getFireButton() {
        return mStick.getRawButton(4);
    }

    public boolean getIntakeButton() {
        return mStick.getRawButton(3);
    }

    public boolean getSpinShooterButton() {
        return mStick.getRawButton(1);
    }

    public boolean getShootButton() {
        return mStick.getRawButton(2);
    }
}
