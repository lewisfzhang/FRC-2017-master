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

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final Joystick mButtonBoard;

    private ControlBoard() {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(1);
        mButtonBoard = new Joystick(2);
    }

    // DRIVER CONTROLS
    public double getThrottle() {
        return -mThrottleStick.getY();
    }

    public double getTurn() {
        return mTurnStick.getX();
    }

    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    public boolean getLowGear() {
        return mThrottleStick.getRawButton(2);
    }

    // OPERATOR CONTROLS
    public boolean getFireButton() {
        return mButtonBoard.getRawAxis(1) < -0.1;
    }
    
    public boolean getIntakeButton() {
        return mButtonBoard.getRawAxis(2) < -0.1;
    }
}
