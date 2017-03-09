package com.team254.frc2017;

import edu.wpi.first.wpilibj.Joystick;

public class GamepadControlBoard extends ControlBoard {

    private final Joystick mGamepad;

    protected GamepadControlBoard() {
        mGamepad = new Joystick(0);
    }

    @Override
    public double getThrottle() {
        return -mGamepad.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return mGamepad.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn() {
        return mGamepad.getRawAxis(2) > 0.1;
    }

    @Override
    public boolean getAimButton() {
        return mGamepad.getRawAxis(3) > 0.1;
    }

    @Override
    public boolean getLowGear() {
        return mGamepad.getRawButton(6);
    }

    @Override
    public boolean getHangButton() {
        return mGamepad.getRawButton(1);
    }

    @Override
    public boolean getIntakeButton() {
       return mGamepad.getRawButton(5);
    }
}
