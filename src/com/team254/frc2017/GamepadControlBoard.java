package com.team254.frc2017;

import edu.wpi.first.wpilibj.Joystick;

public class GamepadControlBoard implements ControlBoardInterface {

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
        // R1
        return mGamepad.getRawButton(6);

    }

    @Override
    public boolean getHangButton() {

        // A
        return mGamepad.getRawButton(1);
    }

    @Override
    public boolean getIntakeButton() {
        // L1
        return mGamepad.getRawButton(5);
    }

    @Override
    public boolean getFeedButton() {
        // X
        return mGamepad.getRawButton(3);
    }

    @Override
    public boolean getGrabGearButton() {
        return false;
    }

    @Override
    public boolean getScoreGearButton() {
        return false;
    }


    @Override
    public boolean getShooterOpenLoopButton() {
        // Y
        return mGamepad.getRawButton(4);
    }

    @Override
    public boolean getExhaustButton() {
        // B
        return mGamepad.getRawButton(2);
    }

    @Override
    public boolean getUnjamButton() {
        return false;
    }

    @Override
    public boolean getShooterClosedLoopButton() {
        // Back
        return mGamepad.getRawButton(7);
    }

    @Override
    public boolean getFlywheelSwitch() {
        return false;
    }
}
