package com.team254.frc2017;

/**
 * Created by leighpauls on 3/11/17.
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getLowGear();

    boolean getAimButton();

    // OPERATOR CONTROLS
    boolean getFeedButton();

    boolean getIntakeButton();

    boolean getShooterOpenLoopButton();

    boolean getExhaustButton();

    boolean getUnjamButton();

    boolean getShooterClosedLoopButton();

    boolean getFlywheelSwitch();

    boolean getHangButton();

    boolean getGrabGearButton();

    boolean getScoreGearButton();
    
    boolean getActuateHopperButton();

    boolean getBlinkLEDButton();
}
