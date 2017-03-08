package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.MotorGearGrabber;

import edu.wpi.first.wpilibj.Timer;

public class ScoreGearAction implements Action {

    private double startTime;
    private MotorGearGrabber mGearGrabber; 

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 0.5;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mGearGrabber.setOpenLoop(0);
    }

    @Override
    public void start() {
        mGearGrabber = MotorGearGrabber.getInstance();
        startTime = Timer.getFPGATimestamp();
        mGearGrabber.setOpenLoop(6);
        mGearGrabber.setWristDown();
    }
}
