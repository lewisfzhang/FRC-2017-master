package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.GearGrabber;
import edu.wpi.first.wpilibj.Timer;

public class ScoreGearAction implements Action {

    private GearGrabber mGearGrabber = GearGrabber.getInstance();
    private double startTime = Timer.getFPGATimestamp();

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 1.5; //TODO: implement function in GearGrabber class to check when it is done switching states
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mGearGrabber.setWantedState(GearGrabber.WantedState.SCORE_GEAR);
    }
}
