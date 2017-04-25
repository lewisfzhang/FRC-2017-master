package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.MotorGearGrabber.WantedState;
import com.team254.frc2017.subsystems.MotorGearGrabber;

import edu.wpi.first.wpilibj.Timer;

public class DeployCowCatcherAction implements Action {

    MotorGearGrabber mGearGrabber = MotorGearGrabber.getInstance();
    double startTime;
    
    static final double kDelay = 0.1;

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > 2*kDelay;
    }

    @Override
    public void update() {
        if(Timer.getFPGATimestamp() - startTime > kDelay) {
            mGearGrabber.setWantedState(WantedState.ACQUIRE);
        }
    }

    @Override
    public void done() {
        mGearGrabber.setWantedState(WantedState.IDLE);
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        mGearGrabber.setWantedState(WantedState.IDLE);
    }
}
