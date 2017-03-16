package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.MotorGearGrabber;
import com.team254.frc2017.subsystems.MotorGearGrabber.WantedState;

public class LowerGearAction extends RunOnceAction {

    @Override
    public void runOnce() {
        MotorGearGrabber.getInstance().setWantedState(WantedState.FORCE_LOWERED);
    }
}
