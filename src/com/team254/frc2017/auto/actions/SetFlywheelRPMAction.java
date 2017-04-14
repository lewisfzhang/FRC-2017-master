package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.Shooter;
import com.team254.frc2017.subsystems.Superstructure;

public class SetFlywheelRPMAction extends RunOnceAction {

    double rpm;

    public SetFlywheelRPMAction(double s) {
        rpm = s;
    }

    @Override
    public synchronized void runOnce() {
        Shooter.getInstance().setClosedLoopRpm(rpm);
    }
}
