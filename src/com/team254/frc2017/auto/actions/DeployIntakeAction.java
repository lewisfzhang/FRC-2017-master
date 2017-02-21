package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.Intake;

public class DeployIntakeAction extends RunOnceAction implements Action {

    @Override
    public void runOnce() {
        Intake.getInstance().deploy();
    }
}
