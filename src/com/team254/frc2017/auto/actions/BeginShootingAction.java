package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.Drive;
import com.team254.frc2017.subsystems.Superstructure;

public class BeginShootingAction extends RunOnceAction implements Action {

    @Override
    public void runOnce() {
        Drive.getInstance().setWantAimToGoal();
        Superstructure.getInstance().setWantedState(Superstructure.WantedState.SHOOT);
    }

}
