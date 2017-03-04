package com.team254.frc2017.auto.actions;

import com.team254.frc2017.subsystems.Superstructure;

public class EndShootingAction extends RunOnceAction implements Action {

    @Override
    public void runOnce() {
        Superstructure.getInstance().setWantedState(Superstructure.WantedState.IDLE);
    }

}
