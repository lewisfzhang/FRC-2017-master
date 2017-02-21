package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.TurnToHeadingAction;
import com.team254.lib.util.Rotation2d;

public class TestTurnInPlaceMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
    }
}
