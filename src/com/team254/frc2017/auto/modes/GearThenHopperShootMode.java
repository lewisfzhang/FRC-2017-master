package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.paths.StartToGear;

public class GearThenHopperShootMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DrivePathAction(new StartToGear()));
    }
}
