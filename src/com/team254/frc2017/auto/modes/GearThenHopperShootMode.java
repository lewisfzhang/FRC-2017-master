package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToGear;

public class GearThenHopperShootMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToGear();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
    }
}
