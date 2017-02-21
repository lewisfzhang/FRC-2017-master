package com.team254.frc2017.auto.modes;

import com.team254.frc2017.Constants;
import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.CenterToHopperBlue;
import com.team254.frc2017.paths.GearToCenterBlue;
import com.team254.frc2017.paths.GearToHopperBlue;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToGear;
import com.team254.frc2017.paths.StartToGearBlue;
import com.team254.frc2017.paths.StartToGearRed;

public class GearThenHopperShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToGearBlue();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        runAction(new WaitAction(0.5));
        Constants.kAutoLookAhead = 24.0;
        runAction(new DrivePathAction(new GearToHopperBlue()));
        //runAction(new DrivePathAction(new GearToCenterBlue()));
        //runAction(new DrivePathAction(new CenterToHopperBlue()));
    }
}
