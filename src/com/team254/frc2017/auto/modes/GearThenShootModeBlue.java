package com.team254.frc2017.auto.modes;


import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.EndShootingAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.ScoreGearAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.GearToShootBlue;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.ShootToLoadBlue;
import com.team254.frc2017.paths.StartToGearBlue;

public class GearThenShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToGearBlue();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        runAction(new ScoreGearAction());
        runAction(new DrivePathAction(new GearToShootBlue()));
        runAction(new BeginShootingAction());
        runAction(new WaitAction(3));
        runAction(new EndShootingAction());
        runAction(new DrivePathAction(new ShootToLoadBlue()));
    }
}
