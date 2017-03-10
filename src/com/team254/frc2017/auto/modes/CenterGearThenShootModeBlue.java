package com.team254.frc2017.auto.modes;


import com.team254.frc2017.Constants;
import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.EndShootingAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.ScoreGearAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.paths.CenterGearToShootBlue;
import com.team254.frc2017.paths.GearToShootBlue;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.ShootToLoadBlue;
import com.team254.frc2017.paths.StartToCenterGearBlue;
import com.team254.frc2017.paths.StartToGearBlue;

public class CenterGearThenShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToCenterGearBlue();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        runAction(new DeployIntakeAction());
        runAction(new WaitAction(0.5));
        //runAction(new ScoreGearAction());
        Constants.kSegmentCompletionTolerance = 3.0;
        runAction(new DrivePathAction(new CenterGearToShootBlue()));
        runAction(new BeginShootingAction());
        runAction(new WaitAction(5));
        runAction(new EndShootingAction());
        //runAction(new DrivePathAction(new ShootToLoadBlue()));
    }
}
