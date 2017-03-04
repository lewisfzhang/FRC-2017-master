package com.team254.frc2017.auto.modes;

import java.util.Arrays;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.GearToHopperRed;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToHopperBlue;

public class GearThenHopperShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer hopperPath = new StartToHopperBlue();
        runAction(new ResetPoseFromPathAction(hopperPath));
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new DrivePathAction(new GearToHopperRed()),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("DeployIntake"), new DeployIntakeAction()
                    }))
                }))
        );
    }
}
