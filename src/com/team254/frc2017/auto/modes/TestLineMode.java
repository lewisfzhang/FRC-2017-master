package com.team254.frc2017.auto.modes;

import java.util.Arrays;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ForceEndPathAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.StartToHopperCurve;
import com.team254.frc2017.subsystems.Superstructure;

public class TestLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseFromPathAction(new StartToHopperCurve()));
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new DrivePathAction(new StartToHopperCurve()),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("DeployIntake"), new DeployIntakeAction()
                    })),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("DeployIntake"), new WaitAction(1.5), new ForceEndPathAction()
                    })) 
                }))
        );
//        runAction(new BeginShootingAction());
//        runAction(new WaitAction(5));
    }
}
