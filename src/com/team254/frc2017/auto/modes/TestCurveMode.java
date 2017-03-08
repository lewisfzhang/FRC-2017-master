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
import com.team254.frc2017.paths.TestCurve;
import com.team254.frc2017.subsystems.Superstructure;

public class TestCurveMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseFromPathAction(new TestCurve()));
        runAction(new DrivePathAction(new TestCurve()));
    }
}
