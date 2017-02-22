package com.team254.frc2017.auto.modes;

import java.util.Arrays;

import com.team254.frc2017.Constants;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.GearToHopperRed;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToGearBlue;
import com.team254.frc2017.paths.StartToGearRed;

public class GearThenHopperShootModeRed extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToGearRed();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        System.out.println(RobotState.getInstance().getLatestFieldToVehicle());
        runAction(new WaitAction(0.25));
        Constants.kAutoLookAhead = 24.0;
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new DrivePathAction(new GearToHopperRed()),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("DeployIntake"), new DeployIntakeAction()
                    }))
                }))
        );
        runAction(new BeginShootingAction());
        runAction(new WaitAction(20));
    }
}
