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
import com.team254.frc2017.auto.actions.PrintDebugAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.SetFlywheelRPMAction;
import com.team254.frc2017.auto.actions.TurnDirectionUntilSeesTargetAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForJoltAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToHopperBlue;
import com.team254.frc2017.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.math.Rotation2d;

public class HopperShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer hopperPath = new StartToHopperBlue();
        runAction(new ResetPoseFromPathAction(hopperPath));
        //runAction(new DeployIntakeAction());
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new DrivePathAction(hopperPath),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("RamWall"), new PrintDebugAction("RamWall"), new WaitAction(0.05), new ForceEndPathAction()
                    }))
                }))
        );
        //runAction(new SetFlywheelRPMAction(3500));
        Drive.getInstance().setVelocitySetpoint(0, 0);
        runAction(new WaitAction(2.3));
        Drive.getInstance().setOpenLoop(new DriveSignal(-1, -1));
        runAction(new WaitAction(0.15));
        runAction(new TurnDirectionUntilSeesTargetAction(Rotation2d.fromDegrees(165)));
        runAction(new BeginShootingAction());
        runAction(new WaitAction(20));

        
    }
}
