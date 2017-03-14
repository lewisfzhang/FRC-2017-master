package com.team254.frc2017.auto.modes;


import java.util.Arrays;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.ActuateHopperAction;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.PrintDebugAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.SetFlywheelRPMAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.BoilerToHopperRed;
import com.team254.frc2017.paths.PathContainer;

import edu.wpi.first.wpilibj.Timer;

public class BoilerHopperShootModeRed extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        double start = Timer.getFPGATimestamp();
        
        PathContainer hopperPath = new BoilerToHopperRed();
        runAction(new ResetPoseFromPathAction(hopperPath));
        
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new SetFlywheelRPMAction(3500.0), //spin up flywheel to save time
                    new DrivePathAction(hopperPath), //drive to hopper
                    new SeriesAction(Arrays.asList(new Action[]{
                            //deploy intake early to save time (need to test to see if it messes with the path)
                            new WaitForPathMarkerAction("DeployIntake"), new DeployIntakeAction(true)
                    })),
                    new SeriesAction(Arrays.asList(new Action[]{
                            //actuate hopper and begin shooting once path finishes
                            new WaitForPathMarkerAction("PathFinished"), new ActuateHopperAction(true), new BeginShootingAction(), new PrintDebugAction("Shoot time: " + (Timer.getFPGATimestamp() - start))
                    }))
                }))
        );
        runAction(new WaitAction(15)); //keep shooting until auto ends
    }
}
