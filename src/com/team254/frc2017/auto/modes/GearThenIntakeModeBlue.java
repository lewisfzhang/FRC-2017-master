package com.team254.frc2017.auto.modes;

import java.util.Arrays;

import com.team254.frc2017.Constants;
import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.ActuateHopperAction;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.EndShootingAction;
import com.team254.frc2017.auto.actions.ForceEndPathAction;
import com.team254.frc2017.auto.actions.LowerGearAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.PrintDebugAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.ScoreGearAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.SetFlywheelRPMAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.BoilerGearToHopperBlue;
import com.team254.frc2017.paths.BoilerGearToShootBlue;
import com.team254.frc2017.paths.GearToIntakeBlue;
import com.team254.frc2017.paths.GearToIntakeRed;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToBoilerGearBlue;
import com.team254.frc2017.paths.StartToBoilerGearRed;
import com.team254.frc2017.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;

public class GearThenIntakeModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer gearPath = new StartToBoilerGearRed();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new SetFlywheelRPMAction(3000.0), //spin up flywheel to save time
                    new DeployIntakeAction(),
                    new ScoreGearAction(),
                }))
        );  
        Intake.getInstance().setOn();
        runAction(new DrivePathAction(new GearToIntakeRed()));
        runAction(new BeginShootingAction());
        runAction(new WaitAction(15));
    }
}
