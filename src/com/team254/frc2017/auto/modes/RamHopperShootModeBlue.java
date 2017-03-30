package com.team254.frc2017.auto.modes;

import java.util.Arrays;

import com.team254.frc2017.Constants;
import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.BeginShootingAction;
import com.team254.frc2017.auto.actions.DeployIntakeAction;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ForceEndPathAction;
import com.team254.frc2017.auto.actions.OpenLoopUntilSeesTargetAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.PrintDebugAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.SetFlywheelRPMAction;
import com.team254.frc2017.auto.actions.TurnUntilSeesTargetAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.HopperToShootPathBlue;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToHopperBlue;
import com.team254.lib.util.math.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class RamHopperShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer hopperPath = new StartToHopperBlue();
        runAction(new ResetPoseFromPathAction(hopperPath));
        double startTime = Timer.getFPGATimestamp();
        runAction(
                new ParallelAction(Arrays.asList(new Action[]{
                    new DeployIntakeAction(true),
                    new DrivePathAction(hopperPath),
                    new SeriesAction(Arrays.asList(new Action[]{
                            new WaitForPathMarkerAction("RamWall"), new PrintDebugAction("RamWall"), new WaitAction(0.25), new ForceEndPathAction()
                    }))
                }))
        ); //Drive to hopper, cancel path once the robot runs into the wall
        runAction(new SetFlywheelRPMAction(3100));
        runAction(new WaitAction(2.6)); //wait for balls
        runAction(new OpenLoopUntilSeesTargetAction(-0.9, -0.02));
//        Constants.kSegmentCompletionTolerance = 5.0;
//        runAction(new DrivePathAction(new HopperToShootPathBlue())); //drive backwards to get off the wall
//        runAction(new TurnUntilSeesTargetAction(Rotation2d.fromDegrees(165))); //turn towards 165 degrees or until camera sees target
        System.out.println("Time to shoot: " + (Timer.getFPGATimestamp() - startTime));
        runAction(new BeginShootingAction()); //aim + fire
        runAction(new WaitAction(20)); //keep firing until auto ends
    }
}
