package com.team254.frc2017.auto.actions;

import com.team254.frc2017.RobotState;
import com.team254.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.Timer;

public class CorrectPoseAction extends RunOnceAction {
    RigidTransform2d mCorrection;
    
    public CorrectPoseAction(RigidTransform2d correction) {
        mCorrection = correction;
    }

    @Override
    public void runOnce() {
        RobotState rs = RobotState.getInstance();
        rs.reset(Timer.getFPGATimestamp(), rs.getLatestFieldToVehicle().getValue().transformBy(mCorrection));
    }

}
