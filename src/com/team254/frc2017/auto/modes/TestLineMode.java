package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.TurnToHeadingAction;
import com.team254.frc2017.paths.TestArcPath;
import com.team254.frc2017.paths.TestLinePath;
import com.team254.lib.util.Rotation2d;

public class TestLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ResetPoseFromPathAction(new TestArcPath()));
        runAction(new DrivePathAction(new TestArcPath()));
    }
}
