package com.team254.frc2017.auto.actions;


import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.subsystems.Drive;
import com.team254.lib.util.Path;
import com.team254.lib.util.Rotation2d;

public class TurnToHeadingAction implements Action {

    private Rotation2d mTargetHeading;
    private Drive mDrive = Drive.getInstance();

    public TurnToHeadingAction(Rotation2d heading) {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}