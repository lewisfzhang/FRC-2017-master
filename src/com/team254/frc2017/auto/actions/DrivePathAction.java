package com.team254.frc2017.auto.actions;


import com.team254.frc2017.RobotState;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.subsystems.Drive;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import edu.wpi.first.wpilibj.Timer;

public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private boolean mResetPose;
    private Drive mDrive = Drive.getInstance();

    DrivePathAction(PathContainer p, boolean resetRobotPose) {
        mPathContainer = p;
        mResetPose = resetRobotPose;
        mPath = mPathContainer.buildPath();
    }

    public DrivePathAction(PathContainer p) {
        this(p, false);
    }

    @Override
    public boolean isFinished() {
        // TODO: @mario: Implement isDoneWithPath in Drive.java that inspects APPC
        return false;
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        // TODO: Perhaps set wheel velocity to 0?
    }

    @Override
    public void start() {
        if (mResetPose) {
            RigidTransform2d startPose =  mPathContainer.getStartPose();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
            mDrive.setGyroAngle(startPose.getRotation());
        }
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
