package com.team254.frc2017.auto.actions;

import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.subsystems.Drive;
import com.team254.lib.util.control.Path;

public class WaitForJoltAction implements Action {

    boolean isDone = false;
    Drive mDrive = Drive.getInstance();

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void update() {
        if (mDrive.getAccelX() < -1)
            isDone = true;
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }
}
