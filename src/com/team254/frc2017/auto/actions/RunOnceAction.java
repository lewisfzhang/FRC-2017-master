package com.team254.frc2017.auto.actions;

public abstract class RunOnceAction implements Action {
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        runOnce();
    }

    public abstract void runOnce();
}
