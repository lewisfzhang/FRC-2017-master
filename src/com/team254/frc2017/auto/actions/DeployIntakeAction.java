package com.team254.frc2017.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import com.team254.frc2017.subsystems.Intake;

public class DeployIntakeAction implements Action {

    Intake mIntake = Intake.getInstance();
    double startTime;
    boolean runIntake;

    public DeployIntakeAction() {
        runIntake = false;
    }

    public DeployIntakeAction(boolean runIntake) {
        this.runIntake = runIntake;
    }

    @Override
    public boolean isFinished() {
        if (runIntake) {
            return Timer.getFPGATimestamp() - startTime > 0.5;
        } else {
            return true;
        }
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        if (runIntake) {
            // mIntake.setOff();
        }
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        mIntake.deploy();
        if (runIntake)
            mIntake.setOn();
    }
}
