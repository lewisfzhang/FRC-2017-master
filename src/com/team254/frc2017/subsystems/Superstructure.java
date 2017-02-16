package com.team254.frc2017.subsystems;

import com.team254.frc2017.loops.Looper;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = new Superstructure();

    public static Superstructure getInstance() {
        return mInstance;
    }

    private Feeder mFeeder = Feeder.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();


    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {

    }
}
